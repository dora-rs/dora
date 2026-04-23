"""CUDA memory utilities for dora-rs dataflow nodes.

Provides helper functions for managing CUDA Inter-Process Communication (IPC)
memory handles. These utilities enable zero-copy GPU data transfer between
dora-rs nodes using PyTorch, facilitating high-performance robotic
applications.

Uses ctypes to call the CUDA runtime API directly, avoiding the need for
the numba package.
"""

import ctypes
import functools
import sys
from contextlib import contextmanager
from typing import ContextManager

import numpy as np
import pyarrow as pa
import torch

# ---------------------------------------------------------------------------
# CUDA runtime bindings via ctypes
# ---------------------------------------------------------------------------

# Constants for CUDA operations
CUDA_IPC_HANDLE_SIZE = 64
_cudaIpcMemLazyEnablePeerAccess = 1
_cudaHostRegisterDefault = 0
_cudaMemcpyHostToDevice = 1


class _CudaIpcMemHandle(ctypes.Structure):
    _fields_ = [("reserved", ctypes.c_byte * CUDA_IPC_HANDLE_SIZE)]


@functools.lru_cache(maxsize=1)
def _libcudart():
    # Resolve the CUDA runtime lazily so non-CUDA hosts (and macOS/Windows
    # importers that never touch IPC) can `import dora.cuda` without blowing
    # up at module load. Raises a descriptive RuntimeError on failure.
    name = {"linux": "libcudart.so", "darwin": "libcudart.dylib"}.get(
        sys.platform, "cudart64_12.dll"
    )
    try:
        lib = ctypes.CDLL(name)
    except OSError as e:
        raise RuntimeError(
            f"failed to load CUDA runtime ({name}): {e}. "
            "CUDA IPC requires a working CUDA installation."
        ) from e
    lib.cudaIpcGetMemHandle.restype = ctypes.c_int
    lib.cudaIpcGetMemHandle.argtypes = [
        ctypes.POINTER(_CudaIpcMemHandle),
        ctypes.c_void_p,
    ]
    lib.cudaIpcOpenMemHandle.restype = ctypes.c_int
    lib.cudaIpcOpenMemHandle.argtypes = [
        ctypes.POINTER(ctypes.c_void_p),
        _CudaIpcMemHandle,
        ctypes.c_uint,
    ]
    lib.cudaIpcCloseMemHandle.restype = ctypes.c_int
    lib.cudaIpcCloseMemHandle.argtypes = [ctypes.c_void_p]
    lib.cudaDeviceSynchronize.restype = ctypes.c_int
    lib.cudaDeviceSynchronize.argtypes = []

    # Additional CUDA function declarations for pinned memory operations
    lib.cudaHostRegister.restype = ctypes.c_int
    lib.cudaHostRegister.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint]

    lib.cudaHostUnregister.restype = ctypes.c_int
    lib.cudaHostUnregister.argtypes = [ctypes.c_void_p]

    lib.cudaHostGetDevicePointer.restype = ctypes.c_int
    lib.cudaHostGetDevicePointer.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_void_p, ctypes.c_uint]

    lib.cudaMemcpyAsync.restype = ctypes.c_int
    lib.cudaMemcpyAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_void_p]

    lib.cudaStreamCreate.restype = ctypes.c_int
    lib.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]

    lib.cudaStreamSynchronize.restype = ctypes.c_int
    lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]

    lib.cudaStreamDestroy.restype = ctypes.c_int
    lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]

    lib.cudaMemcpy.restype = ctypes.c_int
    lib.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]

    return lib

# CUDA error codes
CUDA_ERROR_SUCCESS = 0
CUDA_ERROR_HOST_MEMORY_ALREADY_REGISTERED = 712
CUDA_ERROR_HOST_MEMORY_NOT_REGISTERED = 713
# Additional error codes for better error messages
CUDA_ERROR_INVALID_VALUE = 1
CUDA_ERROR_OUT_OF_MEMORY = 2
CUDA_ERROR_NOT_INITIALIZED = 3
CUDA_ERROR_DEINITIALIZED = 4
CUDA_ERROR_NO_DEVICE = 100
CUDA_ERROR_INVALID_DEVICE = 101

# CUDA constants
BYTE_SIGNED_BOUNDARY = 128
BYTE_WRAP_AROUND = 256
CUDA_STREAM_DEFAULT = 0

# Numpy dtype string -> torch dtype mapping.
_DTYPE_MAP = {
    # Numpy dtype strings
    "<i8": torch.int64,
    "<i4": torch.int32,
    "<i2": torch.int16,
    "<f4": torch.float32,
    "<f8": torch.float64,
    "<f2": torch.float16,
    "<u1": torch.uint8,
    "|b1": torch.bool,
    # Simple type names
    "int64": torch.int64,
    "int32": torch.int32,
    "int16": torch.int16,
    "float32": torch.float32,
    "float64": torch.float64,
    "float16": torch.float16,
    "uint8": torch.uint8,
    "bool": torch.bool,
    # Torch type names
    "torch.int64": torch.int64,
    "torch.int32": torch.int32,
    "torch.int16": torch.int16,
    "torch.float32": torch.float32,
    "torch.float64": torch.float64,
    "torch.float16": torch.float16,
    "torch.uint8": torch.uint8,
    "torch.bool": torch.bool,
    "torch.int8": torch.int8,
    "torch.bfloat16": torch.bfloat16,
}

# Torch dtype -> numpy dtype mapping
_TORCH_TO_NUMPY_DTYPE_MAP = {
    torch.int64: np.int64,
    torch.float32: np.float32,
    torch.float64: np.float64,
    torch.int32: np.int32,
    torch.int16: np.int16,
    torch.int8: np.int8,
    torch.uint8: np.uint8,
    torch.bool: np.bool_,
    torch.float16: np.float16,
    torch.bfloat16: np.float16,  # Note: bfloat16 maps to float16 in numpy
}


def _check(err, msg="CUDA call failed"):
    if err != CUDA_ERROR_SUCCESS:
        raise RuntimeError(f"{msg} (cudaError={err}: {_cuda_error_string(err)})")

def _cuda_error_string(err_code):
    """Convert CUDA error code to human-readable string."""
    error_map = {
        CUDA_ERROR_SUCCESS: "Success",
        CUDA_ERROR_INVALID_VALUE: "Invalid value",
        CUDA_ERROR_OUT_OF_MEMORY: "Out of memory",
        CUDA_ERROR_NOT_INITIALIZED: "CUDA runtime not initialized",
        CUDA_ERROR_DEINITIALIZED: "CUDA runtime deinitialized",
        CUDA_ERROR_NO_DEVICE: "No CUDA-capable device available",
        CUDA_ERROR_INVALID_DEVICE: "Invalid device",
        CUDA_ERROR_HOST_MEMORY_ALREADY_REGISTERED: "Host memory already registered",
        CUDA_ERROR_HOST_MEMORY_NOT_REGISTERED: "Host memory not registered",
    }
    return error_map.get(err_code, f"Unknown error code: {err_code}")


# ---------------------------------------------------------------------------
# Public API — same signatures as the previous numba-based implementation
# ---------------------------------------------------------------------------


def torch_to_ipc_buffer(tensor: torch.Tensor) -> tuple[pa.array, dict]:
    """Convert a PyTorch CUDA tensor into a pyarrow buffer containing the
    IPC handle and its metadata.

    Example::

        torch_tensor = torch.tensor(data, dtype=torch.int64, device="cuda")
        ipc_buffer, metadata = torch_to_ipc_buffer(torch_tensor)
        node.send_output("latency", ipc_buffer, metadata)
    """
    if not tensor.is_cuda:
        raise ValueError("tensor must be on a CUDA device")

    handle = _CudaIpcMemHandle()
    _check(
        _libcudart().cudaIpcGetMemHandle(
            ctypes.byref(handle), ctypes.c_void_p(tensor.data_ptr())
        ),
        "cudaIpcGetMemHandle",
    )

    # Serialize handle as signed int8 list (pa.int8 range is -128..127).
    handle_bytes = bytes(handle)
    signed = [(b if b < BYTE_SIGNED_BOUNDARY else b - BYTE_WRAP_AROUND) for b in handle_bytes]

    metadata = {
        "shape": tuple(tensor.shape),
        "strides": tuple(s * tensor.element_size() for s in tensor.stride()),
        "dtype": np.dtype(
            torch.zeros(1, dtype=tensor.dtype).numpy().dtype
        ).str,
        "size": tensor.nelement() * tensor.element_size(),
        "offset": 0,
    }
    return pa.array(signed, type=pa.int8()), metadata


class IpcHandle:
    """Lightweight wrapper around a CUDA IPC memory handle."""

    def __init__(self, handle_bytes: bytes, size: int, offset: int):
        self._handle_bytes = handle_bytes
        self._size = size
        self._offset = offset
        self._d_ptr = None

    def open(self):
        """Open the IPC handle and return the device pointer value."""
        lib = _libcudart()
        handle = _CudaIpcMemHandle()
        ctypes.memmove(ctypes.byref(handle), self._handle_bytes, CUDA_IPC_HANDLE_SIZE)
        self._d_ptr = ctypes.c_void_p()
        _check(
            lib.cudaIpcOpenMemHandle(
                ctypes.byref(self._d_ptr),
                handle,
                _cudaIpcMemLazyEnablePeerAccess,
            ),
            "cudaIpcOpenMemHandle",
        )
        _check(lib.cudaDeviceSynchronize(), "cudaDeviceSynchronize")
        return self._d_ptr.value

    def close(self):
        """Close the IPC handle mapping."""
        if self._d_ptr is not None and self._d_ptr.value is not None:
            _check(
                _libcudart().cudaIpcCloseMemHandle(self._d_ptr),
                "cudaIpcCloseMemHandle",
            )
            self._d_ptr = None


def ipc_buffer_to_ipc_handle(handle_buffer: pa.array, metadata: dict) -> IpcHandle:
    """Convert a buffer containing a serialized IPC handle back into an
    :class:`IpcHandle`.

    Example::

        from dora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle

        event = node.next()
        ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"])
        with open_ipc_handle(ipc_handle, event["metadata"]) as tensor:
            pass
    """
    signed = handle_buffer.to_pylist()
    handle_bytes = bytes([(b + BYTE_WRAP_AROUND) % BYTE_WRAP_AROUND for b in signed])
    return IpcHandle(
        handle_bytes,
        metadata["size"],
        metadata.get("offset", 0),
    )


class _ArrayInterface:
    """Minimal object implementing ``__array_interface__`` so that
    ``torch.as_tensor`` can wrap raw CPU memory as a tensor (zero-copy)."""

    def __init__(self, ptr, shape, strides, dtype_str):
        self.__array_interface__ = {
            "shape": tuple(shape),
            "strides": tuple(strides) if strides else None,
            "typestr": dtype_str,
            "data": (ptr, False),
            "version": 3,
        }


class _CudaArrayInterface:
    """Minimal object implementing ``__cuda_array_interface__`` so that
    ``torch.as_tensor`` can wrap raw GPU memory as a tensor (zero-copy)."""

    def __init__(self, ptr, shape, strides, dtype_str):
        self.__cuda_array_interface__ = {
            "shape": tuple(shape),
            "strides": tuple(strides) if strides else None,
            "typestr": dtype_str,
            "data": (ptr, False),
            "version": 3,
        }


@contextmanager
def open_ipc_handle(
    ipc_handle: IpcHandle, metadata: dict
) -> ContextManager[torch.Tensor]:
    """Open a CUDA IPC handle and yield a PyTorch tensor backed by the
    shared GPU memory (zero-copy).

    Example::

        from dora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle

        event = node.next()
        ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"])
        with open_ipc_handle(ipc_handle, event["metadata"]) as tensor:
            pass
    """
    shape = metadata["shape"]
    strides = metadata.get("strides")
    dtype_str = metadata["dtype"]

    try:
        ptr = ipc_handle.open()
        ptr += ipc_handle._offset

        wrapper = _CudaArrayInterface(ptr, shape, strides, dtype_str)
        tensor = torch.as_tensor(wrapper, device="cuda")

        yield tensor
    finally:
        ipc_handle.close()

# ---------------------------------------------------------------------------
# Pinned memory API
# ---------------------------------------------------------------------------

def torch_to_ptr(tensor: torch.Tensor) -> tuple[pa.array, dict]:
    """
    只负责将tensor解析为当前进程的数据指针和metadata。

    根据指针位置判断设备：CPU或CUDA。
    """
    if not tensor.is_contiguous():
        tensor = tensor.contiguous()

    # Get the pointer
    ptr = tensor.data_ptr()

    metadata = {
        "size": tensor.nbytes,
        "dtype": str(tensor.dtype),
        "shape": list(tensor.shape),
        "is_cuda": tensor.is_cuda,
        "device": str(tensor.device),
    }
    # Wrap pointer in a list to create pyarrow array
    return pa.array([ptr]), metadata

def ptr_to_torch(data_ptr, metadata) -> torch.Tensor:
    """
    只负责根据数据指针生成tensor，零拷贝。

    根据指针位置判断设备：如果指针在CPU上，生成CPU tensor；
    如果指针在CUDA上，生成CUDA tensor。
    """
    # Extract pointer value from pyarrow array
    # data_ptr is expected to be a pyarrow array containing a single integer (pointer)
    ptr_list = data_ptr.to_pylist()
    if len(ptr_list) != 1:
        raise ValueError(f"Expected data_ptr array with one element, got {len(ptr_list)}")
    ptr = ptr_list[0]
    if ptr == 0:
        size = metadata.get("size", 0)
        if size == 0:
            # Read-after-free case: return an empty tensor
            shape = metadata.get("shape", [0])
            return torch.empty(*shape, dtype=torch.int64)
        raise ValueError("Invalid pointer (NULL)")

    # Get metadata
    dtype_str = metadata.get("dtype", "int64")
    shape = metadata.get("shape", [])
    is_cuda = metadata.get("is_cuda", False)
    device_str = metadata.get("device", "cpu")
    size = metadata.get("size")

    dtype = _DTYPE_MAP.get(dtype_str, torch.int64)

    # Convert torch dtype to numpy dtype string (typestr)
    if dtype not in _TORCH_TO_NUMPY_DTYPE_MAP:
        raise ValueError(f"Unsupported dtype: {dtype}")
    np_dtype = _TORCH_TO_NUMPY_DTYPE_MAP[dtype]
    typestr = np.dtype(np_dtype).str

    if is_cuda:
        # CUDA tensor - use zero-copy via _CudaArrayInterface
        if not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available for CUDA tensor")

        wrapper = _CudaArrayInterface(ptr, shape, None, typestr)
        tensor = torch.as_tensor(wrapper, device="cuda")
        return tensor
    else:
        # CPU pointer - try to get device pointer for zero-copy CUDA access
        if torch.cuda.is_available():
            lib = _libcudart()
            host_ptr = ctypes.c_void_p(ptr)
            device_ptr = ctypes.c_void_p()
            result = lib.cudaHostGetDevicePointer(ctypes.byref(device_ptr), host_ptr, 0)
            if result == CUDA_ERROR_SUCCESS:
                # Success! Create CUDA tensor using device pointer
                try:
                    wrapper = _CudaArrayInterface(device_ptr.value, shape, None, typestr)
                    tensor = torch.as_tensor(wrapper, device="cuda")
                    return tensor
                except RuntimeError:
                    # CUDA tensor creation failed, fall back to CPU
                    pass

        # Fallback: CPU tensor - create via numpy array (zero-copy)
        if size is None:
            # Calculate size from shape and dtype
            nelement = 1
            for dim in shape:
                nelement *= dim
            size = nelement * torch.tensor([], dtype=dtype).element_size()

        # Create ctypes array pointing to the memory
        c_array = (ctypes.c_byte * size).from_address(ptr)

        # For bfloat16, numpy doesn't have native support, use torch directly
        if dtype == torch.bfloat16:
            # Create from byte buffer, then view as bfloat16
            byte_tensor = torch.frombuffer(c_array, dtype=torch.uint8)
            tensor = byte_tensor.view(dtype=torch.bfloat16).reshape(shape)
            return tensor

        # Create numpy array from buffer (zero-copy)
        np_array = np.frombuffer(c_array, dtype=np_dtype).reshape(shape)
        # Convert to torch tensor (zero-copy, shares memory with numpy array)
        tensor = torch.from_numpy(np_array)
        return tensor


