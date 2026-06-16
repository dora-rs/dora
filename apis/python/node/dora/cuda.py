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


class _CudaIpcMemHandle(ctypes.Structure):
    _fields_ = [("reserved", ctypes.c_byte * 64)]


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
    lib.cudaHostRegister.argtypes = [
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_uint,
    ]

    lib.cudaHostUnregister.restype = ctypes.c_int
    lib.cudaHostUnregister.argtypes = [ctypes.c_void_p]

    lib.cudaHostGetDevicePointer.restype = ctypes.c_int
    lib.cudaHostGetDevicePointer.argtypes = [
        ctypes.POINTER(ctypes.c_void_p),
        ctypes.c_void_p,
        ctypes.c_uint,
    ]

    lib.cudaMemcpyAsync.restype = ctypes.c_int
    lib.cudaMemcpyAsync.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_int,
        ctypes.c_void_p,
    ]

    lib.cudaStreamCreate.restype = ctypes.c_int
    lib.cudaStreamCreate.argtypes = [
        ctypes.POINTER(ctypes.c_void_p),
    ]

    lib.cudaStreamSynchronize.restype = ctypes.c_int
    lib.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]

    lib.cudaStreamDestroy.restype = ctypes.c_int
    lib.cudaStreamDestroy.argtypes = [ctypes.c_void_p]

    lib.cudaMemcpy.restype = ctypes.c_int
    lib.cudaMemcpy.argtypes = [
        ctypes.c_void_p,
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_int,
    ]

    return lib

_cudaIpcMemLazyEnablePeerAccess = 1

# Numpy dtype string -> torch dtype mapping.
_DTYPE_MAP = {
    "<i8": torch.int64,
    "<i4": torch.int32,
    "<i2": torch.int16,
    "<f4": torch.float32,
    "<f8": torch.float64,
    "<f2": torch.float16,
    "<u1": torch.uint8,
    "|b1": torch.bool,
    "int64": torch.int64,
    "int32": torch.int32,
    "int16": torch.int16,
    "float32": torch.float32,
    "float64": torch.float64,
    "float16": torch.float16,
    "uint8": torch.uint8,
    "bool": torch.bool,
    # Torch type names (used by tensor_from_info)
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

# Torch dtype -> numpy dtype mapping (used by tensor_from_info for
# constructing numpy arrays from raw memory pointers).
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
    torch.bfloat16: np.float16,  # bfloat16 maps to float16 in numpy
}

# CUDA error codes for better error messages
_CUDA_ERROR_SUCCESS = 0
_CUDA_ERROR_HOST_MEMORY_ALREADY_REGISTERED = 712
_CUDA_ERROR_HOST_MEMORY_NOT_REGISTERED = 713
_CUDA_ERROR_INVALID_VALUE = 1
_CUDA_ERROR_OUT_OF_MEMORY = 2
_CUDA_ERROR_NOT_INITIALIZED = 3
_CUDA_ERROR_DEINITIALIZED = 4
_CUDA_ERROR_NO_DEVICE = 100
_CUDA_ERROR_INVALID_DEVICE = 101


def _cuda_error_string(err_code):
    """Convert CUDA error code to human-readable string."""
    error_map = {
        _CUDA_ERROR_SUCCESS: "Success",
        _CUDA_ERROR_INVALID_VALUE: "Invalid value",
        _CUDA_ERROR_OUT_OF_MEMORY: "Out of memory",
        _CUDA_ERROR_NOT_INITIALIZED: "CUDA runtime not initialized",
        _CUDA_ERROR_DEINITIALIZED: "CUDA runtime deinitialized",
        _CUDA_ERROR_NO_DEVICE: "No CUDA-capable device available",
        _CUDA_ERROR_INVALID_DEVICE: "Invalid device",
        _CUDA_ERROR_HOST_MEMORY_ALREADY_REGISTERED: "Host memory already registered",
        _CUDA_ERROR_HOST_MEMORY_NOT_REGISTERED: "Host memory not registered",
    }
    return error_map.get(err_code, f"Unknown error code: {err_code}")


def _check(err, msg="CUDA call failed"):
    if err != 0:
        raise RuntimeError(f"{msg} (cudaError={err})")


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
    signed = [(b if b < 128 else b - 256) for b in handle_bytes]

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
        ctypes.memmove(ctypes.byref(handle), self._handle_bytes, 64)
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
    handle_bytes = bytes([(b + 256) % 256 for b in signed])
    return IpcHandle(
        handle_bytes,
        metadata["size"],
        metadata.get("offset", 0),
    )


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
# Tensor info helpers for memory-pool operations
# ---------------------------------------------------------------------------


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


def get_tensor_info(tensor: torch.Tensor) -> dict:
    """Serialize a tensor into a ``tensor_info`` dict containing pointer,
    size, dtype, shape, and device.

    This is the canonical way to pass tensor metadata to memory-pool
    operations such as ``register_memory_pool`` and ``write_memory_pool``.
    """
    if not tensor.is_contiguous():
        tensor = tensor.contiguous()
    return {
        "ptr": tensor.data_ptr(),
        "size": tensor.nbytes,
        "dtype": str(tensor.dtype),
        "shape": list(tensor.shape),
        "device": str(tensor.device),
    }


def tensor_from_info(tensor_info: dict) -> torch.Tensor:
    """Reconstruct a PyTorch tensor from a ``tensor_info`` dict (zero-copy).

    The returned tensor shares the same underlying memory as the original
    tensor that produced the ``tensor_info``.  Used by consumers that read
    a memory pool via ``read_memory_pool``.
    """
    ptr = tensor_info.get('ptr', 0)
    if ptr == 0:
        raise ValueError("tensor_info has null pointer (ptr=0); pool may not exist or has been freed")
    dtype_str = tensor_info["dtype"]
    shape = tensor_info["shape"]
    device = tensor_info.get("device", "cpu")
    size = tensor_info.get("size", 0)

    dtype = _DTYPE_MAP.get(dtype_str, torch.int64)

    if device.startswith("cuda"):
        # CUDA tensor — zero-copy via __cuda_array_interface__
        np_dtype = _TORCH_TO_NUMPY_DTYPE_MAP.get(dtype)
        if np_dtype is None:
            raise ValueError(f"Unsupported dtype: {dtype}")

        # Validate that product(shape) * itemsize(dtype) does not
        # exceed the registered size — a peer-controlled header that
        # claims a large shape over a small buffer would produce an
        # out-of-bounds GPU tensor (the CPU path is saved by numpy
        # reshape, but the GPU path has no equivalent backstop).
        expected_bytes = np.dtype(np_dtype).itemsize
        for dim in shape:
            expected_bytes *= dim
        if expected_bytes > size:
            raise ValueError(
                f"tensor shape {shape} * {np_dtype} itemsize = {expected_bytes} bytes "
                f"exceeds registered size {size} bytes — header may be corrupted"
            )

        typestr = np.dtype(np_dtype).str
        wrapper = _CudaArrayInterface(ptr, shape, None, typestr)
        return torch.as_tensor(wrapper, device="cuda")
    else:
        # CPU tensor — zero-copy via numpy / torch.frombuffer
        np_dtype = _TORCH_TO_NUMPY_DTYPE_MAP.get(dtype)
        if np_dtype is None and dtype != torch.bfloat16:
            raise ValueError(f"Unsupported dtype: {dtype}")

        c_array = (ctypes.c_byte * size).from_address(ptr)

        if dtype == torch.bfloat16:
            byte_tensor = torch.frombuffer(c_array, dtype=torch.uint8)
            return byte_tensor.view(dtype=torch.bfloat16).reshape(shape)

        np_array = np.frombuffer(c_array, dtype=np_dtype).reshape(shape)
        return torch.from_numpy(np_array)
