"""CUDA memory utilities for dora-rs dataflow nodes.

Provides helper functions for managing CUDA Inter-Process Communication (IPC)
memory handles. These utilities enable zero-copy GPU data transfer between
dora-rs nodes using PyTorch, facilitating high-performance robotic
applications.

Uses ctypes to call the CUDA runtime API directly, avoiding the need for
the numba package.
"""


import ctypes
from contextlib import contextmanager
from typing import ContextManager

import numpy as np
import pyarrow as pa
import torch

import uuid
import sys
from multiprocessing.shared_memory import SharedMemory

# ---------------------------------------------------------------------------
# CUDA runtime bindings via ctypes
# ---------------------------------------------------------------------------
_libcudart = ctypes.CDLL("libcudart.so")



class _CudaIpcMemHandle(ctypes.Structure):
    _fields_ = [("reserved", ctypes.c_byte * 64)]


_libcudart.cudaIpcGetMemHandle.restype = ctypes.c_int
_libcudart.cudaIpcGetMemHandle.argtypes = [
    ctypes.POINTER(_CudaIpcMemHandle),
    ctypes.c_void_p,
]
_libcudart.cudaIpcOpenMemHandle.restype = ctypes.c_int
_libcudart.cudaIpcOpenMemHandle.argtypes = [
    ctypes.POINTER(ctypes.c_void_p),
    _CudaIpcMemHandle,
    ctypes.c_uint,
]
_libcudart.cudaIpcCloseMemHandle.restype = ctypes.c_int
_libcudart.cudaIpcCloseMemHandle.argtypes = [ctypes.c_void_p]
_libcudart.cudaDeviceSynchronize.restype = ctypes.c_int
_libcudart.cudaDeviceSynchronize.argtypes = []

_cudaIpcMemLazyEnablePeerAccess = 1

# Additional CUDA function declarations for pinned memory operations
_libcudart.cudaHostRegister.restype = ctypes.c_int
_libcudart.cudaHostRegister.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint]

_libcudart.cudaHostUnregister.restype = ctypes.c_int
_libcudart.cudaHostUnregister.argtypes = [ctypes.c_void_p]

_libcudart.cudaMemcpyAsync.restype = ctypes.c_int
_libcudart.cudaMemcpyAsync.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_void_p]

_libcudart.cudaStreamCreate.restype = ctypes.c_int
_libcudart.cudaStreamCreate.argtypes = [ctypes.POINTER(ctypes.c_void_p)]

_libcudart.cudaStreamSynchronize.restype = ctypes.c_int
_libcudart.cudaStreamSynchronize.argtypes = [ctypes.c_void_p]

_libcudart.cudaStreamDestroy.restype = ctypes.c_int
_libcudart.cudaStreamDestroy.argtypes = [ctypes.c_void_p]

_libcudart.cudaMemcpy.restype = ctypes.c_int
_libcudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]

# Constants for CUDA operations
_cudaHostRegisterDefault = 0
_cudaMemcpyHostToDevice = 1

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
}


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
        _libcudart.cudaIpcGetMemHandle(
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
        handle = _CudaIpcMemHandle()
        ctypes.memmove(ctypes.byref(handle), self._handle_bytes, 64)
        self._d_ptr = ctypes.c_void_p()
        _check(
            _libcudart.cudaIpcOpenMemHandle(
                ctypes.byref(self._d_ptr),
                handle,
                _cudaIpcMemLazyEnablePeerAccess,
            ),
            "cudaIpcOpenMemHandle",
        )
        _libcudart.cudaDeviceSynchronize()
        return self._d_ptr.value

    def close(self):
        """Close the IPC handle mapping."""
        if self._d_ptr is not None and self._d_ptr.value is not None:
            _libcudart.cudaIpcCloseMemHandle(self._d_ptr)
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

def torch_to_pinned_buffer(tensor: torch.Tensor) -> tuple[pa.array, dict]:
    """
    Convert a CPU torch tensor to a pinned memory buffer.
    Returns a pyarrow buffer containing the shared memory identifier and metadata.
    """
    if tensor.is_cuda:
        raise ValueError("Tensor must be on CPU for pinned memory allocation")

    size = tensor.nbytes
    shm_name = f"dora_pinned_{uuid.uuid4()}"
    shm = SharedMemory(name=shm_name, create=True, size=size)

    try:
        import multiprocessing.resource_tracker
        multiprocessing.resource_tracker.unregister(shm._name, 'shared_memory')
    except:
        pass

    if not tensor.is_contiguous():
        tensor = tensor.contiguous()

    mv = shm.buf
    temp_arr = (ctypes.c_char * size).from_buffer(mv)
    dst_ptr = ctypes.addressof(temp_arr)

    np_array = tensor.numpy()
    dtype_map = {
        torch.int64: np.int64,
        torch.float32: np.float32,
        torch.float64: np.float64,
        torch.int32: np.int32,
        torch.int16: np.int16,
        torch.int8: np.int8,
        torch.uint8: np.uint8,
        torch.bool: np.bool_,
        torch.float16: np.float16,
    }
    np_dtype = dtype_map.get(tensor.dtype, np.int64)
    shm_np = np.frombuffer(mv, dtype=np_dtype).reshape(tensor.shape)
    np.copyto(shm_np, np_array)

    del shm_np
    del np_array
    del temp_arr

    buffer_id = shm_name.encode('ascii')
    buffer_array = pa.array([buffer_id], type=pa.binary())

    metadata = {
        "ptr": dst_ptr,
        "size": size,
        "dtype": str(tensor.dtype),
        "shape": list(tensor.shape),
        "shared_memory_name": shm_name,
    }
    return buffer_array, metadata

def pinned_buffer_to_torch(memory_buffer: dict) -> torch.Tensor:
    """
    Convert a pinned memory buffer to a CUDA tensor using DMA transfer.

    Args:
        memory_buffer: Dictionary containing metadata (shared_memory_name, size, dtype, shape).

    Returns:
        torch.Tensor on CUDA device.
    """
    # Extract metadata
    size = memory_buffer.get("size", 0)
    dtype_str = memory_buffer.get("dtype", "int64")
    shape = memory_buffer.get("shape", [])
    shared_memory_name = memory_buffer.get("shared_memory_name", None)

    if size == 0 or not shape:
        raise ValueError("Invalid memory buffer metadata")

    # Convert dtype string to torch dtype
    dtype_map = {
        "int64": torch.int64,
        "torch.int64": torch.int64,
        "float32": torch.float32,
        "torch.float32": torch.float32,
        "float64": torch.float64,
        "torch.float64": torch.float64,
        "int32": torch.int32,
        "torch.int32": torch.int32,
        "int16": torch.int16,
        "torch.int16": torch.int16,
        "int8": torch.int8,
        "torch.int8": torch.int8,
        "uint8": torch.uint8,
        "torch.uint8": torch.uint8,
        "bool": torch.bool,
        "torch.bool": torch.bool,
        "float16": torch.float16,
        "torch.float16": torch.float16,
        "bfloat16": torch.bfloat16,
        "torch.bfloat16": torch.bfloat16,
    }
    dtype = dtype_map.get(dtype_str, torch.int64)

    # Check if CUDA is available
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is not available")

    # Allocate GPU memory
    gpu_tensor = torch.empty(shape, dtype=dtype, device="cuda")

    # Open shared memory
    if not shared_memory_name:
        raise ValueError("shared_memory_name is required")

    shm = None
    host_ptr = None
    memory_registered_by_me = False

    try:
        shm = SharedMemory(name=shared_memory_name, create=False)

        # Get pointer to shared memory buffer
        mv = shm.buf
        if hasattr(mv, '__array_interface__'):
            array_info = mv.__array_interface__
            host_ptr = ctypes.c_void_p(array_info['data'][0])
        else:
            temp_arr = (ctypes.c_char * size).from_buffer(mv)
            host_ptr = ctypes.c_void_p(ctypes.addressof(temp_arr))
            del temp_arr

        # Register shared memory as pinned memory for CUDA DMA
        result = _libcudart.cudaHostRegister(host_ptr, size, _cudaHostRegisterDefault)
        if result == 0:
            memory_registered_by_me = True
        elif result != 712:  # CUDA_ERROR_HOST_MEMORY_ALREADY_REGISTERED is OK
            # Non-critical error, continue without pinned memory
            pass

        # Copy data from pinned memory to GPU using asynchronous DMA
        gpu_ptr = ctypes.c_void_p(gpu_tensor.data_ptr())
        stream = ctypes.c_void_p(0)

        result = _libcudart.cudaStreamCreate(ctypes.byref(stream))
        if result == 0:
            # Asynchronous copy with DMA
            result = _libcudart.cudaMemcpyAsync(
                gpu_ptr, host_ptr, size, _cudaMemcpyHostToDevice, stream
            )
            if result != 0:
                _libcudart.cudaStreamDestroy(stream)
                raise RuntimeError(f"cudaMemcpyAsync failed with error code {result}")

            _libcudart.cudaStreamSynchronize(stream)
            _libcudart.cudaStreamDestroy(stream)
        else:
            # Fallback to synchronous copy
            result = _libcudart.cudaMemcpy(
                gpu_ptr, host_ptr, size, _cudaMemcpyHostToDevice
            )
            if result != 0:
                raise RuntimeError(f"cudaMemcpy failed with error code {result}")

        return gpu_tensor

    finally:
        # Unregister CUDA pinned memory if we registered it
        if memory_registered_by_me and host_ptr is not None:
            try:
                _libcudart.cudaHostUnregister(host_ptr)
            except:
                pass  # Ignore errors during cleanup

        # Close shared memory
        if shm is not None:
            shm.close()