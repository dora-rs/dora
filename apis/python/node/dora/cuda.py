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
