"""Pool-side tensor helpers, parked out of `dora/cuda.py`.

`get_tensor_info` / `tensor_from_info` were the documented way to feed the
removed `register_tensor_pool` / `write_tensor_pool` / `read_tensor_pool`
methods, and the two dtype maps existed only to serve them. They have no
other caller in dora, so they travelled with the transport.

`_CudaArrayInterface` lives in `cuda.py` alongside these — `open_ipc_handle` still uses
it. Reinstating these belongs on the dora-pool side of the seam, not in the
`dora` package. See ../README.md.
"""

import ctypes

import numpy as np
import torch

from .cuda import _CudaArrayInterface

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


# ---------------------------------------------------------------------------
# Tensor info helpers for tensor-pool operations
# ---------------------------------------------------------------------------


# DLPack dtype codes: kDLInt=0, kDLUInt=1, kDLFloat=2, kDLBfloat=4.
_DLPACK_TYPE = {
    torch.int8: (0, 8),
    torch.int16: (0, 16),
    torch.int32: (0, 32),
    torch.int64: (0, 64),
    torch.uint8: (1, 8),
    torch.float16: (2, 16),
    torch.float32: (2, 32),
    torch.float64: (2, 64),
    torch.bool: (1, 8),
    torch.bfloat16: (4, 16),
}


class _DLDevice(ctypes.Structure):
    _fields_ = [("device_type", ctypes.c_int32), ("device_id", ctypes.c_int32)]


class _DLDataType(ctypes.Structure):
    _fields_ = [
        ("code", ctypes.c_uint8),
        ("bits", ctypes.c_uint8),
        ("lanes", ctypes.c_uint16),
    ]


class _DLTensor(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.c_void_p),
        ("device", _DLDevice),
        ("ndim", ctypes.c_int32),
        ("dtype", _DLDataType),
        ("shape", ctypes.POINTER(ctypes.c_int64)),
        ("strides", ctypes.POINTER(ctypes.c_int64)),
        ("byte_offset", ctypes.c_uint64),
    ]


class _DLManagedTensor(ctypes.Structure):
    pass


_DLPACK_DELETER = ctypes.CFUNCTYPE(None, ctypes.POINTER(_DLManagedTensor))

_DLManagedTensor._fields_ = [
    ("dl_tensor", _DLTensor),
    ("manager_ctx", ctypes.c_void_p),
    ("deleter", _DLPACK_DELETER),
]


_NPU_DLPACK_DEVICE_TYPE = None
_DLPACK_HOLDERS = {}


def _npu_device_index(device: str) -> int:
    if device == "npu":
        return 0
    if device.startswith("npu:"):
        return int(device.split(":", 1)[1])
    raise ValueError(f"invalid NPU device `{device}`")


def _npu_dlpack_device_type(device_index: int) -> int:
    """Read torch_npu's DLPack device_type from a throwaway tensor."""
    global _NPU_DLPACK_DEVICE_TYPE
    if _NPU_DLPACK_DEVICE_TYPE is not None:
        return _NPU_DLPACK_DEVICE_TYPE
    import torch_npu  # noqa: F401

    probe = torch.empty((1,), dtype=torch.uint8, device=f"npu:{device_index}")
    capsule = probe.__dlpack__()
    get_pointer = ctypes.pythonapi.PyCapsule_GetPointer
    get_pointer.restype = ctypes.c_void_p
    get_pointer.argtypes = [ctypes.py_object, ctypes.c_char_p]
    managed = ctypes.cast(
        get_pointer(capsule, b"dltensor"),
        ctypes.POINTER(_DLManagedTensor),
    )
    _NPU_DLPACK_DEVICE_TYPE = int(managed.contents.dl_tensor.device.device_type)
    return _NPU_DLPACK_DEVICE_TYPE


@_DLPACK_DELETER
def _dlpack_deleter(managed):
    # IPC device memory is owned by the pool, not this wrapper.
    if managed:
        _DLPACK_HOLDERS.pop(ctypes.addressof(managed.contents), None)


def _wrap_npu_ptr(ptr, shape, dtype, device, size) -> torch.Tensor:
    """Wrap an imported NPU pointer with torch.from_dlpack (zero-copy)."""
    import torch_npu  # noqa: F401

    if any(dim < 0 for dim in shape):
        raise ValueError(
            f"tensor shape {shape} has a negative dimension — header may be corrupted"
        )
    type_info = _DLPACK_TYPE.get(dtype)
    if type_info is None:
        raise ValueError(f"Unsupported dtype for NPU wrap: {dtype}")
    expected_bytes = dtype.itemsize
    for dim in shape:
        expected_bytes *= dim
    if expected_bytes > size:
        raise ValueError(
            f"tensor shape {shape} * {dtype} itemsize = {expected_bytes} bytes "
            f"exceeds registered size {size} bytes — header may be corrupted"
        )

    device_index = _npu_device_index(device)
    torch.npu.set_device(device_index)
    device_type = _npu_dlpack_device_type(device_index)

    ndim = len(shape)
    shape_arr = (ctypes.c_int64 * max(ndim, 1))(*([int(d) for d in shape] or [0]))
    strides = []
    running = 1
    for dim in reversed(shape):
        strides.append(running)
        running *= int(dim)
    strides.reverse()
    stride_arr = (ctypes.c_int64 * max(ndim, 1))(*(strides or [1]))
    code, bits = type_info

    managed = _DLManagedTensor()
    managed.dl_tensor.data = ptr
    managed.dl_tensor.device.device_type = device_type
    managed.dl_tensor.device.device_id = device_index
    managed.dl_tensor.ndim = ndim
    managed.dl_tensor.dtype.code = code
    managed.dl_tensor.dtype.bits = bits
    managed.dl_tensor.dtype.lanes = 1
    managed.dl_tensor.shape = ctypes.cast(shape_arr, ctypes.POINTER(ctypes.c_int64))
    managed.dl_tensor.strides = ctypes.cast(stride_arr, ctypes.POINTER(ctypes.c_int64))
    managed.dl_tensor.byte_offset = 0
    managed.manager_ctx = None
    managed.deleter = _dlpack_deleter
    _DLPACK_HOLDERS[ctypes.addressof(managed)] = (managed, shape_arr, stride_arr)

    new_capsule = ctypes.pythonapi.PyCapsule_New
    new_capsule.restype = ctypes.py_object
    new_capsule.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_void_p]
    capsule = new_capsule(ctypes.byref(managed), b"dltensor", None)
    tensor = torch.from_dlpack(capsule)
    if tensor.data_ptr() != ptr:
        raise RuntimeError(
            f"torch.from_dlpack produced a copy (0x{tensor.data_ptr():x} != 0x{ptr:x})"
        )
    return tensor


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

    This is the canonical way to pass tensor metadata to tensor-pool
    operations such as ``register_tensor_pool`` and ``write_tensor_pool``.
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
    a tensor pool via ``read_tensor_pool``.
    """
    ptr = tensor_info.get('ptr', 0)
    if ptr == 0:
        raise ValueError("tensor_info has null pointer (ptr=0); pool may not exist or has been freed")
    dtype_str = tensor_info["dtype"]
    shape = tensor_info["shape"]
    device = tensor_info.get("device", "cpu")
    size = tensor_info.get("size", 0)

    dtype = _DTYPE_MAP.get(dtype_str, torch.int64)

    if device.startswith("npu"):
        return _wrap_npu_ptr(ptr, shape, dtype, device, size)

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
        #
        # Reject negative dimensions first: the size check below uses a
        # plain running product, and a negative dim makes that product
        # negative, so `expected_bytes > size` would be False and the
        # guard would be silently bypassed — exactly the malformed
        # (peer-controlled) header it exists to reject. A zero dim is a
        # legitimate empty tensor and passes the size check as 0 <= size.
        if any(dim < 0 for dim in shape):
            raise ValueError(
                f"tensor shape {shape} has a negative dimension — header may be corrupted"
            )
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
