"""Pool-side tensor helpers, parked out of `dora/cuda.py`.

`get_tensor_info` / `tensor_from_info` were the documented way to feed the
removed `register_memory_pool` / `write_memory_pool` / `read_memory_pool`
methods, and the two dtype maps existed only to serve them. They have no
other caller in dora, so they travelled with the transport.

`_CudaArrayInterface` stays in `dora/cuda.py` — `open_ipc_handle` still uses
it. Reinstating these belongs on the dora-pool side of the seam, not in the
`dora` package. See ../README.md.
"""

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
