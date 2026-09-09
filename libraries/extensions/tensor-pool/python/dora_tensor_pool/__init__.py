"""Python helpers for dora's tensor-pool transport.

**Opt-in extension. Not covered by dora's 1.0 compatibility guarantees.**
See ``libraries/extensions/tensor-pool/README.md``.

These lived in ``dora.cuda`` until 1.0. They moved here because they exist to
serve the tensor-pool transport, which sits behind the extension seam (#3152),
and freezing them into the 1.0 Python API would have committed dora to a
surface that is still changing. Nothing in dora itself imports them.
"""

from .cuda import (
    IpcHandle,
    ipc_buffer_to_ipc_handle,
    open_ipc_handle,
    torch_to_ipc_buffer,
)
from .tensor_info import get_tensor_info, tensor_from_info

__all__ = [
    "IpcHandle",
    "get_tensor_info",
    "ipc_buffer_to_ipc_handle",
    "open_ipc_handle",
    "tensor_from_info",
    "torch_to_ipc_buffer",
]
