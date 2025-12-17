"""TODO: Add docstring."""

import pyarrow as pa

# Make sure to install torch with cuda
import torch
from numba.cuda import to_device

# Make sure to install numba with cuda
from numba.cuda.cudadrv.devicearray import DeviceNDArray
from numba.cuda.cudadrv.devices import get_context
from numba.cuda.cudadrv.driver import IpcHandle


import json

from contextlib import contextmanager
from typing import ContextManager


def torch_to_ipc_buffer(tensor: torch.TensorType) -> tuple[pa.array, dict]:
    """Convert a Pytorch tensor into a pyarrow buffer containing the IPC handle
    and its metadata.

    Example Use:
    ```python
    torch_tensor = torch.tensor(random_data, dtype=torch.int64, device="cuda")
    ipc_buffer, metadata = torch_to_ipc_buffer(torch_tensor)
    node.send_output("latency", ipc_buffer, metadata)
    ```
    """
    device_arr = to_device(tensor)
    ipch = get_context().get_ipc_handle(device_arr.gpu_data)
    _, handle, size, source_info, offset = ipch.__reduce__()[1]
    metadata = {
        "shape": device_arr.shape,
        "strides": device_arr.strides,
        "dtype": device_arr.dtype.str,
        "size": size,
        "offset": offset,
        "source_info": json.dumps(source_info),
    }
    return pa.array(handle, pa.int8()), metadata


def ipc_buffer_to_ipc_handle(handle_buffer: pa.array, metadata: dict) -> IpcHandle:
    """Convert a buffer containing a serialized handler into cuda IPC Handle.

    example use:
    ```python
    from dora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle

    event = node.next()

    ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"])
    with open_ipc_handle(ipc_handle, event["metadata"]) as tensor:
        pass
    ```
    """
    handle = handle_buffer.to_pylist()
    return IpcHandle._rebuild(
        handle,
        metadata["size"],
        json.loads(metadata["source_info"]),
        metadata["offset"],
    )


@contextmanager
def open_ipc_handle(
    ipc_handle: IpcHandle, metadata: dict
) -> ContextManager[torch.TensorType]:
    """Open a CUDA IPC handle and return a Pytorch tensor.

    example use:
    ```python
    from dora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle

    event = node.next()

    ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"])
    with open_ipc_handle(ipc_handle, event["metadata"]) as tensor:
        pass
    ```
    """
    shape = metadata["shape"]
    strides = metadata["strides"]
    dtype = metadata["dtype"]
    try:
        buffer = ipc_handle.open(get_context())
        device_arr = DeviceNDArray(shape, strides, dtype, gpu_data=buffer)
        yield torch.as_tensor(device_arr, device="cuda")
    finally:
        ipc_handle.close()
