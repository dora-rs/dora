import pyarrow as pa

# To install pyarrow.cuda, run `conda install pyarrow "arrow-cpp-proc=*=cuda" -c conda-forge`
import pyarrow.cuda as cuda

# Make sure to install torch with cuda
import torch

# Make sure to install numba with cuda
from numba.cuda.cudadrv.devicearray import DeviceNDArray
from numba.cuda import to_device


def torch_to_buffer(tensor: torch.Tensor) -> tuple[pa.array, dict]:
    """Converts a Pytorch tensor into a pyarrow buffer containing the IPC handle and its metadata."""
    device_arr = to_device(tensor)
    cuda_buf = pa.cuda.CudaBuffer.from_numba(device_arr.gpu_data)
    handle_buffer = cuda_buf.export_for_ipc().serialize()
    metadata = {
        "shape": device_arr.shape,
        "strides": device_arr.strides,
        "dtype": device_arr.dtype.str,
    }
    return pa.array(handle_buffer, type=pa.uint8()), metadata


def buffer_to_ipc(handle_buffer: pa.array) -> cuda.IpcMemHandle:
    """Converts a buffer containing a serialized handler into cuda IPC MemHandle."""
    handle_buffer = handle_buffer.buffers()[1]
    ipc_handle = pa.cuda.IpcMemHandle.from_buffer(handle_buffer)
    return ipc_handle


def ipc_to_torch(
    arr: cuda.CudaBuffer, metadata: dict, device: str | torch.DeviceObjType
) -> torch.Tensor:
    """Converts a pyarrow CUDA buffer to a torch tensor."""

    device = torch.device(device)
    shape = metadata["shape"]
    strides = metadata["strides"]
    dtype = metadata["dtype"]
    device_arr = DeviceNDArray(shape, strides, dtype, gpu_data=arr.to_numba())
    if device.type == "cpu":
        torch_tensor = torch.as_tensor(device_arr.copy_to_host())
    elif device.type == "cuda":
        torch_tensor = torch.as_tensor(device_arr, device=device)
    else:
        raise NotImplementedError("Haven't implemented this device yet!")
    return torch_tensor
