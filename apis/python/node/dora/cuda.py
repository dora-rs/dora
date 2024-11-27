import pyarrow as pa

# To install pyarrow.cuda, run `conda install pyarrow "arrow-cpp-proc=*=cuda" -c conda-forge`
import pyarrow.cuda as cuda

# Make sure to install torch with cuda
import torch

# Make sure to install numba with cuda
from numba.cuda.cudadrv.devicearray import DeviceNDArray
from numba.cuda import to_device


def torch_to_ipc_buffer(tensor: torch.TensorType) -> tuple[pa.array, dict]:
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


def ipc_buffer_to_ipc_handle(handle_buffer: pa.array) -> cuda.IpcMemHandle:
    """Converts a buffer containing a serialized handler into cuda IPC MemHandle."""
    handle_buffer = handle_buffer.buffers()[1]
    ipc_handle = pa.cuda.IpcMemHandle.from_buffer(handle_buffer)
    return ipc_handle


def cudabuffer_to_numba(buffer: cuda.CudaBuffer, metadata: dict) -> DeviceNDArray:
    """Converts a pyarrow CUDA buffer to numba."""

    shape = metadata["shape"]
    strides = metadata["strides"]
    dtype = metadata["dtype"]
    device_arr = DeviceNDArray(shape, strides, dtype, gpu_data=buffer.to_numba())
    return device_arr


def cudabuffer_to_torch(buffer: cuda.CudaBuffer, metadata: dict) -> torch.Tensor:
    """Converts a pyarrow CUDA buffer to a torch tensor."""

    device_arr = cudabuffer_to_numba(buffer, metadata)
    torch_tensor = torch.as_tensor(device_arr, device="cuda")
    return torch_tensor
