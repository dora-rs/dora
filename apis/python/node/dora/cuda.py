"""CUDA memory utilities for dora-rs dataflow nodes.

Provides helper functions for managing CUDA Inter-Process Communication (IPC)
memory handles. These utilities enable zero-copy GPU data transfer between
dora-rs nodes using PyTorch and Numba, facilitating high-performance
robotic applications.
"""

import pyarrow as pa

# Make sure to install torch with cuda
import torch
from numba.cuda import to_device

# Make sure to install numba with cuda
from numba.cuda.cudadrv.devicearray import DeviceNDArray
from numba.cuda.cudadrv.devices import get_context
from numba.cuda.cudadrv.driver import IpcHandle


import json
import ctypes
import atexit

from contextlib import contextmanager
from typing import ContextManager

# Global dictionary to track allocated pinned memory
_allocated_pinned_memory = {}

def _cuda_free_host(ptr):
    """Free pinned memory allocated with cudaHostAlloc."""
    try:
        cuda = ctypes.CDLL("libcudart.so")
        cuda.cudaFreeHost.restype = ctypes.c_int
        cuda.cudaFreeHost.argtypes = [ctypes.c_void_p]
        result = cuda.cudaFreeHost(ptr)
        if result != 0:
            print(f"Warning: cudaFreeHost failed with error code {result}")
    except OSError:
        print("Warning: Could not load CUDA runtime to free pinned memory")

def _cleanup_pinned_memory():
    """Clean up all allocated pinned memory on exit."""
    for key, value in list(_allocated_pinned_memory.items()):
        if isinstance(value, tuple) and len(value) == 2:
            obj, size = value
            if isinstance(obj, ctypes.c_void_p):
                # Old style: CUDA pinned memory pointer
                _cuda_free_host(obj)
            else:
                # New style: SharedMemory object
                try:
                    from multiprocessing.shared_memory import SharedMemory
                    if isinstance(obj, SharedMemory):
                        obj.close()
                        # Only unlink if we created it (should be the case)
                        try:
                            obj.unlink()
                        except FileNotFoundError:
                            pass  # Already unlinked
                except ImportError:
                    pass
        # Also handle direct SharedMemory objects (if stored differently)
        elif hasattr(value, 'close'):
            try:
                value.close()
                if hasattr(value, 'unlink'):
                    try:
                        value.unlink()
                    except FileNotFoundError:
                        pass
            except:
                pass
    _allocated_pinned_memory.clear()

atexit.register(_cleanup_pinned_memory)


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

def torch_to_pinned_buffer(tensor: torch.TensorType) -> tuple[pa.array, dict]:
    """
    input:tensor
    return pinned_buffer, metadata
    pinned_buffer为用py.array包装的共享内存标识符
    metadata为字典，包括tensor的ptr,size,dtype,shape
    生成的共享内存标识符用于node.register_pinned_memory(pinned_buffer, metadata)，由daemon统一分配页锁内存
    """
    import uuid
    from multiprocessing.shared_memory import SharedMemory

    # Check if tensor is on CPU (pinned memory must be allocated on host)
    if tensor.is_cuda:
        raise ValueError("Tensor must be on CPU for pinned memory allocation")

    size = tensor.nbytes
    # Create a unique name for shared memory
    memory_uuid = str(uuid.uuid4())
    shm_name = f"dora_pinned_{memory_uuid}"

    # Allocate shared memory
    shm = SharedMemory(name=shm_name, create=True, size=size)

    # Copy tensor data to shared memory
    # Get tensor as numpy array (CPU tensor) - this creates a view, not a copy
    np_array = tensor.numpy()

    # Get source data as bytes
    src_bytes = np_array.tobytes()

    # Get destination memoryview
    dst_mv = shm.buf

    # Copy data directly using slice assignment (avoids ctypes references)
    dst_mv[:] = src_bytes

    # Optionally register shared memory as pinned memory with CUDA
    # cudaHostRegister requires CUDA runtime
    # Get pointer to shared memory buffer
    try:
        # Get pointer from memoryview using ctypes and temporary array
        # This is safe because we don't keep a reference to the array
        mv = shm.buf
        # Create a temporary ctypes array to get the address
        # Use from_buffer without keeping the array reference
        temp_arr = (ctypes.c_char * size).from_buffer(mv)
        ptr = ctypes.addressof(temp_arr)
        # Immediately delete the temporary array to avoid holding reference
        del temp_arr
    except Exception:
        ptr = 0

    if ptr != 0:
        try:
            cuda = ctypes.CDLL("libcudart.so")
            cuda.cudaHostRegister.restype = ctypes.c_int
            cuda.cudaHostRegister.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint]
            cudaHostRegisterDefault = 0
            result = cuda.cudaHostRegister(ptr, size, cudaHostRegisterDefault)
            if result != 0:
                # print(f"Warning: cudaHostRegister failed with error code {result}. Memory may not be pinned.")
                pass
        except OSError:
            # print("Warning: Could not load CUDA runtime to register shared memory as pinned")
            pass

    # Create buffer identifier (shared memory name)
    buffer_id = shm_name.encode('ascii')
    buffer_array = pa.array([buffer_id], type=pa.binary())

    metadata = {
        "ptr": ptr if ptr is not None else 0,  # Pointer to shared memory in current process
        "size": size,
        "dtype": str(tensor.dtype),
        "shape": list(tensor.shape),
        # Additional metadata
        "is_pinned": True,
        "shared_memory_name": shm_name,
        "buffer_id": memory_uuid,  # Store UUID for later cleanup
    }

    # Store the shared memory object in global dictionary for later cleanup
    _allocated_pinned_memory[memory_uuid] = (shm, size)

    return buffer_array, metadata

def pinned_buffer_to_torch(memory_buffer: dict, free_source: bool = True) -> torch.Tensor:
    """
    input:memory_buffer
    return torch
    根据node.read_pinned_memory(event["value"])返回的memory_buffer,生成cuda上的tensor张量并返回
    memory_buffer是一个字典，包含cuda上的ptr，张量的size,dtype,shape
    free_source: if True, free the source pinned memory after copying to GPU
    """
    import numpy as np
    from multiprocessing.shared_memory import SharedMemory

    # Extract metadata
    # print(f"Debug: memory_buffer keys: {list(memory_buffer.keys())}")
    ptr = memory_buffer.get("ptr", 0)
    size = memory_buffer.get("size", 0)
    dtype_str = memory_buffer.get("dtype", "int64")
    shape = memory_buffer.get("shape", [])
    is_pinned = memory_buffer.get("is_pinned", False)
    buffer_id = memory_buffer.get("buffer_id", None)
    shared_memory_name = memory_buffer.get("shared_memory_name", None)
    # print(f"Debug: extracted shared_memory_name={shared_memory_name}, buffer_id={buffer_id}")

    if size == 0 or not shape:
        raise ValueError("Invalid memory buffer metadata")

    # Convert dtype string to torch dtype
    dtype_map = {
        "int64": torch.int64,
        "float32": torch.float32,
        "float64": torch.float64,
        "int32": torch.int32,
        "int16": torch.int16,
        "int8": torch.int8,
        "uint8": torch.uint8,
        "bool": torch.bool,
        "float16": torch.float16,
        "bfloat16": torch.bfloat16,
    }
    dtype = dtype_map.get(dtype_str, torch.int64)

    # Calculate number of elements
    num_elements = np.prod(shape)

    # Check if CUDA is available
    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is not available")

    # Clear any CUDA errors from previous operations
    torch.cuda.synchronize()
    torch.cuda.empty_cache()

    # Reset CUDA device if needed
    try:
        torch.cuda.reset_peak_memory_stats()
    except:
        pass

    # Allocate GPU memory with error handling
    try:
        gpu_tensor = torch.empty(shape, dtype=dtype, device="cuda")
    except torch.cuda.OutOfMemoryError:
        torch.cuda.empty_cache()
        gpu_tensor = torch.empty(shape, dtype=dtype, device="cuda")
    except RuntimeError as e:
        # Catch CUDA errors like "resource already mapped"
        if "resource already mapped" in str(e).lower() or "cuda error" in str(e).lower():
            # Try a more aggressive reset
            torch.cuda.synchronize()
            torch.cuda.empty_cache()
            import time
            time.sleep(0.01)  # Longer delay
            # Try to reset the device
            try:
                torch.cuda.init()
            except:
                pass
            gpu_tensor = torch.empty(shape, dtype=dtype, device="cuda")
        else:
            raise
    except Exception as e:
        # Try one more time after full reset
        torch.cuda.synchronize()
        torch.cuda.empty_cache()
        import time
        time.sleep(0.001)  # Small delay
        gpu_tensor = torch.empty(shape, dtype=dtype, device="cuda")

    # Determine source pointer: either from shared memory or from ptr
    host_ptr = None
    shm = None
    memory_registered = False
    cuda_lib = None  # Cache for CUDA library

    try:
        # print(f"Debug pinned_buffer_to_torch: shared_memory_name={shared_memory_name}, ptr={ptr}, size={size}, shape={shape}")
        if shared_memory_name:
            try:
                # Open existing shared memory
                # print(f"Debug: Attempting to open shared memory: {shared_memory_name}")
                shm = SharedMemory(name=shared_memory_name, create=False)
                # print(f"Debug: Successfully opened shared memory, buf type: {type(shm.buf)}, buf len: {len(shm.buf)}")

                # Get pointer to shared memory buffer using temporary ctypes array
                # This avoids creating Python objects that hold references to the buffer
                mv = shm.buf
                # Create a temporary ctypes array to get the address
                # Use from_buffer without keeping the array reference
                temp_arr = (ctypes.c_char * size).from_buffer(mv)
                host_ptr = ctypes.c_void_p(ctypes.addressof(temp_arr))
                # Immediately delete the temporary array to avoid holding reference
                del temp_arr
                # print(f"Debug: Got pointer from temporary array: {host_ptr.value}")

                # Register the shared memory as pinned memory for CUDA
                # This is needed for DMA transfers
                try:
                    cuda_lib = ctypes.CDLL("libcudart.so")
                    cuda_lib.cudaHostRegister.restype = ctypes.c_int
                    cuda_lib.cudaHostRegister.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint]
                    cudaHostRegisterDefault = 0
                    result = cuda_lib.cudaHostRegister(host_ptr, size, cudaHostRegisterDefault)
                    if result != 0:
                        # Error 712 is CUDA_ERROR_HOST_MEMORY_ALREADY_REGISTERED, which is OK
                        if result != 712:
                            # print(f"Warning: cudaHostRegister failed with error code {result}. Memory may not be pinned for DMA.")
                            pass
                        else:
                            # print(f"Debug: Memory already registered (error {result}), continuing...")
                            memory_registered = True
                    else:
                        # print(f"Debug: Successfully registered shared memory as pinned (size={size})")
                        memory_registered = True
                except OSError:
                    # print("Warning: Could not load CUDA runtime to register shared memory as pinned")
                    pass

                # print(f"Debug: Opened shared memory {shared_memory_name}, ptr={host_ptr.value}, size={size}")
            except Exception as e:
                # print(f"Warning: Failed to open shared memory {shared_memory_name}: {e}")
                # import traceback
                # traceback.print_exc()
                # Fall back to ptr (might be invalid in receiver process)
                # print(f"Warning: Falling back to metadata ptr={ptr}")
                host_ptr = ctypes.c_void_p(ptr)
        else:
            host_ptr = ctypes.c_void_p(ptr)

        # Copy data from pinned memory to GPU using cudaMemcpy
        if cuda_lib is None:
            try:
                cuda_lib = ctypes.CDLL("libcudart.so")
            except OSError:
                raise RuntimeError("CUDA runtime library not found")

        # Define cudaMemcpy function signature
        cuda_lib.cudaMemcpy.restype = ctypes.c_int
        cuda_lib.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]

        # Constants
        cudaMemcpyHostToDevice = 1

        # Get GPU tensor pointer
        gpu_ptr = ctypes.c_void_p(gpu_tensor.data_ptr())

        # Use synchronous copy for simplicity
        result = cuda_lib.cudaMemcpy(gpu_ptr, host_ptr, size, cudaMemcpyHostToDevice)
        if result != 0:
            raise RuntimeError(f"cudaMemcpy failed with error code {result}")

        return gpu_tensor

    finally:
        # Clean up shared memory if opened
        if shm is not None:
            if free_source:
                # Only close and unlink if we're freeing the source
                # First, unregister the memory if it was registered
                if memory_registered and cuda_lib is not None:
                    try:
                        # Try to unregister the pinned memory
                        cuda_lib.cudaHostUnregister.restype = ctypes.c_int
                        cuda_lib.cudaHostUnregister.argtypes = [ctypes.c_void_p]
                        result = cuda_lib.cudaHostUnregister(host_ptr)
                        if result != 0:
                            print(f"Warning: cudaHostUnregister failed with error code {result}")
                    except (OSError, AttributeError):
                        pass  # CUDA runtime not available or function not found

                shm.close()
                try:
                    shm.unlink()
                except FileNotFoundError:
                    pass
            else:
                # Don't close the shared memory, as it's still in use or will be cleaned up by sender
                # The memory will be automatically cleaned up when the object is garbage collected
                # But we should at least close it to avoid resource tracker warnings
                shm.close()

        # Free source pinned memory if requested (old style CUDA pinned memory)
        # This is for compatibility with old-style CUDA pinned memory (not shared memory)
        if free_source and is_pinned and buffer_id and not shared_memory_name:
            # Remove from global dictionary and free memory
            if buffer_id in _allocated_pinned_memory:
                ptr_entry, size_entry = _allocated_pinned_memory.pop(buffer_id)
                # Ensure we're freeing the same pointer (should be)
                if ptr_entry.value == ptr:
                    _cuda_free_host(ptr_entry)
                else:
                    # This shouldn't happen, but free the given ptr anyway
                    _cuda_free_host(ctypes.c_void_p(ptr))
            else:
                # Buffer ID not found, free using the ptr directly
                _cuda_free_host(ctypes.c_void_p(ptr))