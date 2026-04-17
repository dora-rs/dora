#!/usr/bin/env python
"""TODO: Add docstring."""

# Disable resource tracker to avoid warnings about shared memory
import os
os.environ['PYTHONWARNINGS'] = 'ignore::UserWarning:multiprocessing.resource_tracker'

import time

import pyarrow as pa
import torch
import numpy as np
import ctypes
from multiprocessing.shared_memory import SharedMemory
from dora import Node
from dora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle, pinned_buffer_to_torch
from tqdm import tqdm


node = Node("receiver_node")
mode = os.getenv("mode", "pinned")
massage_num = int(os.getenv("massage_num", "100"))
if not torch.cuda.is_available():
    raise RuntimeError("CUDA is not available.")
device = "cuda"
pbar = tqdm(total=massage_num)
i = 0
velocities = []

while True:
    event = node.next()
    if event is None:
        break
    if event["type"] != "INPUT":
        # 跳过非INPUT事件
        continue

    t_send = event["metadata"]["time"]
    if mode == "torch":
        # event["value"]为<class 'pyarrow.lib.Int64Array'>
        torch_tensor = torch.tensor(event["value"], device=device)
        scope = None
    elif mode == "ipc":
        ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"]) # 解析获得句柄
        scope = open_ipc_handle(ipc_handle, event["metadata"]) # 根据句柄获得内存对象
        torch_tensor = scope.__enter__() # 读取句柄的数据
    elif mode == "pinned_direct":
        # 直接从元数据读取共享内存信息，不通过node.read_pinned_memory
        metadata = event["metadata"]
        shared_memory_name = metadata.get("shared_memory_name")
        if not shared_memory_name:
            # 尝试从value中提取
            import ctypes
            from multiprocessing.shared_memory import SharedMemory
            # event["value"]是包含共享内存名称的二进制数组
            value_list = event["value"].to_pylist()
            if value_list and isinstance(value_list[0], bytes):
                shared_memory_name = value_list[0].decode('ascii')
            else:
                raise ValueError("无法获取共享内存名称")

        # 直接打开共享内存并复制到GPU

        size = metadata.get("size")
        dtype_str = metadata.get("dtype", "int64")
        shape = metadata.get("shape", [])

        if size == 0 or not shape:
            raise ValueError("Invalid metadata")

        # 打开共享内存
        shm = SharedMemory(name=shared_memory_name, create=False)
        try:
            # 转换为numpy数组
            dtype_map = {
                "int64": np.int64,
                "torch.int64": np.int64,
                "float32": np.float32,
                "torch.float32": np.float32,
            }
            np_dtype = dtype_map.get(dtype_str, np.int64)
            np_array = np.frombuffer(shm.buf, dtype=np_dtype).reshape(shape)

            # 复制到GPU
            torch_tensor = torch.from_numpy(np_array).to(device)
        finally:
            shm.close()
            shm.unlink()  # 清理共享内存

        scope = None
    else: # mode==pinned
        memory_buffer = node.read_pinned_memory(event["value"], free=True)
        torch_tensor = pinned_buffer_to_torch(memory_buffer)
        scope = None
        # Note: free_pinned_memory is no longer needed here as free=True was passed to read_pinned_memory

    t_received = time.perf_counter_ns() # ns级别高精度计时
    deta_t = t_received - t_send
    data_bytes = torch_tensor.nbytes
    velocity = data_bytes / (deta_t * 1e-9 * 1024 * 1024) # 单位为MB/S
    velocities.append(velocity)

    node.send_output("next_require", pa.array([]))
    pbar.update(1)

    if scope: # 手动释放内存
        scope.__exit__(None, None, None)
    i += 1
    if i == 1:
        print(f"接收方前五个数据：{torch_tensor[:5]}")
    elif i == massage_num:
        break
pbar.close()
velocity = torch.mean(torch.tensor(velocities))
print(f"已成功接收{i}次数据")
print(f"平均传输速率为{velocity:1f}MB/s")
