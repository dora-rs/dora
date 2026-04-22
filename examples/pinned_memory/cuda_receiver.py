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
from dora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle, pinned_ptr_to_torch
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
scope = None

while True:
    event = node.next()
    t_send = event["metadata"]["time"]
    if mode == "torch":
        # event["value"]为<class 'pyarrow.lib.Int64Array'>
        torch_tensor = torch.tensor(event["value"], device=device)
    elif mode == "ipc":
        ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"]) # 解析获得句柄
        scope = open_ipc_handle(ipc_handle, event["metadata"]) # 根据句柄获得内存对象
        torch_tensor = scope.__enter__() # 读取句柄的数据
    else: # mode==pinned
        pinned_buffer = event["value"]
        pinned_ptr, metadata = node.read_pinned_memory(pinned_buffer, free=False)
        torch_tensor = pinned_ptr_to_torch(pinned_ptr, metadata)
        node.free_pinned_memory(pinned_buffer)

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
