#!/usr/bin/env python
"""TODO: Add docstring."""

import os
import time

import pyarrow as pa
import torch
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
    else: # mode==pinned
        memory_buffer = node.read_pinned_memory(event["value"])
        # Add missing fields from event metadata if not present
        if "shared_memory_name" in event["metadata"] and "shared_memory_name" not in memory_buffer:
            memory_buffer["shared_memory_name"] = event["metadata"]["shared_memory_name"]
        if "buffer_id" in event["metadata"] and "buffer_id" not in memory_buffer:
            memory_buffer["buffer_id"] = event["metadata"]["buffer_id"]
        if "is_pinned" in event["metadata"] and "is_pinned" not in memory_buffer:
            memory_buffer["is_pinned"] = event["metadata"]["is_pinned"]
        torch_tensor = pinned_buffer_to_torch(memory_buffer, free_source=False)
        scope = None
        node.free_pinned_memory(event["value"])

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
