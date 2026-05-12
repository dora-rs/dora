#!/usr/bin/env python
"""TODO: Add docstring."""

import os
import time

import pyarrow as pa
import torch
from dora import Node
from dora.cuda import tensor_from_info
from tqdm import tqdm


node = Node("receiver_node")
massage_num = int(os.getenv("massage_num", "100"))
receiver_device = os.getenv("receiver_device", "cpu")
if not torch.cuda.is_available():
    raise RuntimeError("CUDA is not available.")
pbar = tqdm(total=massage_num)
velocities = []
scope = None

for i in range(massage_num):
    event = node.next()
    t_send = event["metadata"]["t_send"]
    if i == 0:
        memory_pool_id = event["value"]
        tensor_info = node.read_memory_pool(memory_pool_id)
        torch_tensor = tensor_from_info(tensor_info)
        print(f"接收方前五个数据：{torch_tensor[:5]}")
    t_received = time.perf_counter_ns() # ns级别高精度计时
    deta_t = t_received - t_send
    data_bytes = torch_tensor.nbytes
    velocity = data_bytes / (deta_t * 1e-9 * 1024 * 1024) # 单位为MB/S
    velocities.append(velocity)

    node.send_output("next_require", pa.array([]))
    pbar.update(1)
pbar.close()
velocity = torch.mean(torch.tensor(velocities))
print(f"平均传输速率为{velocity:1f}MB/s")
node.free_memory_pool(memory_pool_id)
