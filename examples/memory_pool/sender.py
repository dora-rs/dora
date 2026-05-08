#!/usr/bin/env python
"""TODO: Add docstring."""

import os
import time

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from dora.cuda import get_tensor_info

size = 15000 * 512
massage_num = int(os.getenv("massage_num", "100"))
sender_device = os.getenv("sender_device", "cpu")
receiver_device = os.getenv("receiver_device", "cpu")
node = Node("sender_node")
data_generation = np.random.default_rng()

for i in range(massage_num): # 发送100次
    random_data = data_generation.integers(1000, size=size, dtype=np.int64)
    torch_tensor = torch.tensor(random_data, dtype=torch.int64, device=sender_device)
    t_send = time.perf_counter_ns() # ns级别高精度计时
    metadata = {"t_send": t_send}
    if i == 0:
        print(f"发送方前五个数据：{torch_tensor[:5]}")
        tensor_info = get_tensor_info(torch_tensor)
        memory_pool_id = node.register_memory_pool(tensor_info, receiver_device)
        node.send_output("data", memory_pool_id, metadata)
    else:
        tensor_info = get_tensor_info(torch_tensor)
        node.write_memory_pool(memory_pool_id, tensor_info)
        node.send_output("data", pa.array([]), metadata)
    node.next()
