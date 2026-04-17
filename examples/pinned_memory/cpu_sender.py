#!/usr/bin/env python
"""TODO: Add docstring."""


import os
import time

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from dora.cuda import torch_to_ipc_buffer, torch_to_pinned_buffer

SIZES = [10000 * 512]
mode = os.getenv("mode", "pinned")
massage_num = int(os.getenv("massage_num", "100"))
device = "cpu"
node = Node("sender_node")
data_generation = np.random.default_rng()

for size in SIZES:  # 每次发送size形状的整数
    pinned_buffer = None  # Store pinned buffer for cleanup
    for i in range(massage_num): # 发送100次
        random_data = data_generation.integers(1000, size=size, dtype=np.int64)
        torch_tensor = torch.tensor(random_data, dtype=torch.int64, device=device)
        if i == 0:
            print(f"发送方前五个数据：{torch_tensor[:5]}")
        t_send = time.perf_counter_ns() # ns级别高精度计时
        if mode == "torch": # 速度约为2200MB/S
            metadata = {"time": t_send, "size": torch_tensor.nbytes}
            node.send_output("cpu_data", pa.array(torch_tensor.numpy()), metadata)
        elif mode == "ipc": # 速度约为4200MB/S
            torch_tensor = torch_tensor.to("cuda")
            ipc_buffer, metadata = torch_to_ipc_buffer(torch_tensor)
            metadata["time"] = t_send
            metadata["size"] = torch_tensor.nbytes
            node.send_output("cpu_data", ipc_buffer, metadata)
        else: # mode=pinned 速度期望40000MB/S
            pinned_buffer, metadata = torch_to_pinned_buffer(torch_tensor)
            metadata["time"] = t_send
            node.register_pinned_memory(pinned_buffer, metadata)
            node.send_output("cpu_data", pinned_buffer, metadata)

        node.next()
