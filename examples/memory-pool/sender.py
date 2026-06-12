#!/usr/bin/env python
"""Send tensors through the memory-pool example dataflow."""

import os
import time

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from dora.cuda import get_tensor_info

SIZE = 15000 * 512
MESSAGE_COUNT = int(os.getenv("massage_num", "100"))
SENDER_DEVICE = os.getenv("sender_device", "cpu")
RECEIVER_DEVICE = os.getenv("receiver_device", "cpu")
SCENARIO = os.getenv("memory_pool_scenario", "throughput")

node = Node("sender_node")
data_generation = np.random.default_rng()

memory_pool_id = None
for i in range(MESSAGE_COUNT):
    random_data = data_generation.integers(1000, size=SIZE, dtype=np.int64)
    torch_tensor = torch.tensor(random_data, dtype=torch.int64, device=SENDER_DEVICE)
    t_send = time.perf_counter_ns()
    metadata = {"t_send": t_send, "scenario": SCENARIO}

    if i == 0:
        print(f"Sender preview: {torch_tensor[:5]}")
        tensor_info = get_tensor_info(torch_tensor)
        memory_pool_id = node.register_memory_pool(tensor_info, RECEIVER_DEVICE)
        node.send_output("data", memory_pool_id, metadata)
    else:
        tensor_info = get_tensor_info(torch_tensor)
        if SCENARIO == "write_after_free" and i == 1:
            node.free_memory_pool(memory_pool_id)
        node.write_memory_pool(memory_pool_id, tensor_info)
        node.send_output("data", pa.array([]), metadata)

    node.next()
