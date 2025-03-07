#!/usr/bin/env python

import os
import time

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from dora.cuda import torch_to_ipc_buffer

torch.tensor([], device="cuda")

SIZES = [10000 * 512]

DEVICE = os.getenv("DEVICE", "cuda")

pa.array([])
node = Node()


time.sleep(4)
# test latency first
for size in SIZES:
    for _ in range(100):
        now = time.time()
        random_data = np.random.randint(1000, size=size, dtype=np.int64)
        torch_tensor = torch.tensor(random_data, dtype=torch.int64, device="cuda")
        t_send = time.perf_counter_ns()
        if DEVICE == "cpu":
            # BEFORE
            torch_tensor = torch_tensor.to("cpu")
            metadata = {}
            metadata["time"] = t_send
            metadata["device"] = "cpu"
            node.send_output("latency", pa.array(torch_tensor.numpy()), metadata)
        else:
            # AFTER
            ipc_buffer, metadata = torch_to_ipc_buffer(torch_tensor)
            metadata["time"] = t_send
            metadata["device"] = "cuda"
            node.send_output("latency", ipc_buffer, metadata)

        # Wait before sending next output
        node.next()
