#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import os
import numpy as np
import pyarrow as pa
from dora import Node
from dora.cuda import torch_to_buffer
import torch

torch.tensor([], device="cuda")

SIZES = [512, 10 * 512, 100 * 512, 1000 * 512, 10000 * 512]

DEVICE = os.getenv("DEVICE", "cuda")

pa.array([])
node = Node()


time.sleep(4)
# test latency first
for size in SIZES:
    for _ in range(0, 100):
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
            buffer, metadata = torch_to_buffer(torch_tensor)
            metadata["time"] = t_send
            metadata["device"] = "cuda"
            node.send_output("latency", buffer, metadata)

        # Wait before sending next output
        node.next()
        time.sleep(0.1)
