#!/usr/bin/env python
"""TODO: Add docstring."""


import os
import time

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from dora.cuda import cudabuffer_to_torch, ipc_buffer_to_ipc_handle
from helper import record_results
from tqdm import tqdm

torch.tensor([], device="cuda")


pa.array([])
context = pa.cuda.Context()
node = Node("node_2")

current_size = 8
n = 0
i = 0
latencies = []
mean_cpu = mean_cuda = 0
DEVICE = os.getenv("DEVICE", "cuda")

NAME = f"dora torch {DEVICE}"

ctx = pa.cuda.Context()

print()
print("Receiving 40MB packets using default dora-rs")

while True:
    event = node.next()

    if event["type"] == "INPUT":
        if i == 0:
            pbar = tqdm(total=100)
        elif i == 100:
            print("vs")
            print("Receiving 40MB packets using dora-rs CUDA->CUDA")
            pbar = tqdm(total=100)
        t_send = event["metadata"]["time"]

        if event["metadata"]["device"] != "cuda":
            # BEFORE
            handle = event["value"].to_numpy()
            torch_tensor = torch.tensor(handle, device="cuda")
        else:
            # AFTER
            # storage needs to be spawned in the same file as where it's used. Don't ask me why.
            ipc_handle = ipc_buffer_to_ipc_handle(event["value"])
            cudabuffer = ctx.open_ipc_buffer(ipc_handle)
            torch_tensor = cudabuffer_to_torch(cudabuffer, event["metadata"])  # on cuda
    else:
        break
    t_received = time.perf_counter_ns()
    length = len(torch_tensor) * 8

    pbar.update(1)
    latencies.append((t_received - t_send) / 1000)
    node.send_output("next", pa.array([]))

    i += 1
    if i == 100:
        pbar.close()
        t_end_cpu = time.time()
        mean_cpu = np.array(latencies).mean()
        latencies = []
    n += 1


mean_cuda = np.array(latencies).mean()
pbar.close()

time.sleep(2)

print()
print("----")
print(f"Node communication duration with default dora-rs: {mean_cpu/1000:.1f}ms")
print(f"Node communication duration with dora CUDA->CUDA: {mean_cuda/1000:.1f}ms")

print("----")
print(f"Speed Up: {(mean_cpu)/(mean_cuda):.0f}")
record_results(NAME, current_size, latencies)
