#!/usr/bin/env python

import os
import time

import pyarrow as pa
import torch
from dora import Node
from dora.cuda import cudabuffer_to_torch, ipc_buffer_to_ipc_handle
from helper import record_results
from tqdm import tqdm

torch.tensor([], device="cuda")


pa.array([])
pbar = tqdm(total=100)
context = pa.cuda.Context()
node = Node("node_2")


current_size = 8
n = 0
i = 0
latencies = []
DEVICE = os.getenv("DEVICE", "cuda")

NAME = f"dora torch {DEVICE}"

ctx = pa.cuda.Context()

while True:
    event = node.next()
    if event["type"] == "INPUT":
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

    if length != current_size:
        if n > 0:

            pbar.close()
            pbar = tqdm(total=100)
            record_results(NAME, current_size, latencies)
        current_size = length
        n = 0
        start = time.perf_counter_ns()
        latencies = []

    pbar.update(1)
    latencies.append((t_received - t_send) / 1000)
    node.send_output("next", pa.array([]))

    n += 1
    i += 1

record_results(NAME, current_size, latencies)
