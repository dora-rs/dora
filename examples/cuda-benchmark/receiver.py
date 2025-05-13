#!/usr/bin/env python
"""TODO: Add docstring."""

import os
import time

import pyarrow as pa
import torch
from dora import Node
from dora.cuda import ipc_buffer_to_ipc_handle, open_ipc_handle
from helper import record_results
from tqdm import tqdm

torch.tensor([], device="cuda")


pa.array([])
pbar = tqdm(total=100)
node = Node("node_2")


current_size = 8
n = 0
i = 0
latencies = []
DEVICE = os.getenv("DEVICE", "cuda")

NAME = f"dora torch {DEVICE}"

while True:
    event = node.next()
    if event["type"] == "INPUT":
        t_send = event["metadata"]["time"]

        if event["metadata"]["device"] != "cuda":
            # BEFORE
            handle = event["value"].to_numpy()
            torch_tensor = torch.tensor(handle, device="cuda")
            scope = None
        else:
            # AFTER
            ipc_handle = ipc_buffer_to_ipc_handle(event["value"], event["metadata"])
            scope = open_ipc_handle(ipc_handle, event["metadata"])
            torch_tensor = scope.__enter__()
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

    if scope:
        scope.__exit__(None, None, None)

record_results(NAME, current_size, latencies)
