#!/usr/bin/env python
"""Receive tensors through the memory-pool example dataflow."""

import os
import time

import pyarrow as pa
import torch
from dora import Node
from dora.cuda import tensor_from_info
from tqdm import tqdm

node = Node("receiver_node")
MESSAGE_COUNT = int(os.getenv("message_num", "100"))
RECEIVER_DEVICE = os.getenv("receiver_device", "cpu")
SCENARIO = os.getenv("memory_pool_scenario", "throughput")

if RECEIVER_DEVICE.startswith("cuda") and not torch.cuda.is_available():
    raise RuntimeError("CUDA is not available for the configured receiver device.")

pbar = tqdm(total=MESSAGE_COUNT)
velocities = []
memory_pool_id = None
torch_tensor = None
prev_counter = None

for i in range(MESSAGE_COUNT):
    event = node.next()
    t_send = event["metadata"]["t_send"]

    if i == 0:
        memory_pool_id = event["value"]
        tensor_info = node.read_memory_pool(memory_pool_id)
        torch_tensor = tensor_from_info(tensor_info)
        print(f"Receiver preview: {torch_tensor[:5]}")

    # The tensor is zero-copy — write_memory_pool on the sender overwrites
    # the shmem bytes in place, so the receiver's existing tensor object
    # automatically reflects new data.  Turn-based signaling ensures the
    # sender has finished writing before the receiver accesses the tensor.
    # Validate that the pool write propagated by checking the deterministic
    # counter stamped into element [0] by the sender (sender sets data[0] = i).
    curr_counter = int(torch_tensor[0].item())
    if prev_counter is not None and SCENARIO != "write_after_free":
        assert curr_counter == prev_counter + 1, (
            f"iteration {i}: counter {curr_counter} != expected {prev_counter + 1}"
            " — pool write may not have propagated"
        )
    prev_counter = curr_counter

    t_received = time.perf_counter_ns()
    delta_t = t_received - t_send
    data_bytes = torch_tensor.nbytes
    velocity = data_bytes / (delta_t * 1e-9 * 1024 * 1024)
    velocities.append(velocity)

    if SCENARIO == "duplicate_free" and i == MESSAGE_COUNT - 1:
        node.free_memory_pool(memory_pool_id)
        node.free_memory_pool(memory_pool_id)
    elif SCENARIO == "read_after_free" and i == MESSAGE_COUNT - 1:
        node.free_memory_pool(memory_pool_id)
        try:
            node.read_memory_pool(memory_pool_id)
        except Exception:
            pass  # Expected: pool was freed, read should fail
    elif SCENARIO != "auto_cleanup" and i == MESSAGE_COUNT - 1:
        node.free_memory_pool(memory_pool_id)

    node.send_output("next_require", pa.array([]))
    pbar.update(1)

pbar.close()
average_velocity = torch.mean(torch.tensor(velocities, dtype=torch.float64))
print(f"Average transfer throughput: {average_velocity:1f} MB/s")
