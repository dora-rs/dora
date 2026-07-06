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
# TENSOR_BYTES overrides SIZE when set (env var for future size-variation experiments).
# SIZE is in int64 elements, TENSOR_BYTES is in bytes.
TENSOR_BYTES = int(os.getenv("TENSOR_BYTES", "0"))
if TENSOR_BYTES > 0:
    SIZE = TENSOR_BYTES // 8  # int64 = 8 bytes per element
MESSAGE_COUNT = int(os.getenv("message_num", "100"))
SENDER_DEVICE = os.getenv("sender_device", "cpu")
RECEIVER_DEVICE = os.getenv("receiver_device", "cpu")
SCENARIO = os.getenv("memory_pool_scenario", "throughput")

NO_REUSE = os.environ.get("HETEROPOOL_NO_REUSE") == "1"
# Ablation: HETEROPOOL_MODE controls write_memory_pool DMA strategy.
#   "auto"     → auto-select pinned/pageable based on 25 MiB threshold (default)
#   "pinned"   → always use cudaHostRegister + pinned DMA
#   "pageable" → skip cudaHostRegister, use pageable cudaMemcpy
WRITE_MODE = os.environ.get("HETEROPOOL_MODE", "auto")

node = Node("sender_node")
data_generation = np.random.default_rng()

memory_pool_id = None
for i in range(MESSAGE_COUNT):
    random_data = data_generation.integers(1000, size=SIZE, dtype=np.int64)
    random_data[0] = i  # monotonic counter lets receiver detect change without collision risk
    torch_tensor = torch.tensor(random_data, dtype=torch.int64, device=SENDER_DEVICE)
    t_send = time.perf_counter_ns()
    metadata = {"t_send": t_send, "scenario": SCENARIO}

    tensor_info = get_tensor_info(torch_tensor)

    if NO_REUSE:
        # Ablation: register a fresh pool every frame, write once.
        # Lifecycle: register → write → send → (receiver reads) → free.
        # The sender frees after node.next() below — GPU resources like
        # cudaMalloc / cudaHostRegister are per-process and must be freed
        # by the process that allocated them.  The receiver does NOT free
        # in the NO_REUSE path; only the sender does.
        memory_pool_id = node.register_memory_pool(tensor_info, RECEIVER_DEVICE)
        if i == 0:
            print(f"Sender preview: {torch_tensor[:5]}")
        node.write_memory_pool(memory_pool_id, tensor_info, mode=WRITE_MODE)
        node.send_output("data", memory_pool_id, metadata)
    elif i == 0:
        print(f"Sender preview: {torch_tensor[:5]}")
        memory_pool_id = node.register_memory_pool(tensor_info, RECEIVER_DEVICE)
        node.send_output("data", memory_pool_id, metadata)
    else:
        if SCENARIO == "write_after_free" and i == 1:
            node.free_memory_pool(memory_pool_id)
        node.write_memory_pool(memory_pool_id, tensor_info, mode=WRITE_MODE)
        node.send_output("data", pa.array([]), metadata)

    event = node.next()
    # If the receiver crashed or the dataflow was stopped, node.next()
    # returns an input-closed control event without 'metadata'.  Break
    # instead of spinning on register_memory_pool calls that will fail.
    if "metadata" not in event:
        print(
            f"Sender: input closed at iteration {i}/{MESSAGE_COUNT} "
            f"(receiver may have crashed)"
        )
        break
    if NO_REUSE:
        # Free the pool from the sender side.  GPU resources (cudaMalloc,
        # cudaHostRegister) are per-process — only the sender can free them.
        # The receiver does not free in the NO_REUSE path, so this is the
        # single deallocation point for the register→write→read→free cycle.
        node.free_memory_pool(memory_pool_id)
