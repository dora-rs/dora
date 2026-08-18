#!/usr/bin/env python
"""Send tensors through the memory-pool example dataflow."""

import os
import sys
import time

import numpy as np
import pyarrow as pa
import torch
from dora import Node
from dora.cuda import get_tensor_info

SIZE = 15000 * 512
MESSAGE_COUNT = int(os.getenv("message_num", "100"))
SENDER_DEVICE = os.getenv("sender_device", "cpu")
RECEIVER_DEVICE = os.getenv("receiver_device", "cpu")
SCENARIO = os.getenv("memory_pool_scenario", "throughput")

if SENDER_DEVICE.startswith("cuda"):
    idx = int(SENDER_DEVICE.split(":")[1]) if ":" in SENDER_DEVICE else 0
    torch.cuda.set_device(idx)

node = Node("sender_node")
data_generation = np.random.default_rng()

memory_pool_id = None
for i in range(MESSAGE_COUNT):
    random_data = data_generation.integers(1000, size=SIZE, dtype=np.int64)
    random_data[0] = i  # monotonic counter lets receiver detect change without collision risk
    torch_tensor = torch.tensor(random_data, dtype=torch.int64, device=SENDER_DEVICE)
    # Cross-machine: wall clock (time.time_ns), NOT perf_counter —
    # CLOCK_MONOTONIC's epoch is each machine's boot time, so deltas
    # across machines are dominated by the boot-time difference (the
    # receiver measured ~0.00002 MB/s with perf_counter).  The hosts
    # are NTP-synced, making wall-clock deltas the true transfer time.
    t_send = time.time_ns()
    metadata = {"t_send": t_send, "scenario": SCENARIO}

    if i == 0:
        print(f"Sender preview: {torch_tensor[:5]}")
        tensor_info = get_tensor_info(torch_tensor)
        memory_pool_id = node.register_tensor_pool(
            tensor_info, RECEIVER_DEVICE, machine=os.getenv("cross_machine")
        )
        if memory_pool_id is None:
            if os.getenv("cross_machine"):
                print(
                    "Cross-machine register failed (warned, no pool created) — exiting",
                    flush=True,
                )
            else:
                print(
                    "Tensor pool registration failed (warned, no pool created) — exiting",
                    flush=True,
                )
            sys.exit(1)
        # The receiver retries its first read (frame-order guard) until
        # the registration push lands — no background re-push needed: the
        # daemon declares the memory-pool subscription before building
        # nodes, so the registration push cannot be lost, and the
        # handshake (next_require) confirms the frame was consumed.
        node.send_output("data", memory_pool_id, metadata)
        node.next()
    else:
        tensor_info = get_tensor_info(torch_tensor)
        if SCENARIO == "write_after_free" and i == 1:
            node.free_tensor_pool(memory_pool_id)
        node.write_tensor_pool(memory_pool_id, tensor_info)
        node.send_output("data", pa.array([]), metadata)
        # Turn-based handshake: wait for the receiver's ack (it sends
        # next_require after consuming this frame) before writing the
        # next one. The frame-order guarantee comes from the ack, not
        # from pacing — no sleep needed. The receiver acks every frame
        # after reading, so this cannot deadlock (same-host direct reads
        # ack in ~ms; cross-machine the ack rides the same zenoh path).
        node.next()

    # NOTE: the first iteration's next() (registration handshake) is
    # above; every subsequent frame handshakes in the else branch.
