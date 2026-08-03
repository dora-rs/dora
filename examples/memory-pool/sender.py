#!/usr/bin/env python
"""Send tensors through the memory-pool example dataflow."""

import os
import sys
import threading
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
        memory_pool_id = node.register_memory_pool(
            tensor_info, RECEIVER_DEVICE, machine=os.getenv("cross_machine")
        )
        if memory_pool_id is None:
            print(
                "Cross-machine register failed (warned, no pool created) — exiting",
                flush=True,
            )
            sys.exit(1)
        # Cross-machine: the register's proxy push can be lost while the
        # remote daemon's subscription is still replicating (observed as
        # the receiver reading the *next* write's data at iteration 0).
        # Keep re-pushing the registration data until the receiver has
        # consumed it (signalled by next_require arriving on next()).
        stop = threading.Event()

        def re_push():
            while not stop.is_set():
                time.sleep(0.5)
                try:
                    node.write_memory_pool(memory_pool_id, tensor_info)
                except Exception:
                    pass

        repush_thread = threading.Thread(target=re_push, daemon=True)
        repush_thread.start()
        node.send_output("data", memory_pool_id, metadata)
        node.next()
        stop.set()
    else:
        tensor_info = get_tensor_info(torch_tensor)
        if SCENARIO == "write_after_free" and i == 1:
            node.free_memory_pool(memory_pool_id)
        node.write_memory_pool(memory_pool_id, tensor_info)
        node.send_output("data", pa.array([]), metadata)

    # Cross-machine: the writes must not race ahead of the receiver's
    # reads (the proxy pool is overwritten per frame). Pace the writes
    # well beyond the receiver's per-iteration read latency (observed ~5s
    # under host contention) so its re-read always finds the expected
    # frame.  NOTE: no trailing next() here — it would wait for the next
    # iteration's next_require, which the receiver only sends after the
    # next latency output, which this loop hasn't produced yet: a
    # self-deadlock (observed: sender stuck at the second next() while
    # the receiver waits for the next latency).
    time.sleep(20.0)
