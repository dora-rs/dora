#!/usr/bin/env python
"""Send tensors through the memory-pool example dataflow."""

import os
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
    t_send = time.perf_counter_ns()
    metadata = {"t_send": t_send, "scenario": SCENARIO}

    if i == 0:
        print(f"Sender preview: {torch_tensor[:5]}")
        tensor_info = get_tensor_info(torch_tensor)
        memory_pool_id = node.register_memory_pool(tensor_info, RECEIVER_DEVICE)
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

    # Cross-machine: this fork's next() does not gate on next_require, so
    # the writes race ahead of the receiver's reads and overwrite the pool
    # before the receiver consumes each frame. Pace the writes well beyond
    # the receiver's per-iteration read latency (observed ~5s under host
    # contention) so its re-read always finds the expected frame.
    time.sleep(20.0)

    node.next()
