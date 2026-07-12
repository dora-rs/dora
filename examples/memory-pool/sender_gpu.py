#!/usr/bin/env python
"""GPU-to-GPU sender — uses memory_pool for IPC/P2P tensor transfer."""

import os, time, numpy as np, pyarrow as pa, torch
from dora import Node
from dora.cuda import get_tensor_info

TENSOR_BYTES = int(os.getenv("TENSOR_BYTES", str(80 * 1024 * 1024)))  # default 80MB
SIZE = TENSOR_BYTES // 8
MESSAGE_COUNT = int(os.getenv("message_num", "100"))
SENDER_DEVICE = os.getenv("sender_device", "cuda:0")
RECEIVER_DEVICE = os.getenv("receiver_device", "cuda:0")

node = Node("sender_node")
rng = np.random.default_rng()

memory_pool_id = None
for i in range(MESSAGE_COUNT):
    data = rng.integers(1000, size=SIZE, dtype=np.int64)
    data[0] = i
    tensor = torch.tensor(data, dtype=torch.int64, device=SENDER_DEVICE)
    t_send = time.perf_counter_ns()
    info = get_tensor_info(tensor)

    if i == 0:
        print(f"[sender] GPU {SENDER_DEVICE} → GPU {RECEIVER_DEVICE}, "
              f"tensor {TENSOR_BYTES/1024/1024:.0f}MB × {MESSAGE_COUNT} frames")
        print(f"[sender] preview: {tensor[:5]}")
        memory_pool_id = node.register_memory_pool(info, RECEIVER_DEVICE)
        node.send_output("data", memory_pool_id, {"t_send": t_send})
    else:
        node.write_memory_pool(memory_pool_id, info)
        node.send_output("data", pa.array([]), {"t_send": t_send})

    event = node.next()
    if "metadata" not in event:
        print(f"[sender] input closed at frame {i}")
        break

    if i == MESSAGE_COUNT - 1:
        node.free_memory_pool(memory_pool_id)

print(f"[sender] done, {MESSAGE_COUNT} frames sent")
