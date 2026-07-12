#!/usr/bin/env python
"""GPU-to-GPU receiver — reads tensors from memory_pool."""

import os, time, pyarrow as pa, torch
from dora import Node
from dora.cuda import tensor_from_info

MESSAGE_COUNT = int(os.getenv("message_num", "100"))
RECEIVER_DEVICE = os.getenv("receiver_device", "cuda:0")

node = Node("receiver_node")
memory_pool_id = None
torch_tensor = None
velocities = []

for i in range(MESSAGE_COUNT):
    event = node.next()
    if "metadata" not in event:
        print(f"[receiver] input closed at frame {i}")
        break

    t_send = event["metadata"]["t_send"]

    if i == 0:
        memory_pool_id = event["value"]
        info = node.read_memory_pool(memory_pool_id)
        torch_tensor = tensor_from_info(info)
        print(f"[receiver] GPU {RECEIVER_DEVICE}, preview: {torch_tensor[:5]}")
    else:
        # Pool is reused, tensor view auto-updates
        info = node.read_memory_pool(memory_pool_id)
        torch_tensor = tensor_from_info(info)

    t_recv = time.perf_counter_ns()
    delta_ns = t_recv - t_send
    data_bytes = torch_tensor.nbytes
    velocity = data_bytes / (delta_ns * 1e-9 * 1024 * 1024)
    velocities.append(velocity)

    actual = int(torch_tensor[0].item())
    assert actual == i, f"frame {i}: expected {i}, got {actual}"

    node.send_output("next", pa.array([]))

avg = torch.mean(torch.tensor(velocities, dtype=torch.float64))
print(f"Average transfer throughput: {avg:.6f} MB/s")
