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
if RECEIVER_DEVICE.startswith("cuda"):
    idx = int(RECEIVER_DEVICE.split(":")[1]) if ":" in RECEIVER_DEVICE else 0
    torch.cuda.set_device(idx)

pbar = tqdm(total=MESSAGE_COUNT)
velocities = []
memory_pool_id = None
torch_tensor = None

for i in range(MESSAGE_COUNT):
    event = node.next()
    t_send = event["metadata"]["t_send"]

    if i == 0:
        memory_pool_id = event["value"]
        # First read: the registration push races the data event over
        # zenoh, so the mirror may not hold frame 0 yet.  Retry with the
        # same frame-order guard as the steady-state reads below — a
        # first-frame read of an empty or stale mirror would otherwise
        # corrupt iteration 0.
        deadline = time.monotonic() + 300
        while time.monotonic() < deadline:
            # The pool may not be ready yet: the registration push
            # travels the relay (or a direct-TCP fallback) and can
            # take seconds on a WAN. read_tensor_pool raises after
            # its own 500ms retry window, so catch and retry until
            # the 300s deadline.
            try:
                tensor_info = node.read_tensor_pool(memory_pool_id)
            except RuntimeError:
                time.sleep(0.1)
                continue
            torch_tensor = tensor_from_info(tensor_info)
            if int(torch_tensor[0].item()) == 0:
                break
        else:
            raise AssertionError(
                "iteration 0: expected frame 0 never arrived within the retry window"
            )
        print(f"Receiver preview: {torch_tensor[:5]}")
    else:
        # The zero-copy in-place update only holds for local shmem views.
        # Cross-machine reads go through the daemon-mirrored pool on this
        # host, so the tensor must be re-read (and re-built) each
        # iteration.  The memory-pool event trails the latency output on a
        # WAN (separate topics, no ordering guarantee) — and the mirror
        # write may lag the notification — so a read can return the
        # *previous* frame.  Retry until the expected frame arrives; each
        # read reflects the mirror's current generation.
        # Time-boxed, not count-boxed: on a WAN the mirror write lags the
        # notification, so a count window can burn through before the data
        # lands; 300s covers a slow WAN round trip and caps waits at ~5
        # minutes. Same-host direct reads ack in ~ms, so the retry exits on
        # the first pass there. Monotonic clock: an NTP step-back in the
        # window would otherwise shrink (or stretch) the wall-clock retry
        # window.
        deadline = time.monotonic() + 300
        while time.monotonic() < deadline:
            # Same retry-on-raise as the first frame: the mirror write
            # may lag the notification on a WAN.
            try:
                tensor_info = node.read_tensor_pool(memory_pool_id)
            except RuntimeError:
                time.sleep(0.1)
                continue
            torch_tensor = tensor_from_info(tensor_info)
            if int(torch_tensor[0].item()) == i:
                break
        else:
            raise AssertionError(
                f"iteration {i}: expected frame {i} never arrived within the retry window"
            )

    # The tensor is zero-copy — write_tensor_pool on the sender overwrites
    # the shmem bytes in place, so the receiver's existing tensor object
    # automatically reflects new data.  Turn-based signaling ensures the
    # sender has finished writing before the receiver accesses the tensor.
    # The sender stamps element[0] with the iteration counter so we can
    # verify propagation deterministically (sum-of-8 had ~3% collision rate).
    if SCENARIO != "write_after_free":
        actual = int(torch_tensor[0].item())
        assert actual == i, (
            f"iteration {i}: tensor[0] expected {i}, got {actual}"
            " — pool write may not have propagated"
        )

    # Wall clock for cross-machine deltas (see sender.py note)
    t_received = time.time_ns()
    delta_t = t_received - t_send
    data_bytes = torch_tensor.nbytes
    velocity = data_bytes / (delta_t * 1e-9 * 1024 * 1024)
    velocities.append(velocity)

    if SCENARIO == "duplicate_free" and i == MESSAGE_COUNT - 1:
        node.free_tensor_pool(memory_pool_id)
        node.free_tensor_pool(memory_pool_id)
    elif SCENARIO == "read_after_free" and i == MESSAGE_COUNT - 1:
        node.free_tensor_pool(memory_pool_id)
        try:
            node.read_tensor_pool(memory_pool_id)
        except Exception:
            pass  # Expected: pool was freed, read should fail
    elif SCENARIO != "auto_cleanup" and i == MESSAGE_COUNT - 1:
        node.free_tensor_pool(memory_pool_id)

    node.send_output("next_require", pa.array([]))
    pbar.update(1)

pbar.close()
average_velocity = torch.mean(torch.tensor(velocities, dtype=torch.float64))
print(f"Average transfer throughput: {average_velocity:1f} MB/s")
