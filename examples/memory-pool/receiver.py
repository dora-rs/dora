#!/usr/bin/env python
"""Receive tensors through the memory-pool example dataflow."""

import os
import time

import pyarrow as pa
import torch
from dora import Node
from dora.cuda import tensor_from_info
from tqdm import tqdm

NO_REUSE = os.environ.get("HETEROPOOL_NO_REUSE") == "1"
# Ablation: when HETEROPOOL_NO_FASTPATH=1, read_memory_pool forces the daemon
# slow path every frame (no DORADMA fast path, no view binding).
# Default True (fast path enabled = Full/-Pinned mode).
NO_FASTPATH = os.environ.get("HETEROPOOL_NO_FASTPATH", "0") == "1"

node = Node("receiver_node")
MESSAGE_COUNT = int(os.getenv("message_num", "100"))
RECEIVER_DEVICE = os.getenv("receiver_device", "cpu")
SCENARIO = os.getenv("memory_pool_scenario", "throughput")

if RECEIVER_DEVICE.startswith("cuda") and not torch.cuda.is_available():
    raise RuntimeError("CUDA is not available for the configured receiver device.")

pbar = tqdm(total=MESSAGE_COUNT)
velocities = []
memory_pool_id = None
prev_pool_id = None  # noreuse: track previous pool for receiver-side cleanup
torch_tensor = None

for i in range(MESSAGE_COUNT):
    event = node.next()

    # The event may lack 'metadata' when the sender node crashed or the
    # dataflow was stopped — dora delivers an input-closed control event
    # in that case, which has no metadata field.  Exit cleanly instead
    # of crashing with KeyError so the partial results are visible.
    if "metadata" not in event:
        print(
            f"Receiver: input closed at iteration {i}/{MESSAGE_COUNT} "
            f"(sender may have crashed)"
        )
        break

    t_send = event["metadata"]["t_send"]

    if NO_REUSE:
        # Ablation: sender creates a fresh pool each frame; read the new
        # pool_id from the event value every frame.
        #
        # Each read_memory_pool call caches the shmem mapping in the
        # receiver process (RECV_CPU_SHMEM / RECV_GPU_VA).  Without
        # explicit free, 100 iterations × pool_size leaks gigabytes of
        # mapped memory, causing SIGBUS when /dev/shm is exhausted.
        # Free the PREVIOUS pool's receiver-side mapping before opening
        # the new one (the sender already called free_memory_pool on it
        # after node.next() returned, so the daemon-side unlink is done;
        # our free_memory_pool only cleans up the local cache entry).
        if prev_pool_id is not None:
            try:
                node.free_memory_pool(prev_pool_id)
            except Exception:
                pass
        memory_pool_id = event["value"]
        # Always use if_fast=True — the daemon slow path cannot resolve
        # CUDA pointers, and this pool is freshly registered each frame.
        tensor_info = node.read_memory_pool(memory_pool_id, if_fast=True)
        torch_tensor = tensor_from_info(tensor_info)
        prev_pool_id = memory_pool_id
    elif i == 0:
        memory_pool_id = event["value"]
        # Use if_fast=True even in NO_FASTPATH mode — the daemon slow path
        # cannot resolve CUDA pointers (only provides CPU-side addresses).
        # The ablation measures per-frame tensor rebuild overhead in the
        # NO_FASTPATH branch (frames 1-99); frame 0 always needs correct
        # pointer resolution regardless of mode.
        tensor_info = node.read_memory_pool(memory_pool_id, if_fast=True)
        torch_tensor = tensor_from_info(tensor_info)
        print(f"Receiver preview: {torch_tensor[:5]}")
    elif NO_FASTPATH:
        # Ablation: pool is reused, but every frame we open the shmem,
        # parse the DORADMA header, and rebuild the tensor view — no
        # view caching, no reuse of the tensor object from frame 0.
        # We still use if_fast=True so that DORADMA correctly resolves
        # CUDA pointers (the daemon slow path only provides CPU pointers
        # and cannot service GPU pools).  The per-frame overhead comes
        # from the shmem lookup + header parse + tensor_from_info.
        tensor_info = node.read_memory_pool(memory_pool_id, if_fast=True)
        torch_tensor = tensor_from_info(tensor_info)

    # Record receive timestamp after per-frame processing (read_memory_pool
    # + tensor_from_info for nofastpath) but BEFORE .item() CUDA validation.
    # The sender's write_memory_pool already calls cudaDeviceSynchronize
    # after the DMA transfer, so the data is guaranteed to be in GPU memory.
    # Putting t_received before .item() keeps the CUDA synchronisation cost
    # of .item() out of the throughput window — that cost varies depending
    # on whether the tensor was freshly created (nofastpath) or reused
    # (fast-path), and would otherwise contaminate the comparison.
    t_received = time.perf_counter_ns()
    delta_t = t_received - t_send
    data_bytes = torch_tensor.nbytes
    velocity = data_bytes / (delta_t * 1e-9 * 1024 * 1024)
    velocities.append(velocity)

    # Tensor validation — correctness check only, outside measurement window.
    # The sender stamps element[0] with the iteration counter so we can
    # verify propagation deterministically (sum-of-8 had ~3% collision rate).
    if SCENARIO != "write_after_free":
        actual = int(torch_tensor[0].item())
        assert actual == i, (
            f"iteration {i}: tensor[0] expected {i}, got {actual}"
            " — pool write may not have propagated"
        )

    if NO_REUSE:
        # Ablation: sender creates a fresh pool each frame.
        # The sender frees the pool after node.next() returns below
        # (GPU resources like cudaMalloc/cudaHostRegister are per-process
        # and must be freed by the process that allocated them — the
        # receiver cannot reach sender-side GPU allocations).
        pass
    elif SCENARIO == "duplicate_free" and i == MESSAGE_COUNT - 1:
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
