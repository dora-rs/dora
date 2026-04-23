#!/usr/bin/env python
"""Test large tensor registration."""

import os
os.environ['PYTHONWARNINGS'] = 'ignore::UserWarning:multiprocessing.resource_tracker'

import numpy as np
import torch
import pyarrow as pa
from dora import Node
from dora.cuda import torch_to_pinned_ptr

node = Node("test_node")

# Create large tensor similar to sender1.py
size = 15000 * 512  # 7,680,000 elements
random_data = np.random.default_rng().integers(1000, size=size, dtype=np.int64)
torch_tensor = torch.tensor(random_data, dtype=torch.int64, device="cpu")
print(f"Tensor shape: {torch_tensor.shape}, nbytes: {torch_tensor.nbytes}")

# Convert to pinned pointer
pinned_ptr, metadata = torch_to_pinned_ptr(torch_tensor)
print(f"Pointer: {pinned_ptr}, metadata keys: {list(metadata.keys())}")

# Add time (like sender)
import time
metadata["time"] = time.perf_counter_ns()

try:
    print("Calling register_pinned_memory...")
    pinned_buffer = node.register_pinned_memory(pinned_ptr, metadata)
    print(f"Registered pinned_buffer: {pinned_buffer}")

    # Try to read it back
    print("Calling read_pinned_memory...")
    read_ptr, read_metadata = node.read_pinned_memory(pinned_buffer, free=False)
    print(f"Read pointer: {read_ptr}, metadata keys: {list(read_metadata.keys())}")

    # Free
    print("Calling free_pinned_memory...")
    node.free_pinned_memory(pinned_buffer)
    print("Success!")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()