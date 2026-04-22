#!/usr/bin/env python
"""Minimal test to check what read_pinned_memory returns."""

import sys
import os
os.environ['PYTHONWARNINGS'] = 'ignore::UserWarning:multiprocessing.resource_tracker'

# First, let's check if we can import and create a node
try:
    import dora
    print("✓ Successfully imported dora")

    # Check methods
    print(f"Node has register_pinned_memory: {hasattr(dora.Node, 'register_pinned_memory')}")
    print(f"Node has read_pinned_memory: {hasattr(dora.Node, 'read_pinned_memory')}")
    print(f"Node has free_pinned_memory: {hasattr(dora.Node, 'free_pinned_memory')}")

    # Try to create a node (will fail without daemon)
    try:
        node = dora.Node("test_node")
        print("✓ Created node (but will fail to connect to daemon)")

        # Create a dummy tensor and try the API
        import torch
        import pyarrow as pa
        from dora.cuda import torch_to_pinned_ptr

        tensor = torch.tensor([1, 2, 3], dtype=torch.float32)
        print(f"Created tensor: {tensor}")

        pinned_ptr, metadata = torch_to_pinned_ptr(tensor)
        print(f"Got pinned_ptr: {type(pinned_ptr)}")
        print(f"Got metadata: {metadata}")

        # This will fail because no daemon is running
        try:
            pinned_buffer = node.register_pinned_memory(pinned_ptr, metadata)
            print(f"Registered: {pinned_buffer}")
        except Exception as e:
            print(f"register_pinned_memory failed (expected): {type(e).__name__}: {e}")

    except Exception as e:
        print(f"✗ Node creation failed: {type(e).__name__}: {e}")

except Exception as e:
    print(f"✗ Import failed: {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()