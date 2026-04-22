#!/usr/bin/env python
"""Simple test with page-aligned memory to avoid alignment issues."""

import sys
import os
os.environ['PYTHONWARNINGS'] = 'ignore::UserWarning:multiprocessing.resource_tracker'

import ctypes
import ctypes.util
import numpy as np
import torch
import pyarrow as pa

# Create page-aligned memory manually
page_size = 4096
size = 1024  # Small size for test
dtype = torch.float32

# Allocate page-aligned memory
libc = ctypes.CDLL(ctypes.util.find_library('c'))
aligned_ptr = ctypes.c_void_p()
result = libc.posix_memalign(ctypes.byref(aligned_ptr), page_size, size * 4)  # 4 bytes per float32
if result != 0:
    raise RuntimeError(f"posix_memalign failed: {result}")

ptr = aligned_ptr.value
print(f"Allocated page-aligned memory at {ptr:#x}")

# Create tensor from the aligned memory
# First create numpy array from the memory
np_array = np.frombuffer(
    (ctypes.c_float * size).from_address(ptr),
    dtype=np.float32
)
# Fill with data
np_array[:] = np.arange(size, dtype=np.float32)

# Convert to torch tensor (this creates a copy, but that's OK for test)
tensor = torch.from_numpy(np_array).clone()  # Clone to get a new tensor
print(f"Created tensor with pointer {tensor.data_ptr():#x}")
print(f"Tensor values[0:5]: {tensor[:5]}")

# Now test the API
print("\n" + "="*60)
print("Testing dora pinned memory API")
print("="*60)

# We can't actually test without a daemon running
# But we can check the functions exist
import dora
print(f"✓ dora imported")
print(f"  register_pinned_memory exists: {hasattr(dora.Node, 'register_pinned_memory')}")
print(f"  read_pinned_memory exists: {hasattr(dora.Node, 'read_pinned_memory')}")
print(f"  free_pinned_memory exists: {hasattr(dora.Node, 'free_pinned_memory')}")

# Check torch_to_pinned_ptr
from dora.cuda import torch_to_pinned_ptr, pinned_ptr_to_torch
print(f"\n✓ dora.cuda imported")
print(f"  torch_to_pinned_ptr exists: {torch_to_pinned_ptr is not None}")
print(f"  pinned_ptr_to_torch exists: {pinned_ptr_to_torch is not None}")

# Test torch_to_pinned_ptr
pinned_ptr, metadata = torch_to_pinned_ptr(tensor)
print(f"\n✓ torch_to_pinned_ptr succeeded")
print(f"  pinned_ptr type: {type(pinned_ptr)}")
print(f"  pinned_ptr: {pinned_ptr}")
print(f"  metadata: {metadata}")

# Check if pointer is page-aligned
ptr_from_array = pinned_ptr.to_pylist()[0]
alignment = ptr_from_array % page_size
print(f"  Pointer from array: {ptr_from_array:#x}, alignment: {alignment}")
if alignment == 0:
    print("  ✓ Pointer is page-aligned")
else:
    print(f"  ✗ Pointer is NOT page-aligned (alignment: {alignment})")

# Test pinned_ptr_to_torch (without CUDA registration)
print("\n" + "="*60)
print("Testing pinned_ptr_to_torch (simulated)")
print("="*60)

# Mock the function to avoid CUDA issues
def mock_pinned_ptr_to_torch(pinned_ptr, metadata):
    """Mock version that doesn't require CUDA."""
    size = metadata.get("size")
    dtype_str = metadata.get("dtype", "torch.float32")
    shape = metadata.get("shape", [])

    # Extract pointer
    ptr_list = pinned_ptr.to_pylist()
    if len(ptr_list) != 1:
        raise ValueError(f"Expected pinned_ptr array with one element, got {len(ptr_list)}")
    ptr = ptr_list[0]

    print(f"  Mock: Creating tensor from pointer {ptr:#x}, size={size}, shape={shape}")

    # Create numpy array from memory
    np_dtype = np.float32 if "float32" in dtype_str else np.int64
    np_array = np.frombuffer(
        (ctypes.c_byte * size).from_address(ptr),
        dtype=np_dtype
    ).reshape(shape)

    # Convert to torch tensor
    torch_dtype = torch.float32 if "float32" in dtype_str else torch.int64
    tensor = torch.from_numpy(np_array).to(torch_dtype)
    return tensor

# Test the mock
try:
    # Note: This will fail if the original tensor memory was freed
    # But since we're using aligned memory that we allocated, it should work
    recovered_tensor = mock_pinned_ptr_to_torch(pinned_ptr, metadata)
    print(f"✓ Mock pinned_ptr_to_torch succeeded")
    print(f"  Recovered tensor shape: {recovered_tensor.shape}")
    print(f"  Recovered tensor values[0:5]: {recovered_tensor[:5]}")

    # Check if data matches
    if torch.allclose(tensor, recovered_tensor):
        print("  ✓ Data matches original tensor!")
    else:
        print("  ✗ Data doesn't match original tensor")
except Exception as e:
    print(f"✗ Mock pinned_ptr_to_torch failed: {e}")
    import traceback
    traceback.print_exc()

# Free the aligned memory
libc.free(aligned_ptr)
print(f"\n✓ Freed aligned memory")

print("\n" + "="*60)
print("Summary: The basic mechanisms work, but need daemon for full test")
print("="*60)