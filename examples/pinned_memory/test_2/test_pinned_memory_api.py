#!/usr/bin/env python
"""Test pinned memory API functions as per plan.md requirements.

Test content:
1. Test node.register_pinned_memory can correctly register page-locked memory and return valid pinned_buffer.
2. pinned_buffer can be found by node.read_pinned_memory in pinned_memory_table, and read valid pointer and metadata.
3. pinned_buffer can be found by node.free_pinned_memory in pinned_memory_table, and correctly free memory (no memory errors).
"""

import sys
import os
import traceback
import pyarrow as pa
import torch
import dora

def test_register_pinned_memory():
    """Test 1: node.register_pinned_memory registration."""
    print("=" * 60)
    print("Test 1: Testing node.register_pinned_memory")
    print("=" * 60)

    try:
        # Create a torch tensor
        tensor = torch.tensor([[1, 2, 3], [4, 5, 6]], dtype=torch.float32)
        print(f"Created tensor: shape={tensor.shape}, dtype={tensor.dtype}, pointer={tensor.data_ptr():#x}")

        # Use torch_to_pinned_ptr from dora.cuda to get pinned_ptr and metadata
        from dora.cuda import torch_to_pinned_ptr
        pinned_ptr, metadata = torch_to_pinned_ptr(tensor)
        print(f"Got pinned_ptr type: {type(pinned_ptr)}")
        print(f"Got metadata: {metadata}")

        # Create a dora Node
        node = dora.Node("test_node")
        print("Created dora Node")

        # Call node.register_pinned_memory(pinned_ptr, metadata) -> pinned_buffer
        pinned_buffer = node.register_pinned_memory(pinned_ptr, metadata)
        print(f"Registered pinned memory, got pinned_buffer type: {type(pinned_buffer)}")
        print(f"pinned_buffer: {pinned_buffer}")

        # Check if pinned_buffer is valid (should be a pyarrow array)
        if isinstance(pinned_buffer, pa.Array):
            print(f"✓ pinned_buffer is valid pyarrow array with type: {pinned_buffer.type}")
            # Convert to Python value to see what's inside
            buffer_value = pinned_buffer.to_pylist()
            print(f"  Buffer value: {buffer_value}")
            return True, node, pinned_buffer, metadata
        else:
            print(f"✗ pinned_buffer is not a pyarrow array: {type(pinned_buffer)}")
            return False, node, None, None

    except Exception as e:
        print(f"✗ Test 1 failed with error: {e}")
        traceback.print_exc()
        return False, None, None, None

def test_read_pinned_memory(node, pinned_buffer, original_metadata, free=False):
    """Test 2: node.read_pinned_memory reading."""
    print("\n" + "=" * 60)
    print(f"Test 2: Testing node.read_pinned_memory (free={free})")
    print("=" * 60)

    try:
        # Call node.read_pinned_memory(pinned_buffer, free=False) -> should return pinned_ptr and metadata
        result = node.read_pinned_memory(pinned_buffer, free=free)
        print(f"read_pinned_memory returned type: {type(result)}")

        # Check if result is a tuple
        if isinstance(result, tuple):
            print(f"✓ Result is tuple with {len(result)} elements")
            if len(result) == 2:
                read_pinned_ptr, read_metadata = result
                print(f"  read_pinned_ptr type: {type(read_pinned_ptr)}")
                print(f"  read_metadata type: {type(read_metadata)}")

                # Check if read_pinned_ptr is a pyarrow array
                if isinstance(read_pinned_ptr, pa.Array):
                    print(f"  ✓ read_pinned_ptr is pyarrow array with type: {read_pinned_ptr.type}")
                    ptr_value = read_pinned_ptr.to_pylist()
                    print(f"    Pointer value: {ptr_value}")
                else:
                    print(f"  ✗ read_pinned_ptr is not pyarrow array: {type(read_pinned_ptr)}")

                # Check if read_metadata is a dict
                if isinstance(read_metadata, dict):
                    print(f"  ✓ read_metadata is dict with keys: {list(read_metadata.keys())}")

                    # Compare that the recovered metadata matches original
                    print("\n  Comparing metadata:")
                    all_match = True
                    for key in original_metadata:
                        if key in read_metadata:
                            if original_metadata[key] == read_metadata[key]:
                                print(f"    ✓ {key}: {original_metadata[key]} == {read_metadata[key]}")
                            else:
                                print(f"    ✗ {key}: {original_metadata[key]} != {read_metadata[key]}")
                                all_match = False
                        else:
                            print(f"    ✗ {key} not found in read_metadata")
                            all_match = False

                    # Check for extra keys in read_metadata
                    for key in read_metadata:
                        if key not in original_metadata:
                            print(f"    Note: Extra key in read_metadata: {key} = {read_metadata[key]}")

                    if all_match:
                        print("  ✓ All metadata matches!")
                        return True, read_pinned_ptr, read_metadata
                    else:
                        print("  ✗ Metadata doesn't match")
                        return False, None, None
                else:
                    print(f"  ✗ read_metadata is not dict: {type(read_metadata)}")
                    return False, None, None
            else:
                print(f"  ✗ Tuple has wrong length: {len(result)}")
                return False, None, None
        else:
            print(f"✗ Result is not a tuple: {type(result)}")
            return False, None, None

    except Exception as e:
        print(f"✗ Test 2 failed with error: {e}")
        traceback.print_exc()
        return False, None, None

def test_free_pinned_memory(node, pinned_buffer):
    """Test 3: node.free_pinned_memory."""
    print("\n" + "=" * 60)
    print("Test 3: Testing node.free_pinned_memory")
    print("=" * 60)

    try:
        # Call node.free_pinned_memory(pinned_buffer)
        node.free_pinned_memory(pinned_buffer)
        print("✓ free_pinned_memory completed without error")

        # Try to read the freed buffer - should fail or return None
        print("\n  Trying to read freed memory (should fail):")
        try:
            result = node.read_pinned_memory(pinned_buffer, free=False)
            print(f"  ✗ WARNING: read_pinned_memory succeeded after free: {result}")
            print("  This might indicate memory wasn't properly freed")
            return False
        except Exception as e:
            print(f"  ✓ Good! read_pinned_memory failed as expected after free: {e}")
            return True

    except Exception as e:
        print(f"✗ Test 3 failed with error: {e}")
        traceback.print_exc()
        return False

def main():
    print("Starting comprehensive pinned memory API test")
    print("=" * 60)

    # Test 1: Register pinned memory
    success1, node, pinned_buffer, metadata = test_register_pinned_memory()
    if not success1 or node is None or pinned_buffer is None:
        print("\n✗ Test 1 failed, cannot continue with tests 2 and 3")
        return False

    # Test 2: Read pinned memory (without freeing)
    success2, read_pinned_ptr, read_metadata = test_read_pinned_memory(
        node, pinned_buffer, metadata, free=False
    )
    if not success2:
        print("\n✗ Test 2 failed")
        # Continue with test 3 anyway

    # Test 3: Free pinned memory
    success3 = test_free_pinned_memory(node, pinned_buffer)

    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"Test 1 (register_pinned_memory): {'✓ PASS' if success1 else '✗ FAIL'}")
    print(f"Test 2 (read_pinned_memory): {'✓ PASS' if success2 else '✗ FAIL'}")
    print(f"Test 3 (free_pinned_memory): {'✓ PASS' if success3 else '✗ FAIL'}")

    all_passed = success1 and success2 and success3
    if all_passed:
        print("\n✓ All tests passed!")
    else:
        print("\n✗ Some tests failed")

    return all_passed

if __name__ == "__main__":
    # Set environment variable to suppress warnings
    os.environ['PYTHONWARNINGS'] = 'ignore::UserWarning:multiprocessing.resource_tracker'

    success = main()
    sys.exit(0 if success else 1)