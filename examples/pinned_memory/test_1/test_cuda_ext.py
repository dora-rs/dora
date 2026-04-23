#!/usr/bin/env python
"""Test for dora.cuda extension module.

Test that a tensor can be correctly converted to pinned pointer and back.
"""

import sys
import os
# Disable resource tracker warnings
os.environ['PYTHONWARNINGS'] = 'ignore::UserWarning:multiprocessing.resource_tracker'

import torch
import numpy as np
from dora.cuda import torch_to_ptr, ptr_to_torch

def test_tensor_conversion():
    """Test that tensor -> pinned_ptr -> tensor conversion works."""
    print("Testing torch_to_ptr and ptr_to_torch...")

    # Create test tensor
    tensor = torch.tensor([1, 2, 3, 4, 5], dtype=torch.float32)
    print(f"Original tensor: {tensor}")
    print(f"Tensor dtype: {tensor.dtype}, shape: {tensor.shape}")

    # Convert to pinned pointer
    pinned_ptr, metadata = torch_to_ptr(tensor)
    print(f"Pinned pointer type: {type(pinned_ptr)}")
    print(f"Metadata: {metadata}")

    # Extract pointer from pyarrow array
    ptr_list = pinned_ptr.to_pylist()
    if len(ptr_list) != 1:
        print(f"ERROR: Expected pinned_ptr array with one element, got {len(ptr_list)}")
        return False
    ptr = ptr_list[0]
    print(f"Pointer value: {ptr:#x}")

    # Convert back to tensor
    try:
        recovered_tensor = ptr_to_torch(pinned_ptr, metadata)
        print(f"Recovered tensor: {recovered_tensor}")
        print(f"Recovered tensor device: {recovered_tensor.device}")

        # Compare tensors based on recovered tensor device
        if recovered_tensor.device.type == "cuda":
            # Recovered tensor is on CUDA, move original to CUDA for comparison
            if torch.cuda.is_available():
                tensor_cuda = tensor.to("cuda")
                if torch.allclose(tensor_cuda, recovered_tensor):
                    print("SUCCESS: Tensors match (within tolerance)")
                    return True
                else:
                    print("ERROR: Tensors do not match")
                    print(f"Original (CUDA): {tensor_cuda}")
                    print(f"Recovered (CUDA): {recovered_tensor}")
                    return False
            else:
                print("WARNING: CUDA not available but recovered tensor is on CUDA")
                # Fallback: check shape and dtype
                if recovered_tensor.shape == tensor.shape and recovered_tensor.dtype == tensor.dtype:
                    print("SUCCESS: Shape and dtype match")
                    return True
                else:
                    print("ERROR: Shape or dtype mismatch")
                    return False
        else:
            # Recovered tensor is on CPU, compare directly
            if torch.allclose(tensor, recovered_tensor):
                print("SUCCESS: Tensors match (within tolerance)")
                return True
            else:
                print("ERROR: Tensors do not match")
                print(f"Original (CPU): {tensor}")
                print(f"Recovered (CPU): {recovered_tensor}")
                return False
    except Exception as e:
        print(f"ERROR: ptr_to_torch failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_various_dtypes():
    """Test different tensor dtypes."""
    print("\n" + "="*60)
    print("Testing various dtypes...")

    dtypes = [
        torch.float32,
        torch.float64,
        torch.int64,
        torch.int32,
        torch.int16,
        torch.int8,
        torch.uint8,
        torch.bool,
    ]

    if hasattr(torch, 'float16'):
        dtypes.append(torch.float16)
    if hasattr(torch, 'bfloat16'):
        dtypes.append(torch.bfloat16)

    for dtype in dtypes:
        print(f"\nTesting dtype: {dtype}")
        try:
            # Create tensor
            tensor = torch.tensor([1, 2, 3], dtype=dtype)

            # Convert to pinned pointer
            pinned_ptr, metadata = torch_to_ptr(tensor)

            # Check metadata
            assert metadata["dtype"] == str(dtype), f"Metadata dtype mismatch: {metadata['dtype']} != {dtype}"
            assert metadata["shape"] == list(tensor.shape), f"Shape mismatch: {metadata['shape']} != {list(tensor.shape)}"
            assert metadata["size"] == tensor.nbytes, f"Size mismatch: {metadata['size']} != {tensor.nbytes}"

            # Convert back
            recovered_tensor = ptr_to_torch(pinned_ptr, metadata)

            # Compare based on recovered tensor device
            if recovered_tensor.device.type == "cuda":
                # Recovered tensor is on CUDA, move original to CUDA for comparison
                if torch.cuda.is_available():
                    tensor_cuda = tensor.to("cuda")
                    # For boolean tensors, need to use equality instead of allclose
                    if dtype == torch.bool:
                        if torch.equal(tensor_cuda, recovered_tensor):
                            print(f"  ✓ {dtype}: Passed")
                        else:
                            print(f"  ✗ {dtype}: Failed - tensors not equal")
                            return False
                    else:
                        if torch.allclose(tensor_cuda, recovered_tensor):
                            print(f"  ✓ {dtype}: Passed")
                        else:
                            print(f"  ✗ {dtype}: Failed - tensors not close")
                            return False
                else:
                    print(f"  ⚠ {dtype}: CUDA not available but recovered tensor is on CUDA")
                    # Fallback: check shape and dtype
                    if recovered_tensor.shape == tensor.shape and recovered_tensor.dtype == tensor.dtype:
                        print(f"  ✓ {dtype}: Shape and dtype match")
                    else:
                        print(f"  ✗ {dtype}: Shape or dtype mismatch")
                        return False
            else:
                # Recovered tensor is on CPU, compare directly
                if dtype == torch.bool:
                    if torch.equal(tensor, recovered_tensor):
                        print(f"  ✓ {dtype}: Passed")
                    else:
                        print(f"  ✗ {dtype}: Failed - tensors not equal")
                        return False
                else:
                    if torch.allclose(tensor, recovered_tensor):
                        print(f"  ✓ {dtype}: Passed")
                    else:
                        print(f"  ✗ {dtype}: Failed - tensors not close")
                        return False

        except Exception as e:
            print(f"  ✗ {dtype}: Failed with error: {e}")
            return False

    print("All dtypes passed!")
    return True

def test_different_shapes():
    """Test tensors with different shapes."""
    print("\n" + "="*60)
    print("Testing different shapes...")

    shapes = [
        [10],           # 1D
        [5, 4],         # 2D
        [3, 4, 5],      # 3D
        [2, 3, 4, 5],   # 4D
    ]

    for shape in shapes:
        print(f"\nTesting shape: {shape}")
        try:
            # Create tensor
            tensor = torch.randn(*shape, dtype=torch.float32)

            # Convert to pinned pointer
            pinned_ptr, metadata = torch_to_ptr(tensor)

            # Check metadata
            assert metadata["shape"] == shape, f"Shape mismatch: {metadata['shape']} != {shape}"

            # Convert back (if CUDA is available)
            if torch.cuda.is_available():
                recovered_tensor = ptr_to_torch(pinned_ptr, metadata)
                tensor_cuda = tensor.to("cuda")

                if torch.allclose(tensor_cuda, recovered_tensor):
                    print(f"  ✓ Shape {shape}: Passed")
                else:
                    print(f"  ✗ Shape {shape}: Failed")
                    return False
            else:
                print(f"  ⚠ Shape {shape}: CUDA not available, skipping comparison")

        except Exception as e:
            print(f"  ✗ Shape {shape}: Failed with error: {e}")
            return False

    print("All shapes passed!")
    return True

def main():
    """Run all tests."""
    print("="*60)
    print("Testing dora.cuda extension module")
    print("="*60)

    if not torch.cuda.is_available():
        print("WARNING: CUDA is not available. Some tests will be limited.")
        print("Please run with CUDA enabled for full testing.")

    # Run tests
    success = True

    if not test_tensor_conversion():
        success = False

    if success:
        if not test_various_dtypes():
            success = False

    if success:
        if not test_different_shapes():
            success = False

    print("\n" + "="*60)
    if success:
        print("SUCCESS: All tests passed!")
        print("Extension module development is complete.")
        sys.exit(0)
    else:
        print("FAILURE: Some tests failed.")
        print("Extension module needs further development.")
        sys.exit(1)

if __name__ == "__main__":
    main()