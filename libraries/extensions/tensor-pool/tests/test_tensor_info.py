"""Regression tests for the pointer lifetime of `get_tensor_info`.

Run: python -m unittest discover -s libraries/extensions/tensor-pool/tests
Skipped where torch, numpy or pyarrow is not installed (the CI interpreter);
the CUDA case also needs a GPU.
"""
import ctypes
import importlib.util
import sys
import unittest
from pathlib import Path

DEPS = all(importlib.util.find_spec(name) for name in ('torch', 'numpy', 'pyarrow'))
if DEPS:
    import torch

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
    from dora_tensor_pool import get_tensor_info, tensor_from_info


@unittest.skipUnless(DEPS, 'requires torch, numpy and pyarrow')
class TensorInfoLifetimeTests(unittest.TestCase):
    def assert_info_outlives_reallocation(self, info, expected, device):
        # `expected` must already exist: allocated after get_tensor_info, it
        # could take over a freed block and refill it with the right values.
        # Same-size allocations are what the allocator hands a freed block to.
        fillers = [torch.full(expected.shape, -7.0, device=device) for _ in range(8)]
        self.assertTrue(torch.equal(tensor_from_info(info), expected))
        if device == 'cpu':
            # The bytes register_tensor_pool / write_tensor_pool copy from ptr.
            data = ctypes.string_at(info['ptr'], info['size'])
            self.assertEqual(data, expected.numpy().tobytes())
        del fillers

    def test_non_contiguous_tensor(self):
        source = torch.arange(64 * 48, dtype=torch.float32).reshape(64, 48).T
        expected = source.contiguous()
        info = get_tensor_info(source)
        self.assert_info_outlives_reallocation(info, expected, 'cpu')

    def test_temporary_tensor(self):
        expected = torch.arange(64 * 48, dtype=torch.float32)
        info = get_tensor_info(torch.arange(64 * 48, dtype=torch.float32))
        self.assert_info_outlives_reallocation(info, expected, 'cpu')

    @unittest.skipUnless(DEPS and torch.cuda.is_available(), 'requires a CUDA device')
    def test_non_contiguous_cuda_tensor(self):
        source = torch.arange(64 * 48, dtype=torch.float32, device='cuda').reshape(64, 48).T
        expected = source.contiguous()
        info = get_tensor_info(source)
        self.assert_info_outlives_reallocation(info, expected, 'cuda')


if __name__ == '__main__':
    unittest.main()
