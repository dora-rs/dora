"""CPU-only regression tests for the CUDA helpers embedded in transport.rs.

Run: python -m unittest discover -s libraries/extensions/tensor-pool/tests
The fake driver checks copy devices, completion, and error propagation; it does
not claim to validate GPU hardware. The companion dataflow covers real GPUs.
"""
import ctypes
from pathlib import Path
import re
import unittest
from unittest.mock import patch


class Function:
    def __init__(self, function):
        self.function = function

    def __call__(self, *args):
        return self.function(*args)


class Cuda:
    def __init__(self):
        self.device = 0
        self.next_ptr = 4096
        self.calls = []
        self.copy_error = 0
        self.sync_error = 0
        self.cudaMalloc = Function(self.malloc)
        self.cudaGetDevice = Function(self.get_device)
        self.cudaSetDevice = Function(self.set_device)
        self.cudaMemcpy = Function(lambda *args: self.copy('copy', args))
        self.cudaMemcpyPeer = Function(lambda *args: self.copy('peer', args))
        self.cudaDeviceSynchronize = Function(self.synchronize)

    def __getattr__(self, name):
        function = Function(lambda *args: 0)
        setattr(self, name, function)
        return function

    def malloc(self, pointer, size):
        pointer._obj.value = self.next_ptr
        self.calls.append(('malloc', self.device, size))
        self.next_ptr += 4096
        return 0

    def get_device(self, pointer):
        pointer._obj.value = self.device
        return 0

    def set_device(self, device):
        self.device = device
        return 0

    def copy(self, kind, args):
        self.calls.append((kind, self.device, tuple(
            arg.value if isinstance(arg, ctypes.c_void_p) else arg for arg in args)))
        return self.copy_error

    def synchronize(self):
        self.calls.append(('sync', self.device))
        return self.sync_error


class TargetGpuTests(unittest.TestCase):
    def setUp(self):
        source = (Path(__file__).resolve().parents[1] / 'python/src/transport.rs').read_text()
        code = re.search(r'let code = r#"(.*?)"#;', source, re.S).group(1)
        self.cuda = Cuda()
        self.helpers = {}
        with patch('ctypes.CDLL', return_value=self.cuda):
            exec(compile(code, 'tensor_pool_cuda_helpers', 'exec'), self.helpers)

    def allocate(self, device):
        self.cuda.device = device
        return self.helpers['_get_gpu_buf'](7, 1024)

    def write(self, source_device):
        self.helpers['_cuda_memcpy_gpu_buf'](7, 8192, 1024, source_device)

    def test_cross_gpu_copy_targets_pool_device_and_synchronizes(self):
        pointer = self.allocate(1)
        self.cuda.device = 0
        self.write(0)
        self.assertIn(('peer', 1, (pointer, 1, 8192, 0, 1024)), self.cuda.calls)
        self.assertEqual(self.cuda.calls[-1], ('sync', 1))
        self.assertEqual(self.cuda.device, 0)

    def test_reverse_direction_including_target_zero(self):
        pointer = self.allocate(0)
        self.cuda.device = 1
        self.write(1)
        self.assertIn(('peer', 0, (pointer, 0, 8192, 1, 1024)), self.cuda.calls)
        self.assertEqual(self.cuda.device, 1)

    def test_same_gpu_keeps_device_to_device_copy(self):
        pointer = self.allocate(1)
        self.cuda.device = 0
        self.write(1)
        self.assertIn(('copy', 1, (pointer, 8192, 1024, 3)), self.cuda.calls)
        self.assertFalse(any(call[0] == 'peer' for call in self.cuda.calls))
        self.assertEqual(self.cuda.device, 0)

    def test_host_staging_still_uses_receiver_allocation(self):
        pointer = self.allocate(1)
        self.cuda.device = 0
        self.helpers['_transit_copy_gpu_buf'](7, 8192, 0, 12288, 1, 1024)
        self.assertIn(('copy', 0, (12288, 8192, 1024, 2)), self.cuda.calls)
        self.assertIn(('copy', 1, (pointer, 12288, 1024, 1)), self.cuda.calls)
        self.assertEqual(self.cuda.device, 0)

    def test_peer_copy_failure_propagates_and_restores_device(self):
        self.allocate(1)
        self.cuda.device = 0
        self.cuda.copy_error = 217
        with self.assertRaises(RuntimeError):
            self.write(0)
        self.assertEqual(self.cuda.device, 0)
        self.assertFalse(any(call[0] == 'sync' for call in self.cuda.calls))

    def test_completion_failure_propagates_and_restores_device(self):
        self.allocate(1)
        self.cuda.device = 0
        self.cuda.sync_error = 700
        with self.assertRaises(RuntimeError):
            self.write(0)
        self.assertEqual(self.cuda.device, 0)

    def test_capacity_guard_runs_before_copy(self):
        self.allocate(1)
        with self.assertRaises(RuntimeError):
            self.helpers['_cuda_memcpy_gpu_buf'](7, 8192, 1025, 0)
        self.assertFalse(any(call[0] in ('copy', 'peer') for call in self.cuda.calls))

    def test_allocation_device_is_retained_when_slot_is_reused(self):
        pointer = self.allocate(1)
        self.cuda.device = 0
        self.assertEqual(self.helpers['_get_gpu_buf'](7, 512), pointer)
        self.write(0)
        self.assertIn(('peer', 1, (pointer, 1, 8192, 0, 1024)), self.cuda.calls)
        self.assertEqual(sum(call[0] == 'malloc' for call in self.cuda.calls), 1)


if __name__ == '__main__':
    unittest.main()