"""Hardware regression: target-device allocation and persistent-view updates.

Run cuda_target_pool.yml with a tensor-pool-enabled wheel/CLI and two P2P GPUs.
Set SOURCE_DEVICE=1 and TARGET_DEVICE=0 to exercise the reverse direction.
This checks correctness, not performance; hashing is outside any benchmark.
"""
import ctypes
import hashlib
import os
import sys

import pyarrow as pa
import torch
from dora import Node
from dora_tensor_pool import get_tensor_info, tensor_from_info

source = int(os.getenv('SOURCE_DEVICE', '0'))
target = int(os.getenv('TARGET_DEVICE', '1'))
role = os.environ['ROLE']
elements = int(os.getenv('ELEMENT_COUNT', '65536'))
assert torch.cuda.device_count() >= 2, 'requires two visible CUDA devices'
assert source == target or torch.cuda.can_device_access_peer(source, target), 'requires P2P'
node = Node(role)

if role == 'sender':
    pool = None
    try:
        for sequence in range(4):
            tensor = torch.randint(0, 1000, (elements,), device=f'cuda:{source}')
            tensor[0] = sequence
            expected = hashlib.sha256(tensor.cpu().numpy().tobytes()).hexdigest()
            # Deliberately make the ambient device differ from the source.
            ambient = 1 - source
            torch.cuda.set_device(ambient)
            info = get_tensor_info(tensor)
            if pool is None:
                pool = node.register_tensor_pool(info, f'cuda:{target}')
                assert pool is not None
                helper = sys.modules['_cuda_helpers']
                pointer = next(iter(helper._gpu_bufs.values()))[0]

                class Attributes(ctypes.Structure):
                    _fields_ = [('type', ctypes.c_int), ('device', ctypes.c_int),
                                ('device_pointer', ctypes.c_void_p),
                                ('host_pointer', ctypes.c_void_p)]

                attributes = Attributes()
                query = helper._lib.cudaPointerGetAttributes
                query.argtypes = [ctypes.POINTER(Attributes), ctypes.c_void_p]
                query.restype = ctypes.c_int
                assert query(ctypes.byref(attributes), ctypes.c_void_p(pointer)) == 0
                # Query the original allocation in its owning process, not an
                # IPC-imported pointer whose reported device can be misleading.
                assert attributes.device == target, (
                    f'pool allocated on GPU {attributes.device}, expected {target}')
                print(f'Pool allocation verified on cuda:{target}', flush=True)
            else:
                node.write_tensor_pool(pool, info)
            assert torch.cuda.current_device() == ambient, 'ambient device changed'
            node.send_output('data', pool if sequence == 0 else pa.array([]),
                             {'sequence': sequence, 'sha256': expected})
            event = node.next()
            assert event['type'] == 'INPUT' and event['id'] == 'ack'
    finally:
        if pool is not None:
            node.free_tensor_pool(pool)
else:
    torch.cuda.set_device(target)
    view = None
    for sequence in range(4):
        event = node.next()
        assert event['type'] == 'INPUT' and event['id'] == 'data'
        if view is None:
            view = tensor_from_info(node.read_tensor_pool(event['value']))
            assert view.device == torch.device(f'cuda:{target}')
            pointer = view.data_ptr()
        assert view.data_ptr() == pointer, 'view was replaced'
        actual = hashlib.sha256(view.cpu().numpy().tobytes()).hexdigest()
        assert actual == event['metadata']['sha256'], 'full payload mismatch'
        assert int(view[0]) == sequence
        node.send_output('ack', pa.array([]))
    print('Four full-payload checks passed using one receiver view', flush=True)