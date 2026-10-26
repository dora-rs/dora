"""Compare source-resident IPC and receiver-resident pools with full consumption.

Run the same script under two tensor-pool-enabled Python environments and one
CLI. This is a workload benchmark, NOT a pure-transfer benchmark: the second
endpoint includes receiver consumption, while SHA checks and ACK remain outside.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shlex
import statistics
import subprocess
import sys
import tempfile
import time


def node(role):
    import numpy as np
    import pyarrow as pa
    import torch
    from dora import Node
    from dora_tensor_pool import get_tensor_info, tensor_from_info

    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    torch.cuda.set_device(0 if role == 'sender' else 1)
    transport = Node(role)
    count, warmup = int(os.environ['SAMPLES']), int(os.environ['WARMUP'])
    elements = int(os.environ['PAYLOAD_MB']) * 1_000_000 // 8
    if role == 'sender':
        rng = np.random.default_rng(42)
        pool = None
        try:
            for sequence in range(count + warmup):
                data = rng.integers(0, 1000, size=elements, dtype=np.int64)
                data[0] = sequence
                expected = hashlib.sha256(data.tobytes()).hexdigest()
                tensor = torch.from_numpy(data).to('cuda:0')
                torch.cuda.synchronize(0)
                info = get_tensor_info(tensor)
                if pool is None:
                    pool = transport.register_tensor_pool(info, 'cuda:1')
                    if pool is None:
                        raise RuntimeError('registration failed')
                metadata = {'sequence': sequence, 'sha256': expected}
                metadata['send_ns'] = time.perf_counter_ns()
                transport.write_tensor_pool(pool, info)
                transport.send_output('data', pool if sequence == 0 else pa.array([]), metadata)
                event = transport.next()
                assert event['type'] == 'INPUT' and event['id'] == 'ack'
        finally:
            if pool is not None:
                transport.free_tensor_pool(pool)
    else:
        view, destination = None, None
        rows = []
        for sequence in range(count + warmup):
            event = transport.next()
            assert event['type'] == 'INPUT' and event['id'] == 'data'
            if view is None:
                view = tensor_from_info(transport.read_tensor_pool(event['value']))
                destination = torch.empty(view.shape, dtype=view.dtype, device='cuda:1')
            torch.cuda.synchronize(1)
            delivered = time.perf_counter_ns()
            if os.environ['WORKLOAD'] == 'sum':
                consumed = view.sum()
            else:
                destination.copy_(view)
            torch.cuda.synchronize(1)
            finished = time.perf_counter_ns()
            # Full verification is after BOTH endpoints, including the output.
            host = view.cpu() if os.environ['WORKLOAD'] == 'sum' else destination.cpu()
            assert hashlib.sha256(host.numpy().tobytes()).hexdigest() == event['metadata']['sha256']
            if os.environ['WORKLOAD'] == 'sum':
                assert int(consumed) == int(host.sum())
            assert int(host[0]) == sequence
            if sequence >= warmup:
                rows.append(dict(sample=sequence - warmup + 1, payload_bytes=view.nbytes,
                                 transfer_ns=delivered - event['metadata']['send_ns'],
                                 consumption_ns=finished - delivered,
                                 delivery_plus_consumption_ns=finished - event['metadata']['send_ns'],
                                 validation='sha256_ok'))
            transport.send_output('ack', pa.array([]))
        with Path(os.environ['RUN_DIR'], 'raw.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--role', choices=['sender', 'receiver'])
    parser.add_argument('--upstream-python', type=Path)
    parser.add_argument('--branch-python', type=Path)
    parser.add_argument('--dora', type=Path)
    parser.add_argument('--gpus', help='two physical GPU UUIDs/indices in source,target order')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--warmup', type=int, default=10)
    args = parser.parse_args()
    if args.role:
        node(args.role)
        return
    if not all([args.upstream_python, args.branch_python, args.dora, args.gpus, args.output]):
        parser.error('comparison needs both Python paths, --dora, --gpus and --output')
    if args.output.exists():
        parser.error('--output must be a new directory; comparisons must not overwrite earlier evidence')
    import yaml
    import torch
    if args.samples < 1 or args.warmup < 1:
        parser.error("samples and warmup must be positive")
    args.output.mkdir(parents=True)
    versions = [('upstream', args.upstream_python), ('receiver_pool', args.branch_python)]
    identity_code = "import dora,hashlib,json;from pathlib import Path;print(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(dora.__file__).parent.glob('*.so')}))"
    identities = {label: json.loads(subprocess.check_output([str(python), '-c', identity_code], text=True)) for label, python in versions}
    if identities['upstream'] == identities['receiver_pool']:
        parser.error('both Python environments load the same binding')
    (args.output / 'environment.json').write_text(json.dumps(dict(
        cli_sha256=hashlib.sha256(args.dora.read_bytes()).hexdigest(), bindings=identities,
        harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        gpu_inventory_before=subprocess.check_output(['nvidia-smi', '--query-gpu=index,uuid,utilization.gpu,memory.used', '--format=csv,noheader'], text=True),
        gpus=args.gpus, samples=args.samples, warmup=args.warmup, seed=42,
        note='Shared GPU workloads; sequential single-batch observations, not peak or controlled-isolation claims.'), indent=2))
    summary = []
    with tempfile.TemporaryDirectory(prefix='pool-consumption-') as temporary:
        compat = Path(temporary)
        runtime = Path(torch.__file__).resolve().parent.parent / 'nvidia/cuda_runtime/lib/libcudart.so.12'
        (compat / 'libcudart.so').symlink_to(runtime)
        for workload in ['sum', 'materialize']:
            for mb in [1, 10, 100]:
                # Alternate order to reduce (not eliminate) scheduling bias.
                for label, python in (versions if mb != 10 else list(reversed(versions))):
                    run = args.output / f'{workload}_{mb}MB_{label}'
                    run.mkdir()
                    env = dict(CUDA_VISIBLE_DEVICES=args.gpus, WORKLOAD=workload,
                               PAYLOAD_MB=str(mb), SAMPLES=str(args.samples), WARMUP=str(args.warmup),
                               RUN_DIR=str(run.resolve()), OMP_NUM_THREADS='1',
                               MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
                    nodes = [dict(id=role, path=str(python.absolute()),
                                  args=shlex.join(['-u', str(Path(__file__).resolve()), '--role', role]),
                                  inputs={'ack': 'receiver/ack'} if role == 'sender' else {'data': 'sender/data'},
                                  outputs=['data'] if role == 'sender' else ['ack']) for role in ['sender', 'receiver']]
                    config = run / 'flow.yml'
                    config.write_text(yaml.safe_dump(dict(env=env, nodes=nodes)))
                    process_env = dict(os.environ, **env, LD_LIBRARY_PATH=str(compat) + ':' + os.environ.get('LD_LIBRARY_PATH', ''))
                    with (run / 'run.log').open('w') as log:
                        subprocess.run([str(args.dora.resolve()), 'run', str(config.resolve()), '--stop-after', '180s'],
                                       cwd=run, env=process_env, stdout=log, stderr=subprocess.STDOUT,
                                       timeout=200, check=True)
                    rows = list(csv.DictReader((run / 'raw.csv').open()))
                    assert len(rows) == args.samples and all(r['validation'] == 'sha256_ok' for r in rows)
                    entry = dict(workload=workload, payload_MB=mb, condition=label, samples=len(rows),
                                 mean_transfer_MB_s=statistics.mean(int(r['payload_bytes']) * 1000 / int(r['transfer_ns']) for r in rows),
                                 mean_delivery_plus_consumption_MB_s=statistics.mean(int(r['payload_bytes']) * 1000 / int(r['delivery_plus_consumption_ns']) for r in rows),
                                 median_delivery_plus_consumption_us=statistics.median(int(r['delivery_plus_consumption_ns']) / 1000 for r in rows),
                                 raw_sha256=hashlib.sha256((run / 'raw.csv').read_bytes()).hexdigest())
                    summary.append(entry)
                    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
                    print(json.dumps(entry), flush=True)


if __name__ == '__main__':
    main()