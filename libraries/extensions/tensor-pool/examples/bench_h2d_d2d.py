#!/usr/bin/env python
"""Time Host→device and same-device D2D with the example tensor size.

Does not start a dataflow. Run inside the tensor-pool venv on the NPU/GPU box:

    python examples/bench_h2d_d2d.py
    python examples/bench_h2d_d2d.py --device npu:0
    python examples/bench_h2d_d2d.py --device cuda:0

Prints MB/s with 1024*1024, same unit as receiver.py.
"""

from __future__ import annotations

import argparse
import ctypes
import time

import numpy as np
import torch

# Same payload as sender.py / receiver.py.
SIZE = 15000 * 512
BYTES = SIZE * 8
WARMUP = 5
ITERS = 20


def mb_per_s(seconds: float) -> float:
    return BYTES / (seconds * 1024 * 1024)


def mean_s(samples: list[float]) -> float:
    return sum(samples) / len(samples)


def sync_device(kind: str) -> None:
    if kind == "npu":
        torch.npu.synchronize()
    elif kind == "cuda":
        torch.cuda.synchronize()


def bench_torch(device: str, kind: str) -> None:
    host = torch.randint(0, 1000, (SIZE,), dtype=torch.int64)
    for _ in range(WARMUP):
        _ = host.to(device, non_blocking=False)
        sync_device(kind)

    h2d = []
    for _ in range(ITERS):
        sync_device(kind)
        t0 = time.perf_counter()
        dev = host.to(device, non_blocking=False)
        sync_device(kind)
        h2d.append(time.perf_counter() - t0)
        del dev

    src = host.to(device)
    dst = torch.empty_like(src)
    sync_device(kind)
    for _ in range(WARMUP):
        dst.copy_(src)
        sync_device(kind)

    d2d = []
    for _ in range(ITERS):
        sync_device(kind)
        t0 = time.perf_counter()
        dst.copy_(src)
        sync_device(kind)
        d2d.append(time.perf_counter() - t0)

    print(
        f"torch H2D  {device}: {mb_per_s(mean_s(h2d)):.1f} MB/s  "
        f"({mean_s(h2d)*1e3:.2f} ms, {BYTES/1024/1024:.1f} MiB)"
    )
    print(
        f"torch D2D  {device}: {mb_per_s(mean_s(d2d)):.1f} MB/s  "
        f"({mean_s(d2d)*1e3:.2f} ms)"
    )


def _acl_check(err: int, what: str) -> None:
    if err != 0:
        raise RuntimeError(f"{what} failed: {err}")


def bench_acl(device_index: int) -> None:
    """Same copy kinds tensor-pool uses: aclrtMemcpy H2D and D2D."""
    acl = ctypes.CDLL("libascendcl.so")
    acl.aclInit.restype = ctypes.c_int
    acl.aclInit.argtypes = [ctypes.c_char_p]
    acl.aclrtSetDevice.restype = ctypes.c_int
    acl.aclrtSetDevice.argtypes = [ctypes.c_int32]
    acl.aclrtMalloc.restype = ctypes.c_int
    acl.aclrtMalloc.argtypes = [
        ctypes.POINTER(ctypes.c_void_p),
        ctypes.c_size_t,
        ctypes.c_int,
    ]
    acl.aclrtFree.restype = ctypes.c_int
    acl.aclrtFree.argtypes = [ctypes.c_void_p]
    acl.aclrtMallocHost.restype = ctypes.c_int
    acl.aclrtMallocHost.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
    acl.aclrtFreeHost.restype = ctypes.c_int
    acl.aclrtFreeHost.argtypes = [ctypes.c_void_p]
    acl.aclrtMemcpy.restype = ctypes.c_int
    acl.aclrtMemcpy.argtypes = [
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_int,
    ]
    acl.aclrtSynchronizeDevice.restype = ctypes.c_int
    acl.aclrtSynchronizeDevice.argtypes = []

    err = acl.aclInit(None)
    if err not in (0, 100002):
        raise RuntimeError(f"aclInit failed: {err}")
    _acl_check(acl.aclrtSetDevice(device_index), f"aclrtSetDevice({device_index})")

    pageable = np.random.randint(0, 1000, size=SIZE, dtype=np.int64)
    host = ctypes.c_void_p()
    _acl_check(acl.aclrtMallocHost(ctypes.byref(host), BYTES), "aclrtMallocHost")
    ctypes.memmove(host.value, pageable.ctypes.data, BYTES)

    d0 = ctypes.c_void_p()
    d1 = ctypes.c_void_p()
    _acl_check(acl.aclrtMalloc(ctypes.byref(d0), BYTES, 0), "aclrtMalloc d0")
    _acl_check(acl.aclrtMalloc(ctypes.byref(d1), BYTES, 0), "aclrtMalloc d1")

    try:
        for src, label, kind in (
            (pageable.ctypes.data, "ACL H2D pageable (numpy, like torch.tensor)", 1),
            (host.value, "ACL H2D pinned (aclrtMallocHost, like cpu2npu pool)", 1),
        ):
            for _ in range(WARMUP):
                _acl_check(
                    acl.aclrtMemcpy(d0, BYTES, src, BYTES, kind),
                    "warmup H2D",
                )
            samples = []
            for _ in range(ITERS):
                acl.aclrtSynchronizeDevice()
                t0 = time.perf_counter()
                _acl_check(
                    acl.aclrtMemcpy(d0, BYTES, src, BYTES, kind),
                    label,
                )
                samples.append(time.perf_counter() - t0)
            print(
                f"{label}: {mb_per_s(mean_s(samples)):.1f} MB/s  "
                f"({mean_s(samples)*1e3:.2f} ms)"
            )

        for _ in range(WARMUP):
            _acl_check(acl.aclrtMemcpy(d1, BYTES, d0, BYTES, 3), "warmup D2D")
        samples = []
        for _ in range(ITERS):
            acl.aclrtSynchronizeDevice()
            t0 = time.perf_counter()
            _acl_check(acl.aclrtMemcpy(d1, BYTES, d0, BYTES, 3), "ACL D2D")
            samples.append(time.perf_counter() - t0)
        print(
            f"ACL D2D same-device: {mb_per_s(mean_s(samples)):.1f} MB/s  "
            f"({mean_s(samples)*1e3:.2f} ms)"
        )

        from dora_tensor_pool.tensor_info import _wrap_npu_ptr

        src_host = torch.from_numpy(pageable)
        src_dev = src_host.to(f"npu:{device_index}")
        device = f"npu:{device_index}"
        dst_t = _wrap_npu_ptr(int(d1.value), [BYTES], torch.uint8, device, BYTES)
        src_bytes = _wrap_npu_ptr(int(src_dev.data_ptr()), [BYTES], torch.uint8, device, BYTES)
        for _ in range(WARMUP):
            dst_t.copy_(src_bytes)
            torch.npu.synchronize()
        samples = []
        for _ in range(ITERS):
            torch.npu.synchronize()
            t0 = time.perf_counter()
            dst_t.copy_(src_bytes)
            torch.npu.synchronize()
            samples.append(time.perf_counter() - t0)
        print(
            f"torch.copy_ into aclrtMalloc: {mb_per_s(mean_s(samples)):.1f} MB/s  "
            f"({mean_s(samples)*1e3:.2f} ms)"
        )
    finally:
        acl.aclrtFree(d0)
        acl.aclrtFree(d1)
        acl.aclrtFreeHost(host)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device",
        default="npu:0",
        help="npu:0 or cuda:0 (default npu:0)",
    )
    args = parser.parse_args()
    device = args.device
    print(f"payload {BYTES} bytes ({BYTES/1024/1024:.1f} MiB), warmup={WARMUP}, iters={ITERS}")

    if device.startswith("npu"):
        import torch_npu  # noqa: F401

        idx = int(device.split(":")[1]) if ":" in device else 0
        torch.npu.set_device(idx)
        bench_torch(device, "npu")
        bench_acl(idx)
    elif device.startswith("cuda"):
        idx = int(device.split(":")[1]) if ":" in device else 0
        torch.cuda.set_device(idx)
        bench_torch(device, "cuda")
    else:
        raise SystemExit(f"unsupported device {device}")


if __name__ == "__main__":
    main()
