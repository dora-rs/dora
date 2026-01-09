#!/usr/bin/env python3
"""
Benchmark comparing zero-copy send_output_raw vs regular send_output performance.

Sends benchmark data to receiver which measures and reports all results.
"""

import os
import time
import tracemalloc

import numpy as np
import pyarrow as pa
from dora import Node

rng = np.random.default_rng()


def benchmark_regular_send(node, output_id, data_size, iterations):
    """Benchmark regular send_output (with copy)"""
    for i in range(iterations):
        data = rng.integers(low=0, high=256, size=data_size, dtype=np.uint8)

        # Measure preparation time (data creation in Python)
        start_timestamp = time.perf_counter()
        regular_data = data.copy()
        arrow_array = pa.array(regular_data)
        # simulate the data generating process

        node.send_output(
            output_id,
            arrow_array,
            metadata={
                "start_timestamp": start_timestamp,
                "method": "regular",
                "size": data_size,
                "iteration": i,
            },
        )
        time.sleep(0.1)  # Give time for the warmup messages to be processed


def benchmark_zero_copy_send(node, output_id, data_size, iterations):
    """Benchmark zero-copy send_output_raw (no copy)"""
    for i in range(iterations):
        data = rng.integers(low=0, high=256, size=data_size, dtype=np.uint8)

        start_timestamp = time.perf_counter()
        with node.send_output_raw(
            output_id,
            data_size,
            metadata={
                "start_timestamp": start_timestamp,
                "method": "zero_copy",
                "size": data_size,
                "iteration": i,
            },
        ) as sample:
            arr = sample.as_array()
            # simulate the process of fulfilling the promise
            arr[:] = data
            if i == 0:
                print(f"  Got numpy array, shape: {arr.shape}, dtype: {arr.dtype}")

        time.sleep(0.1)  # Give time for the warmup messages to be processed


def send_memory_usage(node, output_id, data_size):
    """Send memory usage information to receiver"""

    data = np.random.randint(0, 256, size=data_size, dtype=np.uint8)

    # Regular send memory usage
    tracemalloc.start()
    tracemalloc.reset_peak()
    regular_data = data.copy()
    arrow_array = pa.array(regular_data)
    node.send_output(output_id, arrow_array, metadata={"skip": True})
    _, regular_peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    # Zero-copy send memory usage
    tracemalloc.start()
    tracemalloc.reset_peak()
    with node.send_output_raw(output_id, data_size, metadata={"skip": True}) as sample:
        arr = sample.as_array()
        arr[:] = data
    _, zero_copy_peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    # Send memory results as metadata
    node.send_output(
        output_id,
        pa.array([]),
        metadata={
            "memory_results": True,
            "size": data_size,
            "regular_peak_bytes": regular_peak,
            "zero_copy_peak_bytes": zero_copy_peak,
        },
    )


def format_size(bytes_size):
    """Format bytes to human-readable size"""
    for unit in ["B", "KB", "MB", "GB"]:
        if bytes_size <= 1024.0:
            return f"{bytes_size:.2f} {unit}"
        bytes_size /= 1024.0
    return f"{bytes_size:.2f} TB"


def main():
    node = Node()

    # Test different data sizes
    data_sizes = [
        # 512,  # 0.5 KB (below threshold, uses Vec)
        1 * 1024,  # 1 KB (below threshold, uses Vec)
        4 * 1024,  # 4 KB (threshold for shared memory)
        8 * 1024,  # 8 KB
        64 * 1024,  # 64 KB
        1 * 1024 * 1024,  # 1 MB
        10 * 1024 * 1024,  # 10 MB
        # 100 * 1024 * 1024,  # 100 MB
    ]

    data_resp = data_sizes

    iterations = int(
        os.environ.get("ITERATIONS", 100)
    )  # Number of iterations for each benchmark

    print("=" * 80)
    print("DORA Python API Performance Benchmark")
    print("Comparing send_output (regular) vs send_output_raw (zero-copy)")
    print("=" * 80)
    print(f"\nIterations per test: {iterations}")
    print("Warming up...")

    # Warmup
    for _ in range(5):
        node.send_output("benchmark_output", pa.array(np.zeros(1024, dtype=np.uint8)))
        with node.send_output_raw("benchmark_output", 1024) as _sample:
            pass
        time.sleep(0.1)  # Give time for the warmup messages to be processed

    # Run all benchmarks
    for data_size in data_sizes:
        print(f"\nBenchmarking {format_size(data_size)}...")

        print("  Running regular send_output benchmark...")
        benchmark_regular_send(node, "benchmark_output", data_size, iterations)

        print("  Running zero-copy send_output_raw benchmark...")
        benchmark_zero_copy_send(node, "benchmark_output", data_size, iterations)

        print("  Sending memory usage information...")
        send_memory_usage(node, "benchmark_output", data_size)

        for event in node:
            if event["type"] == "INPUT":
                if event["id"] == "receiver_response":
                    metadata = event.get("metadata", {})
                    if metadata.get("size"):
                        size = metadata.get("size")
                        data_resp.remove(size)
                        print(f"Remaining data sizes: {data_resp}")
                        break
                    continue

            if event["type"] == "STOP":
                print("Received stop signal")
                break

    print(f"\n{'=' * 80}")
    print("Benchmark Complete! Waiting for receiver to finish...")
    print(f"{'=' * 80}\n")


if __name__ == "__main__":
    main()
