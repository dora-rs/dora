#!/usr/bin/env python3
"""
Benchmark receiver that measures end-to-end latency and prints all results.
"""

import os
import statistics
import time

import pyarrow as pa
from dora import Node


def format_size(bytes_size):
    """Format bytes to human-readable size"""
    for unit in ["B", "KB", "MB", "GB"]:
        if bytes_size < 1024.0:
            return f"{bytes_size:.2f} {unit}"
        bytes_size /= 1024.0
    return f"{bytes_size:.2f} TB"


def format_time(seconds):
    """Format time with appropriate unit"""
    if seconds < 1e-6:
        return f"{seconds * 1e9:.2f} ns"
    elif seconds < 1e-3:
        return f"{seconds * 1e6:.2f} µs"
    elif seconds < 1:
        return f"{seconds * 1e3:.2f} ms"
    else:
        return f"{seconds:.2f} s"


def print_results(data_size, regular_data, zero_copy_data, memory_results):
    """Print complete benchmark results"""
    print(f"\n{'=' * 80}")
    print(f"Benchmark Results for Data Size: {format_size(data_size)}")
    print(f"{'=' * 80}")

    # Calculate statistics
    def calc_stats(times):
        return {
            "mean": statistics.mean(times),
            "median": statistics.median(times),
            "min": min(times),
            "max": max(times),
            "stdev": statistics.stdev(times) if len(times) > 1 else 0,
        }

    regular_e2e_stats = calc_stats(regular_data["e2e_latencies"])
    zero_copy_e2e_stats = calc_stats(zero_copy_data["e2e_latencies"])

    # Print End-to-End Latency
    print("\n1. END-TO-END LATENCY (Send to Receive)")
    print(
        f"{'Metric':<20} {'Regular send_output':<25} {'Zero-copy send_output_raw':<25} {'Improvement':<15}"
    )
    print("-" * 85)

    e2e_improvement = (
        (regular_e2e_stats["mean"] - zero_copy_e2e_stats["mean"])
        / regular_e2e_stats["mean"]
        * 100
    )
    print(
        f"{'Mean:':<20} {format_time(regular_e2e_stats['mean']):<25} {format_time(zero_copy_e2e_stats['mean']):<25} {e2e_improvement:>6.2f}%"
    )
    print(
        f"{'Median:':<20} {format_time(regular_e2e_stats['median']):<25} {format_time(zero_copy_e2e_stats['median']):<25}"
    )
    print(
        f"{'Min:':<20} {format_time(regular_e2e_stats['min']):<25} {format_time(zero_copy_e2e_stats['min']):<25}"
    )
    print(
        f"{'Max:':<20} {format_time(regular_e2e_stats['max']):<25} {format_time(zero_copy_e2e_stats['max']):<25}"
    )
    print(
        f"{'Std Dev:':<20} {format_time(regular_e2e_stats['stdev']):<25} {format_time(zero_copy_e2e_stats['stdev']):<25}"
    )

    # Print Memory Usage
    print("\n3. PEAK MEMORY USAGE")
    print(
        f"{'Metric':<20} {'Regular send_output':<25} {'Zero-copy send_output_raw':<25} {'Reduction':<15}"
    )
    print("-" * 85)

    regular_mem = memory_results["regular_peak_bytes"]
    zero_copy_mem = memory_results["zero_copy_peak_bytes"]
    mem_reduction = (
        ((regular_mem - zero_copy_mem) / regular_mem * 100) if regular_mem > 0 else 0
    )

    print(
        f"{'Peak Memory:':<20} {format_size(regular_mem):<25} {format_size(zero_copy_mem):<25} {mem_reduction:>6.2f}%"
    )

    # Calculate throughput
    regular_throughput = data_size / regular_e2e_stats["mean"]
    zero_copy_throughput = data_size / zero_copy_e2e_stats["mean"]

    print("\n4. END-TO-END THROUGHPUT")
    print(f"{'Method':<20} {'Throughput':<25}")
    print("-" * 45)
    print(f"{'Regular send_output:':<20} {format_size(regular_throughput)}/s")
    print(f"{'Zero-copy:':<20} {format_size(zero_copy_throughput)}/s")
    print(f"{'Speedup:':<20} {zero_copy_throughput / regular_throughput:.2f}x")
    print(f"{'=' * 80}\n")


def main():
    node = Node()

    # Track data per (method, size)
    # Each bucket stores: e2e_latencies, prep_times
    data_buckets = {}
    memory_results = {}

    iterations = int(os.environ.get("ITERATIONS", 100))

    print(f"The iterations is {iterations}")
    print("Benchmark receiver started, waiting for data...")

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]

            if input_id == "benchmark_output":
                receive_time = time.perf_counter()

                # Extract metadata
                # Or skip messages used for memory measurement
                metadata = event.get("metadata", {})
                if metadata == {} or metadata.get("skip"):
                    continue

                # Check if this is memory results
                if metadata.get("memory_results"):
                    size = metadata.get("size")
                    memory_results[size] = {
                        "regular_peak_bytes": metadata.get("regular_peak_bytes"),
                        "zero_copy_peak_bytes": metadata.get("zero_copy_peak_bytes"),
                    }
                    print(f"Received memory results for size {size}")

                else:
                    send_timestamp = metadata.get("start_timestamp")
                    method = metadata.get("method")
                    size = metadata.get("size")
                    iteration = metadata.get("iteration")

                    if iteration is not None and iteration % 10 == 0:
                        print(f"received {method}: {iteration}")

                    if (
                        send_timestamp is not None
                        and method is not None
                        and size is not None
                    ):
                        # Calculate end-to-end latency
                        e2e_latency = receive_time - send_timestamp

                        # Store in the appropriate bucket if we have prep_time
                        bucket_key = (method, size)
                        if bucket_key not in data_buckets:
                            data_buckets[bucket_key] = {
                                "e2e_latencies": [],
                                "count": 0,
                            }

                        data_buckets[bucket_key]["e2e_latencies"].append(e2e_latency)
                        data_buckets[bucket_key]["count"] += 1

                # Check if we have complete data for this size, then print
                regular_key = ("regular", size)
                zero_copy_key = ("zero_copy", size)

                if (
                    regular_key in data_buckets
                    and zero_copy_key in data_buckets
                    and size in memory_results
                    and data_buckets[regular_key]["count"] == iterations
                    and data_buckets[zero_copy_key]["count"] == iterations
                ):
                    print_results(
                        size,
                        data_buckets[regular_key],
                        data_buckets[zero_copy_key],
                        memory_results[size],
                    )
                    print(f"Send response for size {size}")
                    node.send_output(
                        "benchmark_receiver_output",
                        pa.array([]),
                        metadata={"size": size},
                    )
                    del data_buckets[regular_key]
                    del data_buckets[zero_copy_key]
                    del memory_results[size]

        elif event["type"] == "STOP":
            print("Received stop signal.")
            break


if __name__ == "__main__":
    main()
