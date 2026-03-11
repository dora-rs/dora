"""Adora benchmark receiver for ROS2 comparison.

Same measurement logic as ros2_receiver.py — uses perf_counter_ns timestamps
embedded in the first 8 bytes of each payload for comparable latency measurement.
"""

import os
import time

from adora import Node


def percentile(sorted_vals, pct):
    if not sorted_vals:
        return 0
    idx = round((pct / 100.0) * (len(sorted_vals) - 1))
    return sorted_vals[min(idx, len(sorted_vals) - 1)]


def format_size(b):
    if b == 0:
        return "0B"
    if b < 1024:
        return f"{b}B"
    if b < 1024 * 1024:
        return f"{b // 1024}KB"
    return f"{b // (1024 * 1024)}MB"


def fmt_us(ns):
    return f"{ns / 1000:.1f}us"


node = Node()
csv_path = os.getenv("BENCH_CSV")
latency_mode = True
current_size = -1
n = 0
latencies_ns = []
start_ns = 0


def flush():
    global n, current_size
    if n == 0:
        return

    size_label = format_size(current_size)

    if latency_mode:
        sorted_lat = sorted(latencies_ns)
        avg = sum(sorted_lat) // len(sorted_lat)
        p50 = percentile(sorted_lat, 50)
        p95 = percentile(sorted_lat, 95)
        p99 = percentile(sorted_lat, 99)
        p999 = percentile(sorted_lat, 99.9)
        mn = sorted_lat[0]
        mx = sorted_lat[-1]

        print(
            f"  {size_label:>6}  avg={fmt_us(avg):>10}  p50={fmt_us(p50):>10}  "
            f"p95={fmt_us(p95):>10}  p99={fmt_us(p99):>10}  p99.9={fmt_us(p999):>10}  "
            f"min={fmt_us(mn):>10}  max={fmt_us(mx):>10}  (n={n})"
        )

        if csv_path:
            with open(csv_path, "a") as f:
                f.write(
                    f"latency,{current_size},{size_label},{n},"
                    f"{avg},{p50},{p95},{p99},{p999},{mn},{mx}\n"
                )
    else:
        elapsed_ns = time.perf_counter_ns() - start_ns
        elapsed_s = elapsed_ns / 1e9
        msg_per_sec = n / elapsed_s if elapsed_s > 0 else 0
        print(f"  {size_label:>6}  {msg_per_sec:.0f} msg/s  (n={n}, {elapsed_s:.3f}s)")

        if csv_path:
            with open(csv_path, "a") as f:
                f.write(
                    f"throughput,{current_size},{size_label},{n},"
                    f"{msg_per_sec:.0f},{elapsed_ns},0,0,0,0,0\n"
                )


print("Adora Latency:")

for event in node:
    if event["type"] == "INPUT":
        recv_ns = time.perf_counter_ns()
        event_id = event["id"]
        arrow_array = event["value"]
        data = bytes(arrow_array.buffers()[1])
        data_len = len(arrow_array)

        if data_len != current_size:
            if n > 0 and current_size != 1:
                flush()
            current_size = data_len
            n = 0
            latencies_ns = []
            start_ns = recv_ns

        if event_id == "latency" and latency_mode:
            pass
        elif event_id == "throughput" and latency_mode:
            latency_mode = False
            flush()
            print("\nAdora Throughput:")
            current_size = data_len
            n = 0
            latencies_ns = []
            start_ns = recv_ns
        elif event_id == "throughput":
            pass
        else:
            continue

        n += 1

        # Extract send timestamp from first 8 bytes
        if data_len >= 8:
            send_ns = int.from_bytes(data[:8], "little")
            latencies_ns.append(recv_ns - send_ns)
        else:
            latencies_ns.append(0)

    elif event["type"] == "STOP":
        flush()
        break
