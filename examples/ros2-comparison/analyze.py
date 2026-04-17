#!/usr/bin/env python3
"""Compare Dora and ROS2 benchmark CSV results side-by-side."""

import csv
import sys

# CSV value column indices (after row[4:] slicing):
# latency: avg, p50, p95, p99, p999, min, max
# throughput: msg_per_sec, elapsed_ns, 0, 0, 0, 0, 0
AVG = 0
P99 = 3
MSG_PER_SEC = 0


def load_csv(path):
    """Load benchmark CSV into dict keyed by (mode, size)."""
    results = {}
    with open(path) as f:
        for row in csv.reader(f):
            if len(row) < 7:
                continue
            mode = row[0]        # "latency" or "throughput"
            size = int(row[1])   # payload bytes
            label = row[2]       # human-readable size
            n = int(row[3])      # sample count
            results[(mode, size)] = {
                "label": label,
                "n": n,
                "values": [float(x) for x in row[4:]],
            }
    return results


def fmt_us(ns):
    return f"{float(ns) / 1000:.1f}us"


def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <dora.csv> <ros2.csv>")
        sys.exit(1)

    dora = load_csv(sys.argv[1])
    ros2 = load_csv(sys.argv[2])

    # Latency comparison
    print("Latency Comparison (avg / p99):")
    print(f"{'Size':>8}  {'Dora avg':>12}  {'ROS2 avg':>12}  {'Speedup':>8}  "
          f"{'Dora p99':>12}  {'ROS2 p99':>12}  {'Speedup':>8}")
    print("-" * 90)

    lat_keys = sorted(
        [k for k in dora if k[0] == "latency" and k in ros2],
        key=lambda k: k[1],
    )
    for key in lat_keys:
        a = dora[key]
        r = ros2[key]
        a_avg, a_p99 = a["values"][AVG], a["values"][P99]
        r_avg, r_p99 = r["values"][AVG], r["values"][P99]
        avg_speedup = r_avg / a_avg if a_avg > 0 else 0
        p99_speedup = r_p99 / a_p99 if a_p99 > 0 else 0
        print(
            f"{a['label']:>8}  {fmt_us(a_avg):>12}  {fmt_us(r_avg):>12}  {avg_speedup:>7.1f}x  "
            f"{fmt_us(a_p99):>12}  {fmt_us(r_p99):>12}  {p99_speedup:>7.1f}x"
        )

    # Throughput comparison
    print()
    print("Throughput Comparison (msg/s):")
    print(f"{'Size':>8}  {'Dora msg/s':>14}  {'ROS2 msg/s':>14}  {'Speedup':>8}")
    print("-" * 50)

    tp_keys = sorted(
        [k for k in dora if k[0] == "throughput" and k in ros2],
        key=lambda k: k[1],
    )
    for key in tp_keys:
        a = dora[key]
        r = ros2[key]
        a_mps = a["values"][MSG_PER_SEC]
        r_mps = r["values"][MSG_PER_SEC]
        speedup = a_mps / r_mps if r_mps > 0 else 0
        print(
            f"{a['label']:>8}  {a_mps:>14.0f}  {r_mps:>14.0f}  {speedup:>7.1f}x"
        )


if __name__ == "__main__":
    main()
