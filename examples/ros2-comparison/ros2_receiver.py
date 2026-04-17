"""ROS2 benchmark receiver.

Subscribes to bench_latency and bench_throughput topics, measures arrival latency
using time.perf_counter_ns(), and prints percentile results.

Requires: rclpy, std_msgs
"""

import csv
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray


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


class BenchReceiver(Node):
    def __init__(self):
        super().__init__("bench_receiver")
        self.latency = True
        self.current_size = -1
        self.n = 0
        self.latencies_ns = []
        self.start_ns = 0
        self.csv_path = os.getenv("BENCH_CSV")

        self.create_subscription(ByteMultiArray, "bench_latency", self.on_latency, 10)
        self.create_subscription(ByteMultiArray, "bench_throughput", self.on_throughput, 1000)

        print("ROS2 Latency:")

    def on_latency(self, msg):
        self._handle(msg, is_latency=True)

    def on_throughput(self, msg):
        if self.latency:
            self.latency = False
            self._flush()
            print("\nROS2 Throughput:")
        self._handle(msg, is_latency=False)

    def _handle(self, msg, is_latency):
        recv_ns = time.perf_counter_ns()
        data = bytes(msg.data)
        data_len = len(data)

        if data_len != self.current_size:
            if self.n > 0 and self.current_size != 1:
                self._flush()
            self.current_size = data_len
            self.n = 0
            self.latencies_ns = []
            self.start_ns = recv_ns

        self.n += 1

        # Extract send timestamp from first 8 bytes if available
        if data_len >= 8:
            send_ns = int.from_bytes(data[:8], "little")
            self.latencies_ns.append(recv_ns - send_ns)
        else:
            self.latencies_ns.append(0)

    def _flush(self):
        if self.n == 0:
            return

        size_label = format_size(self.current_size)

        if self.latency:
            sorted_lat = sorted(self.latencies_ns)
            avg = sum(sorted_lat) // len(sorted_lat)
            p50 = percentile(sorted_lat, 50)
            p95 = percentile(sorted_lat, 95)
            p99 = percentile(sorted_lat, 99)
            p999 = percentile(sorted_lat, 99.9)
            mn = sorted_lat[0]
            mx = sorted_lat[-1]

            def fmt_us(ns):
                return f"{ns / 1000:.1f}us"

            print(
                f"  {size_label:>6}  avg={fmt_us(avg):>10}  p50={fmt_us(p50):>10}  "
                f"p95={fmt_us(p95):>10}  p99={fmt_us(p99):>10}  p99.9={fmt_us(p999):>10}  "
                f"min={fmt_us(mn):>10}  max={fmt_us(mx):>10}  (n={self.n})"
            )

            if self.csv_path:
                self._write_csv(
                    f"latency,{self.current_size},{size_label},{self.n},"
                    f"{avg},{p50},{p95},{p99},{p999},{mn},{mx}"
                )
        else:
            elapsed_ns = time.perf_counter_ns() - self.start_ns
            elapsed_s = elapsed_ns / 1e9
            msg_per_sec = self.n / elapsed_s if elapsed_s > 0 else 0
            print(f"  {size_label:>6}  {msg_per_sec:.0f} msg/s  (n={self.n}, {elapsed_s:.3f}s)")

            if self.csv_path:
                self._write_csv(
                    f"throughput,{self.current_size},{size_label},{self.n},"
                    f"{msg_per_sec:.0f},{elapsed_ns},0,0,0,0,0"
                )

    def _write_csv(self, line):
        with open(self.csv_path, "a") as f:
            f.write(line + "\n")


def main():
    rclpy.init()
    node = BenchReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._flush()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
