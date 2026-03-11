"""Adora benchmark sender for ROS2 comparison.

Same workload as ros2_sender.py but using Adora's Python API.
Embeds perf_counter_ns in first 8 bytes for comparable latency measurement.
"""

import time

import pyarrow as pa
from adora import Node

LATENCY_SAMPLES = 100
THROUGHPUT_MESSAGES = 100
LATENCY_SLEEP_S = 0.005
SIZES = [0, 8, 64, 512, 2048, 4096, 4 * 4096, 10 * 4096, 100 * 4096, 1000 * 4096]

node = Node()


def stamp_payload(buf, size):
    """Write perf_counter_ns into the first 8 bytes of a pre-allocated buffer."""
    t = time.perf_counter_ns()
    t_bytes = t.to_bytes(8, "little")
    if size >= 8:
        buf[:8] = t_bytes
    elif size > 0:
        buf[:size] = t_bytes[:size]


# Latency test
for size in SIZES:
    buf = bytearray(size)
    for _ in range(LATENCY_SAMPLES):
        stamp_payload(buf, size)
        node.send_output("latency", pa.array(buf, type=pa.uint8()))
        time.sleep(LATENCY_SLEEP_S)

time.sleep(2)

# Throughput test
for size in SIZES:
    buf = bytearray(size)
    for _ in range(THROUGHPUT_MESSAGES):
        stamp_payload(buf, size)
        node.send_output("throughput", pa.array(buf, type=pa.uint8()))

    # Sentinel
    node.send_output("throughput", pa.array([1], type=pa.uint8()))
    time.sleep(2)
