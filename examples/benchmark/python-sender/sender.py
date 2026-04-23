"""Python benchmark sender mirroring the Rust node logic.

Sends latency and throughput messages across 10 payload sizes (0B to ~4MB).
Used for Rust vs Python performance comparison.
"""

import time

import numpy as np
import pyarrow as pa
from dora import Node

LATENCY_SAMPLES = 100
THROUGHPUT_MESSAGES = 100
LATENCY_SLEEP_S = 0.005  # 5ms

SIZES = [0, 8, 64, 512, 2048, 4096, 4 * 4096, 10 * 4096, 100 * 4096, 1000 * 4096]

rng = np.random.default_rng(42)

node = Node()

# Pre-generate payloads
payloads = []
for size in SIZES:
    if size == 0:
        payloads.append(pa.array([], type=pa.uint8()))
    else:
        payloads.append(pa.array(rng.integers(0, 256, size=size, dtype=np.uint8)))

# Latency test: LATENCY_SAMPLES per size bracket
for payload in payloads:
    for _ in range(LATENCY_SAMPLES):
        node.send_output("latency", payload)
        time.sleep(LATENCY_SLEEP_S)

# Wait for latency messages to drain
time.sleep(2)

# Throughput test: burst of THROUGHPUT_MESSAGES per size bracket
for payload in payloads:
    for _ in range(THROUGHPUT_MESSAGES):
        node.send_output("throughput", payload)

    # Sentinel: single-byte marker signals end of this size bracket
    node.send_output("throughput", pa.array([1], type=pa.uint8()))
    time.sleep(2)
