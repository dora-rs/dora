"""Data sender example for dora-rs with draining behavior.

This script demonstrates how to send a sequence of data packets from a
Python node. It uses a loop to emit timestamps and then terminates,
allowing the dora-rs runtime to drain the remaining messages.
"""

import time

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

i = 0
for event in node:
    if i == 100:
        break
    else:
        i += 1
    now = time.perf_counter_ns()
    node.send_output("data", pa.array([np.uint64(now)]))
