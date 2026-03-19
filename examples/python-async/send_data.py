"""Async example: send timestamped data at 10ms intervals."""

import time

import numpy as np
import pyarrow as pa
from adora import Node

node = Node()

for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] == "INPUT":
        now = time.perf_counter_ns()
        node.send_output("data", pa.array([np.uint64(now)]))
