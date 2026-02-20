"""TODO: Add docstring."""

import time

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

i = 0
for event in node:
    if event["type"] == "STOP":
        break
    if event["type"] != "INPUT":
        continue
    if i == 100:
        break
    i += 1
    now = time.perf_counter_ns()
    node.send_output("data", pa.array([np.uint64(now)]))
