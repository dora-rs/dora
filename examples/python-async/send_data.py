"""Asynchronous data sender example for dora-rs.

This script demonstrates how to send data from a Python node to a dora-rs
dataflow. It sends a sequence of timestamps as 64-bit unsigned integers
to an output named "data".
"""

import time

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
    node.send_output("data", pa.array([now], type=pa.uint64()))
