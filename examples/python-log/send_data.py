"""Data sender example for dora-rs with logging.

This script demonstrates how to send data from a Python node to the
dora-rs dataflow. It is typically used in conjunction with a logger node
to verify message transmission.
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
