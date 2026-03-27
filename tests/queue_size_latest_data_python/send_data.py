"""Sender node for the 'latest data' queue size test in dora-rs.

This script generates a high-frequency stream of timestamps to be
processed by a receiver node. Its purpose is to create a backlog of data
to test the runtime's ability to prioritize and pull only the latest
available message.
"""

import time

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

i = 0
for event in node:
    if i == 1000:
        break
    else:
        i += 1
    now = time.perf_counter_ns()
    node.send_output("data", pa.array([np.uint64(now)]))
