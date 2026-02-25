"""TODO: Add docstring."""

import time

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

for event in node:
    if event["type"] in {"STOP", "INPUT_CLOSED", "ALL_INPUTS_CLOSED"}:
        break
    if event["type"] != "INPUT":
        continue
    now = time.perf_counter_ns()
    node.send_output("data", pa.array([np.uint64(now)]))
