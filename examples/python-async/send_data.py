"""TODO: Add docstring."""

import time
import logging

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

log = logging.getLogger(__name__)

i = 0
for event in node:
    if i == 100:
        break
    else:
        i += 1
    now = time.perf_counter_ns()
    logging.warning(f"sending {i}: {now}")
    node.send_output("data", pa.array([np.uint64(now)]))
