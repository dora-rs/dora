"""TODO: Add docstring."""

import time

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()


now = time.time()

for j in range(0, 100):
    for i in range(110):
        time.sleep(0.033)
        y = 0.0 + (j * 0.1)
        x = -0.1 - (i * 0.01)

        node.send_output(
            "action",
            pa.array([0.0 + (j * 0.05), -0.1 - (i * 0.001), 0.1, -1.6, -0.9, -3.14]),
            metadata={"encoding": "xyzrpy"},
        )
