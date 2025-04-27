"""TODO: Add docstring."""

import time

import pyarrow as pa
from dora import Node

node = Node()


now = time.time()

for j in range(100):
    for i in range(110):
        time.sleep(0.033)

        print("Sending action")
        node.send_output(
            "action",
            pa.array([0.0 + (j * 0.005), -0.1 - (i * 0.001), 0.18, -3.1, -1.8, 0.45]),
            metadata={"encoding": "xyzrpy"},
        )
