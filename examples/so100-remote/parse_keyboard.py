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
        x = 0.0 + (j * 0.005)
        y = -0.1 - (i * 0.001)
        if x == 0:
            theta = -1.8
        else:
            theta = np.arctan(y / x)

        print("Sending action: ", x, y, theta)
        node.send_output(
            "action",
            pa.array([0.0 + (j * 0.005), -0.1 - (i * 0.001), 0.18, -3.1, theta, 0.45]),
            metadata={"encoding": "xyzrpy"},
        )
