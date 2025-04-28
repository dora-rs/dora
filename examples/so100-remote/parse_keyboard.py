"""TODO: Add docstring."""

import time

import pyarrow as pa
from dora import Node

node = Node()

target_x = -0.08
target_y = -0.20

place_x = -0.08
place_y = 0.20

now = time.time()
time.sleep(1.5)

node.send_output(
    "action",
    pa.array([target_y, target_x, 0.15, -1.6, -0.0, -1]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(0.8)

node.send_output(
    "action",
    pa.array([target_y, target_x, 0.15, -1.6, -0.0, -1]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(0.5)

node.send_output(
    "action",
    pa.array([target_y, target_x, 0.09, -1.6, -0.0, -1]),
    metadata={"encoding": "xyzrpy"},
)
time.sleep(0.2)


node.send_output(
    "action",
    pa.array([target_y, target_x, 0.09, -1.6, -0.0, -3]),
    metadata={"encoding": "xyzrpy"},
)


time.sleep(1.0)

node.send_output(
    "action",
    pa.array([target_y, target_x, 0.15, -1.6, -0.0, -3]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(0.3)


node.send_output(
    "action",
    pa.array([place_y, place_x, 0.15, -1.6, -0.0, -3]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(1.0)

node.send_output(
    "action",
    pa.array([place_y, place_x, 0.10, -1.6, -0.0, -3]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(0.2)

node.send_output(
    "action",
    pa.array([place_y, place_x, 0.10, -1.6, -0.0, -1]),
    metadata={"encoding": "xyzrpy"},
)
time.sleep(1.0)

node.send_output(
    "action",
    pa.array([place_y, place_x, 0.15, -1.6, -0.0, -3]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(1.0)

node.send_output(
    "action",
    pa.array([place_y, place_x, 0.15, -1.6, -0.0, -3]),
    metadata={"encoding": "xyzrpy"},
)
