"""TODO: Add docstring."""

import time

import pyarrow as pa
from dora import Node

node = Node()

target_y = -0.02
target_x = 0.00

place_x = -0.02
place_y = -0.1

top_z = -0.50
low_z = -0.57

roll = 1.86
pitch = 1.43
yaw_closed = 0.8
yaw_opened = -0.5

now = time.time()
time.sleep(1.5)

node.send_output(
    "action",
    pa.array([target_x, target_y, top_z, roll, pitch, yaw_closed]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(0.8)

node.send_output(
    "action",
    pa.array([target_x, target_y, top_z, roll, pitch, yaw_closed]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(0.5)

node.send_output(
    "action",
    pa.array([target_x, target_y, low_z, roll, pitch, yaw_closed]),
    metadata={"encoding": "xyzrpy"},
)
time.sleep(0.2)


node.send_output(
    "action",
    pa.array([target_x, target_y, low_z, roll, pitch, yaw_opened]),
    metadata={"encoding": "xyzrpy"},
)


time.sleep(1.0)

node.send_output(
    "action",
    pa.array([target_x, target_y, top_z, roll, pitch, yaw_opened]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(0.3)


node.send_output(
    "action",
    pa.array([place_x, place_y, top_z, roll, pitch, yaw_opened]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(1.0)

node.send_output(
    "action",
    pa.array([place_x, place_y, low_z, roll, pitch, yaw_opened]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(0.2)

node.send_output(
    "action",
    pa.array([place_x, place_y, low_z, roll, pitch, yaw_closed]),
    metadata={"encoding": "xyzrpy"},
)
time.sleep(1.0)

node.send_output(
    "action",
    pa.array([place_x, place_y, top_z, roll, pitch, yaw_opened]),
    metadata={"encoding": "xyzrpy"},
)

time.sleep(1.0)

node.send_output(
    "action",
    pa.array([place_x, place_y, top_z, roll, pitch, yaw_opened]),
    metadata={"encoding": "xyzrpy"},
)
