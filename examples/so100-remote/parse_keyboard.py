"""TODO: Add docstring."""

import time

import pyarrow as pa
from dora import Node

node = Node()

target_y = -0.02
target_x = 0.00

place_x = -0.02
place_y = 0.2
place_z = -0.48
top_z = -0.44
low_z = -0.57

roll = 1.86
pitch = 1.43
yaw_open = 0.8
yaw_close = -0.5


def grab(target_x, target_y, low_z, top_z, roll, pitch, yaw_open, yaw_close):
    node.send_output(
        "action",
        pa.array([target_x, target_y, top_z, roll, pitch, yaw_open]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(0.8)

    node.send_output(
        "action",
        pa.array([target_x, target_y, low_z, roll, pitch, yaw_open]),
        metadata={"encoding": "xyzrpy"},
    )
    time.sleep(0.2)


    node.send_output(
        "action",
        pa.array([target_x, target_y, low_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(1.0)

    node.send_output(
        "action",
        pa.array([target_x, target_y, top_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )


def place(place_x, place_y, place_z, top_z, roll, pitch, yaw_open, yaw_close):

    node.send_output(
        "action",
        pa.array([place_x, place_y, top_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(1.0)

    node.send_output(
        "action",
        pa.array([place_x, place_y, place_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(1.0)

    node.send_output(
        "action",
        pa.array([place_x, place_y, place_z, roll, pitch, yaw_open]),
        metadata={"encoding": "xyzrpy"},
    )
    time.sleep(0.5)

    node.send_output(
        "action",
        pa.array([place_x, place_y, place_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(0.5)

    node.send_output(
        "action",
        pa.array([place_x, place_y, top_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )

