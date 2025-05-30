"""TODO: Add docstring."""

import time
import numpy as np
import pyarrow as pa
from dora import Node

node = Node()
top_z = -0.43
low_z = -0.57

roll = 1.86
pitch = 1.43
yaw_open = 0.8
yaw_close = -0.5


def grab(target_x, target_y, low_z, top_z, roll, pitch, yaw_open, yaw_close, last_x, last_y):

    node.send_output(
        "action",
        pa.array([target_x, target_y, top_z, roll, pitch, yaw_open]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(0.6)

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

    time.sleep(0.4)

    node.send_output(
        "action",
        pa.array([target_x, target_y, top_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(0.5)

    node.send_output(
        "action",
        pa.array([0.05, 0.0, top_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )
        

def place(place_x, place_y, place_z, top_z, roll, pitch, yaw_open, yaw_close, last_x, last_y):


    node.send_output(
        "action",
        pa.array([place_x, place_y, top_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(0.6)

    node.send_output(
        "action",
        pa.array([place_x, place_y, place_z, roll, pitch, yaw_close]),
        metadata={"encoding": "xyzrpy"},
    )

    time.sleep(0.2)


    node.send_output(
        "action",
        pa.array([place_x, place_y, place_z, roll, pitch, yaw_open]),
        metadata={"encoding": "xyzrpy"},
    )    
    
    time.sleep(0.3)


    node.send_output(
        "action",
        pa.array([place_x, place_y, top_z, roll, pitch, yaw_open]),
        metadata={"encoding": "xyzrpy"},
    )    
    
    time.sleep(0.3)

    node.send_output(
        "action",
        pa.array([0.05, 0.0, top_z, roll, pitch, yaw_open]),
        metadata={"encoding": "xyzrpy"},
    )
        
time.sleep(0.6)


node.send_output(
    "action",
    pa.array([0.05, 0.0, top_z, roll, pitch, yaw_open]),
    metadata={"encoding": "xyzrpy"},
)

last_x = 0
last_y = 0
last_z = 0

for event in node:
    if event["type"] == "INPUT":
        if event["id"] == "pose":
            values = event["value"]
            values = values.to_numpy()
            print(values)
            if len(values) == 0:
                continue
            x = values[0]
            y = values[1]
            z = values[2]
            action = event["metadata"]["action"]
            

            # Adjust z with the size of the gripper
            z = z + 0.06
            match action:
                case "grab":
                    y = y + -0.01
                    grab(
                        x,
                        y,
                        z,
                        top_z,
                        roll,
                        pitch,
                        yaw_open,
                        yaw_close,
                        last_x,
                        last_y
                    )
                case "release":
                    place(
                        x,
                        y,
                        z,
                        top_z,
                        roll,
                        pitch,
                        yaw_open,
                        yaw_close,
                        last_x,
                        last_y
                    )
            last_x = -0.05
            last_y = 0.04
            last_z = z
