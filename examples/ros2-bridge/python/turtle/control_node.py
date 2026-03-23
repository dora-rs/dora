#!/usr/bin/env python
"""Control node for the ROS2 turtlesim example in dora-rs.

This script demonstrates how to control a ROS2 turtle by sending random
movement commands ("direction") and logging the turtle's pose based on
incoming feedback from the ROS2 bridge.
"""


import random

import pyarrow as pa
from dora import Node

node = Node()

for i in range(500):
    event = node.next()
    if event is None:
        break
    if event["type"] == "INPUT":
        event_id = event["id"]
        if event_id == "turtle_pose":
            print(
                f"""Pose: {event["value"].tolist()}""".replace("\r", "").replace(
                    "\n", " ",
                ),
            )
        elif event_id == "tick":
            direction = {
                "linear": {
                    "x": 1.0 + random.random(),
                },
                "angular": {"z": (random.random() - 0.5) * 5},
            }

            node.send_output(
                "direction",
                pa.array([direction]),
            )
