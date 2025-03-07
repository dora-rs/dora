#!/usr/bin/env python

import random

import pyarrow as pa
from dora import Node

node = Node()

for _i in range(500):
    event = node.next()
    if event is None:
        break
    if event["type"] == "INPUT":
        event_id = event["id"]
        if event_id == "turtle_pose":
            pass
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
