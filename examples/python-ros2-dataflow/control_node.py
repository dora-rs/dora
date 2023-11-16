#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
from dora import Node
import pyarrow as pa

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
                    "\n", " "
                )
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
