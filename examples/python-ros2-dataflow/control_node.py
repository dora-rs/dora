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
        match event["id"]:
            case "turtle_pose":
                print(
                    f"""Pose: {event["value"]}""".replace("\r", "").replace("\n", " ")
                )
            case "tick":
                direction = {
                    "linear": {
                        "x": 1.0,
                    },
                    "angular": {"z": 1.0},
                }

                node.send_output(
                    "direction",
                    pa.array([direction]),
                )
