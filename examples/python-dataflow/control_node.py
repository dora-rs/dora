#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
from dora import Node
import pyarrow as pa

node = Node()

event = node.next()
direction = {
    "linear": {
        "x": random.random() + 1,
    },
    "angular": {
        "z": (random.random() - 0.5) * 5,
    },
}
if event["type"] == "INPUT":
    print(
        f"""Node received:
    id: {event["id"]},
    value: {event["data"]},
    metadata: {event["metadata"]}"""
    )
    node.send_output(
        "direction",
        pa.array(
            [random.random() + 1, 0, 0, 0, 0, random.random() - 0.5],
            type=pa.uint8(),
        ),
    )
