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
        print(
            f"""Node received:
    id: {event["id"]},
    value: {event["value"]},
    metadata: {event["metadata"]}"""
        )
        node.send_output(
            "direction",
            pa.array(
                [random.random() + 1, 0, 0, 0, 0, (random.random() - 0.5) * 5],
                type=pa.float32(),
            ),
        )
