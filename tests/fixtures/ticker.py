"""Tick source that emits an incrementing counter forever."""

import pyarrow as pa
from dora import Node

node = Node()
counter = 0
for event in node:
    if event["type"] == "INPUT" and event["id"] == "tick":
        node.send_output("count", pa.array([counter], type=pa.uint64()))
        counter += 1
    elif event["type"] == "STOP":
        break
