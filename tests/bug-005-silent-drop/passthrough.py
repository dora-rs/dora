#!/usr/bin/env python3
"""透传 tick → data_out"""
import pyarrow as pa
from dora import Node

node = Node()
for event in node:
    if event["type"] == "INPUT" and event["id"] == "tick_in":
        tick_val = event["value"].to_pylist()[0]
        print(f"passthrough: received tick={tick_val}, forwarding...")
        node.send_output("data_out", event["value"])
    elif event["type"] == "STOP":
        break
