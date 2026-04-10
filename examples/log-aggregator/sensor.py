"""Sensor node that emits readings and structured logs."""

import random

from dora import Node

node = Node()

for event in node:
    if event["type"] == "INPUT" and event["id"] == "tick":
        reading = round(random.uniform(20.0, 30.0), 1)
        node.send_output("reading", str(reading).encode())

        if reading > 28.0:
            node.log_warn(f"High temperature: {reading}C")
        else:
            node.log_info(f"Temperature: {reading}C")
