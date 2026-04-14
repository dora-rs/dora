"""Sink that consumes counter values until the dataflow stops."""

from dora import Node

node = Node()
for event in node:
    if event["type"] == "STOP":
        break
    # Drain inputs silently. The sink exists to give topics a subscriber.
