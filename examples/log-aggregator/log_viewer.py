"""Log viewer node that receives ALL logs via adora/logs virtual input.

No manual wiring needed -- this node automatically receives structured
log messages from every node in the dataflow.
"""

import json

from dora import Node

node = Node()

print("=== Log Viewer Started (receiving all dataflow logs) ===")

for event in node:
    if event["type"] == "INPUT" and event["id"] == "logs":
        raw = bytes(event["value"]).decode("utf-8")
        try:
            log = json.loads(raw)
            level = log.get("level", "?").upper()
            node_id = log.get("node_id", "unknown")
            message = log.get("message", raw)
            print(f"[{level:6}][{node_id}] {message}")
        except json.JSONDecodeError:
            print(f"[RAW] {raw}")
