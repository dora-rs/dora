"""Log viewer node that receives ALL logs via dora/logs virtual input.

No manual wiring needed -- this node automatically receives structured
log messages from every node in the dataflow.
"""

import json

from dora import Node

node = Node()

print("=== Log Viewer Started (receiving all dataflow logs) ===")

for event in node:
    if event["type"] == "INPUT" and event["id"] == "logs":
        value = event["value"]
        # `dora/logs` may deliver a StringScalar, an Array, or raw bytes
        # depending on the emitter; handle all three.
        if hasattr(value, "as_py"):
            raw = value.as_py()
            if isinstance(raw, list) and raw:
                raw = raw[0]
        elif isinstance(value, (bytes, bytearray)):
            raw = value.decode("utf-8")
        else:
            raw = str(value)
        try:
            log = json.loads(raw)
            level = log.get("level", "?").upper()
            node_id = log.get("node_id", "unknown")
            message = log.get("message", raw)
            print(f"[{level:6}][{node_id}] {message}")
        except json.JSONDecodeError:
            print(f"[RAW] {raw}")
