"""Log source node that emits structured log entries at varying levels."""

import json
import time

from dora import Node

node = Node()

levels = ["error", "warn", "info", "debug"]
for i in range(20):
    level = levels[i % len(levels)]
    entry = {
        "timestamp": "2025-01-01T00:00:00Z",
        "level": level,
        "node_id": "source",
        "message": f"log entry {i}",
    }
    print(json.dumps(entry), flush=True)
    time.sleep(0.2)
