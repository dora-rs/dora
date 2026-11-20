"""Record the runtime configuration seen by each incarnation of the test node."""

import json
import os
from pathlib import Path

from dora import Node

node = Node()
pid = os.getpid()
params = {}
with Path(os.environ["PARAM_JOURNAL"]).open("a", buffering=1) as journal:
    print(json.dumps([pid, params]), file=journal)
    for event in node:
        kind = event["type"]
        if kind == "STOP":
            break
        if kind == "PARAM_UPDATE":
            params[event["id"]] = event["value"]
        elif kind == "PARAM_DELETED":
            params.pop(event["id"], None)
        elif kind == "INPUT" and Path(os.environ["CRASH_PID"]).exists():
            if Path(os.environ["CRASH_PID"]).read_text() == str(pid):
                os._exit(42)
            continue
        else:
            continue
        print(json.dumps([pid, params]), file=journal)
