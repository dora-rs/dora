"""Echo tool — built-in tool that echoes back messages."""

import json
import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            values = event["value"].to_pylist()
            for v in values:
                try:
                    request = json.loads(v)
                except json.JSONDecodeError:
                    continue

                if request.get("tool") == "echo":
                    response = json.dumps({
                        "tool": "echo",
                        "result": request.get("message", ""),
                    })
                    node.send_output("response", pa.array([response]))
                    logging.info("Echo: %s", request.get("message", ""))

        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
