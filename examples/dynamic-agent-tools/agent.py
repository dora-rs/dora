"""Agent node — sends tool requests and processes responses.

Simulates an AI agent that dispatches tasks to tools. The set of
available tools can grow dynamically via `adora node add`.
"""

import json
import logging
import random

import pyarrow as pa
from adora import Node

TASKS = [
    {"tool": "echo", "message": "Hello, world!"},
    {"tool": "echo", "message": "Testing dynamic tools"},
    {"tool": "calc", "expression": "2 + 3"},
    {"tool": "calc", "expression": "42 * 7"},
    {"tool": "echo", "message": "Agent is running"},
]


def main():
    node = Node()
    task_index = 0
    responses_received = 0

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "tick":
                # Send a tool request on each tick
                task = TASKS[task_index % len(TASKS)]
                request_json = json.dumps(task)
                node.send_output("tool-request", pa.array([request_json]))
                task_index += 1

                if task_index % 5 == 0:
                    logging.info(
                        "Agent: sent %d requests, received %d responses",
                        task_index,
                        responses_received,
                    )

            elif event["id"] == "tool-response":
                values = event["value"].to_pylist()
                for v in values:
                    logging.info("Agent received tool response: %s", v)
                responses_received += len(values)

        elif event["type"] == "STOP":
            break

    logging.info("Agent done: %d requests, %d responses", task_index, responses_received)


if __name__ == "__main__":
    main()
