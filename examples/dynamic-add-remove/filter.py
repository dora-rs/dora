"""Filter node — dynamically added to running dataflow.

Passes through only even numbers. Odd numbers are dropped silently.
"""

import logging

import pyarrow as pa
from adora import Node


def main():
    node = Node()
    passed = 0
    dropped = 0

    logging.info("Filter node started (dynamically added)")

    for event in node:
        if event["type"] == "INPUT":
            values = event["value"].to_pylist()
            evens = [v for v in values if v % 2 == 0]
            if evens:
                node.send_output("output", pa.array(evens))
                passed += len(evens)
            else:
                dropped += len(values)

            if (passed + dropped) % 20 == 0:
                logging.info("Filter: %d passed, %d dropped", passed, dropped)
        elif event["type"] == "STOP":
            break

    logging.info("Filter done: %d passed, %d dropped", passed, dropped)


if __name__ == "__main__":
    main()
