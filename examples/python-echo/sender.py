"""Sender node: emits a fixed PyArrow array every 500ms."""

import pyarrow as pa
from dora import Node


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            node.send_output("data", pa.array([1, 2, 3, 4, 5]), event["metadata"])
        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
