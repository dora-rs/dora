"""Doubles each incoming integer value."""

import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            values = event["value"].to_pylist()
            doubled = [v * 2 for v in values]
            node.send_output("doubled", pa.array(doubled))
            logging.info("Doubled %s -> %s", values, doubled)
        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
