"""Filters values, only passing through even numbers."""

import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            values = event["value"].to_pylist()
            evens = [v for v in values if v % 2 == 0]
            if evens:
                node.send_output("filtered", pa.array(evens))
                logging.info("Passed through even values: %s", evens)
            else:
                logging.info("Filtered out odd values: %s", values)
        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
