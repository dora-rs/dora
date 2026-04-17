"""Processor node that converts float readings to string labels."""

import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            value = event["value"].to_pylist()[0]
            if value > 22.0:
                label = f"HIGH: {value:.1f}"
            elif value < 18.0:
                label = f"LOW: {value:.1f}"
            else:
                label = f"OK: {value:.1f}"
            node.send_output("result", pa.array([label]))
            logging.info("Processed %.1f -> %s", value, label)
        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
