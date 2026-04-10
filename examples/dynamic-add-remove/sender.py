"""Sender node — emits incrementing integers."""

import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()
    counter = 0

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "tick":
                counter += 1
                node.send_output("value", pa.array([counter]))
                if counter % 10 == 0:
                    logging.info("Sent %d", counter)
        elif event["type"] == "STOP":
            break

    logging.info("Sender done (sent %d values)", counter)


if __name__ == "__main__":
    main()
