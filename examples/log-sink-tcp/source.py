"""Source node that generates logs at various levels for testing log sinks."""

import logging
import random

import pyarrow as pa
from dora import Node


def main():
    node = Node()
    count = 0

    for event in node:
        if event["type"] == "INPUT":
            count += 1
            value = random.uniform(20.0, 30.0)

            logging.info("reading #%d: %.2f", count, value)

            if value > 28.0:
                logging.warning("high value detected: %.2f", value)
            if value > 29.5:
                logging.error("critical threshold exceeded: %.2f", value)

            node.send_output("data", pa.array([value]))

        elif event["type"] == "STOP":
            logging.info("source stopping after %d readings", count)
            break


if __name__ == "__main__":
    main()
