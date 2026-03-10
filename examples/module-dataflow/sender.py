"""Sender node that emits 20 messages then exits."""

import logging
import time

import pyarrow as pa
from adora import Node


def main():
    node = Node()

    for i in range(20):
        node.send_output("value", pa.array([i]))
        logging.info("Sent %d", i)
        time.sleep(0.05)

    logging.info("Sender done")


if __name__ == "__main__":
    main()
