"""Temperature sensor node that logs readings at various levels."""

import logging
import random

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            temp = 22.0 + random.gauss(0, 3.0)

            logging.info("sensor reading: %.2f C", temp)

            if temp > 28.0:
                logging.warning("high temperature: %.2f C", temp)
            if temp < 15.0:
                logging.warning("low temperature: %.2f C", temp)

            node.send_output("reading", pa.array([temp]))

        elif event["type"] == "STOP":
            logging.info("sensor stopping")
            break


if __name__ == "__main__":
    main()
