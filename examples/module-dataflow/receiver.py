"""Receiver node that logs incoming values."""

import logging

from adora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            values = event["value"].to_pylist()
            logging.info("Received [%s]: %s", event["id"], values)
        elif event["type"] == "STOP":
            logging.info("Receiver stopping")
            break

    logging.info("Receiver done")


if __name__ == "__main__":
    main()
