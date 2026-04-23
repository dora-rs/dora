"""Sink node that prints string results."""

import logging

from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            label = event["value"].to_pylist()[0]
            print(f"[sink] {label}")
        elif event["type"] == "STOP":
            logging.info("Sink stopping")
            break


if __name__ == "__main__":
    main()
