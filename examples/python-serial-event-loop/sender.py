"""Dora node that publishes an incrementing counter for every timer tick."""

import pyarrow as pa


def run_sender(node):
    """Process Dora events from ``node`` until a STOP event is received."""
    counter = 0
    for event in node:
        if event["type"] == "STOP":
            break
        if event["type"] == "INPUT" and event["id"] == "tick":
            node.send_output("counter", pa.array([counter], type=pa.uint64()))
            counter += 1


def main():
    from dora import Node

    run_sender(Node())


if __name__ == "__main__":
    main()
