"""Module to handle node input events and send speech output."""

import pyarrow as pa
from dora import Node


def main():
    """Process node input events and send speech output."""
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            print(
                f"""Node received:
            id: {event["id"]},
            value: {event["value"]},
            metadata: {event["metadata"]}""",
            )
            node.send_output("speech", pa.array(["Hello World"]))

if __name__ == "__main__":
    main()
