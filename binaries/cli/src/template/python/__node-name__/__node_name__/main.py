"""Main entry point for a dora-rs Python node.

This script demonstrates a basic dora-rs node implementation that listens
for events and optionally sends outputs back to the dataflow.
"""

import pyarrow as pa
from dora import Node


def main():
    """Initialize and run the dora-rs node event loop.

    The node connects to the dora-rs runtime, iterates over incoming events,
    and handles inputs like "TICK" or custom input IDs.
    """
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "TICK":
                print(
                    f"""Node received:
                id: {event["id"]},
                value: {event["value"]},
                metadata: {event["metadata"]}""",
                )

            elif event["id"] == "my_input_id":
                # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                node.send_output(
                    output_id="my_output_id", data=pa.array([1, 2, 3]), metadata={},
                )


if __name__ == "__main__":
    main()
