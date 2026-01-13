"""Transformer node that receives messages and outputs a struct with multiple fields."""

import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            # Get the incoming message number
            values = event["value"].to_pylist()
            number = values[0]

            # Create a struct array with multiple fields
            struct_array = pa.StructArray.from_arrays(
                [
                    pa.array([number * 2]),  # doubled value
                    pa.array([f"Message #{number}"]),  # string description
                    pa.array([number % 2 == 0]),  # is_even flag
                ],
                names=["doubled", "description", "is_even"],
            )

            node.send_output("transformed", struct_array)
            logging.info(
                "Transformed message %d -> struct with doubled=%d", number, number * 2
            )

        elif event["type"] == "STOP":
            logging.info("Transformer stopping")
            break

    logging.info("Transformer finished")


if __name__ == "__main__":
    main()
