"""TODO: Add docstring."""

import argparse
import ast
import os

import pyarrow as pa
from dora import Node


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same values as the ENV variables.
    """TODO: Add docstring."""
    parser = argparse.ArgumentParser(description="Simple arrow sender")

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="arrow-assert",
    )
    parser.add_argument(
        "--data",
        type=str,
        required=False,
        help="Arrow Data as string.",
        default="",
    )

    args = parser.parse_args()

    data = os.getenv("DATA", args.data)

    node = Node(
        args.name,
    )  # provide the name to connect to the dataflow if dynamic node

    try:
        data = ast.literal_eval(data)
    except ValueError:
        print("Passing input as string")

    if isinstance(data, (str, int, float)):
        data = pa.array([data])
    else:
        data = pa.array(data)  # initialize pyarrow array

    for event in node:
        if event["type"] == "INPUT":
            value = event["value"]
            assert value == data, f"Expected {data}, got {value}"


if __name__ == "__main__":
    main()
