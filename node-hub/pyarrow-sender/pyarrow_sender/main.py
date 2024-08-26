import argparse
import os
import ast

import pyarrow as pa

from dora import Node

RUNNER_CI = True if os.getenv("CI") == "true" else False


def main():

    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same values as the ENV variables.
    parser = argparse.ArgumentParser(description="Simple arrow sender")

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="pyarrow-sender",
    )
    parser.add_argument(
        "--data",
        type=str,
        required=False,
        help="Arrow Data as string.",
        default=None,
    )

    args = parser.parse_args()

    data = os.getenv("DATA", args.data)

    node = Node(
        args.name
    )  # provide the name to connect to the dataflow if dynamic node

    if data is None:
        raise ValueError(
            "No data provided. Please specify `DATA` environment argument or as `--data` argument"
        )
    else:
        try:
            data = ast.literal_eval(data)
        except ValueError:
            print("Passing input as string")
        if isinstance(data, list):
            data = pa.array(data)  # initialize pyarrow array
        elif isinstance(data, str):
            data = pa.array([data])
        elif isinstance(data, int):
            data = pa.array([data])
        elif isinstance(data, float):
            data = pa.array([data])
        else:
            data = pa.array(data)  # initialize pyarrow array
        node.send_output("data", data)


if __name__ == "__main__":
    main()
