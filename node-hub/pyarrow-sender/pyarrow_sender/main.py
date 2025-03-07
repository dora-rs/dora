import argparse
import ast
import contextlib
import os

import pyarrow as pa
from dora import Node

RUNNER_CI = os.getenv("CI") == "true"


def main() -> None:
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
        args.name,
    )  # provide the name to connect to the dataflow if dynamic node

    if data is None:
        msg = "No data provided. Please specify `DATA` environment argument or as `--data` argument"
        raise ValueError(
            msg,
        )
    with contextlib.suppress(ValueError):
        data = ast.literal_eval(data)
    if isinstance(data, list):
        data = pa.array(data)  # initialize pyarrow array
    elif isinstance(data, (str, int, float)):
        data = pa.array([data])
    else:
        data = pa.array(data)  # initialize pyarrow array
    node.send_output("data", data)


if __name__ == "__main__":
    main()
