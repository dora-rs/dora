import argparse
import ast
import contextlib
import os
import time

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
        default="terminal-input",
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
    last_err = ""
    while True:
        try:
            node = Node(
                args.name,
            )  # provide the name to connect to the dataflow if dynamic node
        except RuntimeError as err:
            if err != last_err:
                last_err = err
            time.sleep(1)

        if data is None and os.getenv("DORA_NODE_CONFIG") is None:
            while True:
                data = input(
                    "Provide the data you want to send:  ",
                )
                try:
                    data = ast.literal_eval(data)
                except ValueError:
                    pass
                except SyntaxError:
                    pass
                if isinstance(data, list):
                    data = pa.array(data)  # initialize pyarrow array
                elif isinstance(data, (str, int, float, dict)):
                    data = pa.array([data])
                else:
                    data = pa.array(data)  # initialize pyarrow array
                node.send_output("data", data)
                while True:
                    event = node.next(timeout=0.2)
                    if event is not None and event["type"] == "INPUT":
                        pass
                    else:
                        break
        else:
            with contextlib.suppress(ValueError):
                data = ast.literal_eval(data)
            if isinstance(data, list):
                data = pa.array(data)  # initialize pyarrow array
            elif isinstance(data, (str, int, float, dict)):
                data = pa.array([data])
            else:
                data = pa.array(data)  # initialize pyarrow array
            node.send_output("data", data)


if __name__ == "__main__":
    main()
