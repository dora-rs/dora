import argparse
import os
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
        default="echo",
    )
    args = parser.parse_args()

    node = Node(
        args.name
    )  # provide the name to connect to the dataflow if dynamic node

    for event in node:
        if event["type"] == "INPUT":
            node.send_output(event["id"], event["value"], event["metadata"])


if __name__ == "__main__":
    main()
