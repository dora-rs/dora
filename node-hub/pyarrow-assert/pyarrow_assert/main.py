import argparse
import os
import ast


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
        args.name
    )  # provide the name to connect to the dataflow if dynamic node

    assert_data = ast.literal_eval(data)

    for event in node:
        if event["type"] == "INPUT":
            value = event["value"]
            assert value == assert_data, f"Expected {assert_data}, got {value}"


if __name__ == "__main__":
    main()
