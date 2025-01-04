import os
from dora import Node

RUNNER_CI = True if os.getenv("CI") == "true" else False


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            print(
                f"""Node received:
            id: {event["id"]},
            value: {event["value"]},
            metadata: {event["metadata"]}"""
            )


if __name__ == "__main__":
    main()
