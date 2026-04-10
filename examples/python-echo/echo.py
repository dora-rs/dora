"""Echo node: forwards any incoming value and metadata unchanged."""

from dora import Node


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            node.send_output("data", event["value"], event["metadata"])
        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
