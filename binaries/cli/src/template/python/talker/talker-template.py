import pyarrow as pa
from dora import Node


def main() -> None:
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            node.send_output("speech", pa.array(["Hello World"]))


if __name__ == "__main__":
    main()
