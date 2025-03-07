import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            print(
                f"""Node received:
            id: {event["id"]},
            value: {event["value"]},
            metadata: {event["metadata"]}""",
            )
            node.send_output("speech", pa.array(["Hello World"]))


if __name__ == "__main__":
    main()
