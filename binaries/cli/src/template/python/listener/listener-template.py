from dora import Node


def main() -> None:
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            event["value"][0].as_py()


if __name__ == "__main__":
    main()
