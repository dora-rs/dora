from dora import Node


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            message = event["value"][0].as_py()
            print(f"""I heard {message} from {event["id"]}""")


if __name__ == "__main__":
    main()
