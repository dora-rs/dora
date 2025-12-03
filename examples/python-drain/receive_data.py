import time

from dora import Node


def main():
    node = Node()
    time.sleep(1)
    drained_data = node.drain()
    print("drained: ", drained_data)
    for _ in range(100):
        try:
            _event = node.try_recv()
        except Exception as e:
            print("Error receiving event:", e)
    print("done!")


if __name__ == "__main__":
    main()
