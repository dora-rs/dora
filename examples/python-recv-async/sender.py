"""Send 10 messages using sync API for the async receiver to consume."""

import time

import pyarrow as pa
from adora import Node


def main():
    node = Node()

    for i in range(10):
        node.send_output("values", pa.array([i], type=pa.int64()))
        time.sleep(0.05)


if __name__ == "__main__":
    main()
