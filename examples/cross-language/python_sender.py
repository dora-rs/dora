"""Python sender for cross-language test. Sends 10 i64 Arrow arrays to Rust receiver."""

import time

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for i in range(10):
        value = i * 10
        node.send_output("values", pa.array([value], type=pa.int64()))
        print(f"python-sender: sent {value}")
        time.sleep(0.05)

    print("python-sender: finished sending 10 messages")


if __name__ == "__main__":
    main()
