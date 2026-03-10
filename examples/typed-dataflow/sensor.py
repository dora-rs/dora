"""Sensor node that emits float readings."""

import random
import time

import pyarrow as pa
from adora import Node


def main():
    node = Node()

    for i in range(50):
        value = 20.0 + random.gauss(0, 2.0)  # temperature-like reading
        node.send_output("reading", pa.array([value], type=pa.float64()))
        time.sleep(0.1)


if __name__ == "__main__":
    main()
