"""TODO: Add docstring."""

import time

import pyarrow as pa
from dora import Node


def main() -> None:
    """TODO: Add docstring."""
    dora_node = Node()
    start = time.time()
    i = 0
    while time.time() - start < 10:
        dora_node.send_output("ts", pa.array([time.perf_counter_ns(), i]))
        i += 1

        event = dora_node.next(timeout=0.001)
        if event is not None and event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
