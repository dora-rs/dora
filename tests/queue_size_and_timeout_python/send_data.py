"""TODO: Add docstring."""

import time

import pyarrow as pa
from dora import Node


def main() -> None:
    """TODO: Add docstring."""
    dora_node = Node()
    i = 0
    for event in dora_node:
        event_type = event["type"]
        if event_type == "STOP":
            break
        if event_type != "INPUT":
            continue

        dora_node.send_output("ts", pa.array([time.perf_counter_ns(), i]))
        i += 1


if __name__ == "__main__":
    main()
