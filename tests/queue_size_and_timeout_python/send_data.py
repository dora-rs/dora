"""TODO: Add docstring."""

import time

import pyarrow as pa
from adora import Node

start = time.time()


def main() -> None:
    """TODO: Add docstring."""
    adora_node = Node()
    i = 0
    while time.time() - start < 10:
        adora_node.send_output("ts", pa.array([time.perf_counter_ns(), i]))
        i += 1
        # print(f"Sent {i} times", flush=True)
        time.sleep(0.001)
        if adora_node.next(timeout=0.001) is None:
            break


if __name__ == "__main__":
    main()
