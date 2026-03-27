"""Sender node for the queue size and timeout test in dora-rs.

This script emits a stream of timestamps and sequence numbers to the "ts"
output for a fixed duration (10 seconds). It is used to test the
interplay between queue sizes and timeouts in the dora-rs messaging system.
"""

import time

import pyarrow as pa
from dora import Node


def main() -> None:
    """Run the message sending loop for the duration of the test."""
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
