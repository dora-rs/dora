"""TODO: Add docstring."""

import time

from dora import Node

node = Node()

# Voluntarily sleep for 5 seconds to ensure that the node is dropping the oldest input
time.sleep(5)

for event in node:
    event_type = event["type"]

    if event_type == "INPUT":
        event_id = event["id"]
        send_time = event["value"][0].as_py()

        duration = (time.perf_counter_ns() - send_time) / 1_000_000_000
        print("Duration: ", duration)
        assert (
            duration < 10
        ), f"Duration: {duration:.2f}s exceeds 10s threshold (should pull latest data, not stale)"
        time.sleep(1)
