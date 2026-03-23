"""Receiver node for the 'latest data' queue size test in dora-rs.

This script verifies that the dora-rs runtime correctly implements
LIFO-like behavior for inputs with a queue size of 1. It sleeps
initially to allow a backlog to build, then asserts that it only receives
the most recent data packets, ensuring low latency.
"""

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
            duration < 2
        ), f"Duration: {duration} should be less than 1 as we should always pull latest data."
        time.sleep(1)
