"""Receive events using recv_async() and send output from async context.

This exercises the PyO3 async path (not run_in_executor) to catch
GIL/async event loop deadlocks.
"""

import asyncio
import sys

import pyarrow as pa
from dora import Node


async def main():
    node = Node()
    received = 0

    while True:
        event = await node.recv_async(timeout=2.0)
        if event is None:
            break
        if event["type"] == "INPUT" and event["id"] == "values":
            values = event["value"].to_pylist()
            if len(values) != 1 or not isinstance(values[0], int):
                print(f"recv-async: ERROR unexpected value: {values}")
                sys.exit(1)
            # Send output from async context (tests GIL interaction)
            node.send_output("echo", pa.array(values, type=pa.int64()))
            received += 1
        elif event["type"] == "STOP":
            break

    if received < 5:
        print(f"recv-async: ERROR got only {received} messages (expected >= 5)")
        sys.exit(1)

    print(f"recv-async: SUCCESS - received and echoed {received} messages")


if __name__ == "__main__":
    asyncio.run(main())
