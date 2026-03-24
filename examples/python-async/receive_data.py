import asyncio
import time

from dora import Node


async def main():
    node = Node()
    received_inputs = 0
    # Avoid hanging indefinitely in CI if the dataflow stalls.
    deadline = time.monotonic() + 30.0
    while received_inputs < 50 and time.monotonic() < deadline:
        event = await node.recv_async(timeout=1.0)
        if event is None:
            continue
        if event["type"] == "STOP":
            break
        if event["type"] == "INPUT":
            received_inputs += 1
        del event

    if received_inputs == 0:
        raise RuntimeError("python-async did not receive any INPUT events")
    print("done!")


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())
