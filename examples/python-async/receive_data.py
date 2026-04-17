"""Async example: receive data using asyncio with run_in_executor."""

import asyncio

from dora import Node


async def main():
    node = Node()
    loop = asyncio.get_event_loop()
    while True:
        event = await loop.run_in_executor(None, node.next)
        if event is None or event["type"] == "STOP":
            break
        del event
    print("done!")


if __name__ == "__main__":
    asyncio.run(main())
