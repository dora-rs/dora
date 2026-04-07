import asyncio

from dora import Node


async def main():
    node = Node()
    for _ in range(50):
        event = await node.recv_async()
        if event is None or event["type"] == "STOP":
            break
        del event
    print("done!")


if __name__ == "__main__":
    asyncio.run(main())
