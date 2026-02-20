import asyncio

from dora import Node


async def main():
    node = Node()
    while True:
        event = await node.recv_async()
        if event["type"] == "STOP":
            break
        if event["type"] != "INPUT":
            continue
    print("done!")


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())
