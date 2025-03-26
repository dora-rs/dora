import asyncio

from dora import Node


async def main():
    node = Node()
    for _ in range(100):
        event = await node.recv_async()
        print(event)
        # del event
    print('done!')


if __name__ ==  '__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())
