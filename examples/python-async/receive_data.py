import asyncio
import logging

from dora import Node


async def main():
    node = Node()
    logging.error("starting receiver")
    for _ in range(50):
        logging.info("waiting for event")
        event = await node.recv_async()
        logging.info(f"received {event}")
        if event["type"] == "STOP":
            break
        del event
    print("done!")


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())
