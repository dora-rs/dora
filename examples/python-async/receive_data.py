import asyncio
import signal
import sys

from dora import Node


def handle_sigterm(signum, frame):
    """Handle SIGTERM for graceful shutdown."""
    print("Received SIGTERM, shutting down gracefully...")
    sys.exit(0)


# Register signal handler
signal.signal(signal.SIGTERM, handle_sigterm)


async def main():
    node = Node()
    for _ in range(50):
        event = await node.recv_async()
        if event["type"] == "STOP":
            break
        del event
    print("done!")


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())
