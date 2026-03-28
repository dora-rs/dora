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
    while True:
        event = await node.recv_async()

        # recv_async returns None when the event stream is closed
        # (e.g. all upstream senders have exited / AllInputsClosed)
        if event is None:
            break

        event_type = event.get("type")

        if event_type == "STOP":
            break

        if event_type in ("ERROR", "INPUT_CLOSED"):
            break

        if event_type == "INPUT":
            # Process inputs here if needed; this example just discards them.
            continue

    print("done!")


if __name__ == "__main__":
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(main())
