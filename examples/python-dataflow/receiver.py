"""Simple receiver node that receives messages from sender and transformer."""

import logging
from datetime import datetime, timezone

from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            # Access timestamp from metadata (as Unix timestamp float)
            timestamp_float = event["metadata"]["timestamp"]
            timestamp = datetime.fromtimestamp(timestamp_float, tz=timezone.utc)

            if input_id == "message":
                # Simple integer array from sender
                message = event["value"].to_pylist()
                logging.info("Received message at %s: %s", timestamp, message)
            elif input_id == "transformed":
                # Struct array from transformer
                transformed = event["value"].to_pylist()
                logging.info("Received transformed at %s: %s", timestamp, transformed)
            else:
                logging.warning("Unknown input ID: %s", input_id)
        elif event["type"] == "STOP":
            logging.info("Receiver stopping after receiving event: %s", event)
            break

    logging.info("Receiver finished")


if __name__ == "__main__":
    main()
