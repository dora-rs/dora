"""Simple receiver node that receives messages from sender and transformer."""

import logging

from dora import Node


def main():
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            input_id = event["id"]
            if input_id == "message":
                # Simple integer array from sender
                message = event["value"].to_pylist()
                logging.info("Received message: %s", message)
            elif input_id == "transformed":
                # Struct array from transformer
                transformed = event["value"].to_pylist()
                logging.info("Received transformed: %s", transformed)
            else:
                logging.warning("Unknown input ID: %s", input_id)
        elif event["type"] == "STOP":
            logging.info("Receiver stopping after receiving event: %s", event)
            break

    logging.info("Receiver finished")


if __name__ == "__main__":
    main()
