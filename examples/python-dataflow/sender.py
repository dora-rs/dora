"""Simple sender node that sends 100 messages and then exits."""

import logging
import time

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for i in range(100):
        # Create a simple Apache Arrow array with the message number
        data = pa.array([i])
        node.send_output("message", data)
        logging.info("Sent message %d", i)

        # wait a bit before sending the next message
        time.sleep(0.1)

    logging.info("Sender finished - sent 100 messages")


if __name__ == "__main__":
    main()
