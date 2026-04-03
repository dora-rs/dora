"""Simple sender node that sends 100 messages and then exits."""

import logging

import pyarrow as pa
from dora import Node


def main():
    node = Node()

    for i in range(100):
        data = pa.array([i])
        node.send_output("message", data)
        logging.info("Sent message %d", i)

        # Use node.next() with timeout instead of time.sleep()
        # so the node can respond to stop events promptly
        event = node.next(timeout=0.1)
        if event is not None and event["type"] == "STOP":
            logging.info("Sender stopping early at message %d", i)
            break

    logging.info("Sender finished")


if __name__ == "__main__":
    main()
