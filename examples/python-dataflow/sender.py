"""Simple sender node that sends messages and exits when Dora sends STOP."""

import logging
import time

import pyarrow as pa
from dora import Node


def main():
    node = Node()
    sent = 0
    stopped = False

    while sent < 100:
        event = node.try_recv()
        if event is not None:
            if event["type"] == "STOP":
                stopped = True
                logging.info("Sender stopping after receiving STOP")
                break

            logging.warning("Ignoring unexpected event: %s", event)

        # Create a simple Apache Arrow array with the message number.
        data = pa.array([sent])
        node.send_output("message", data)
        logging.info("Sent message %d", sent)
        sent += 1

        # Pace the source while still polling for STOP on each iteration.
        time.sleep(0.1)

    if stopped:
        logging.info("Sender finished after STOP - sent %d messages", sent)
    else:
        logging.info("Sender finished - sent %d messages", sent)


if __name__ == "__main__":
    main()
