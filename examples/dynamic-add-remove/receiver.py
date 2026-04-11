"""Receiver node — logs every value it receives."""

import logging

from dora import Node


def main():
    node = Node()
    count = 0

    for event in node:
        if event["type"] == "INPUT":
            values = event["value"].to_pylist()
            input_id = event["id"]
            count += 1
            if count % 10 == 0:
                logging.info("[%s] Received %d messages (latest: %s)", input_id, count, values)
        elif event["type"] == "STOP":
            break

    logging.info("Receiver done (received %d messages)", count)


if __name__ == "__main__":
    main()
