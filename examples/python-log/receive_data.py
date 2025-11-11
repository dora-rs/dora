from dora import Node

import logging


def main():
    node = Node()
    log = logging.getLogger(__name__)
    for event in node:
        if event["type"] == "INPUT":
            logging.info(f"info {event['value'].to_numpy()}")
            log.log(logging.DEBUG, f"received {event['id']} with data")
            print("123")
        if event["type"] == "ERROR":
            log.log(logging.ERROR, f"received {event} with data")

        # del event
    log.log(logging.INFO, "done!")


if __name__ == "__main__":
    main()
