from dora import Node

import logging
import threading
import time

import numpy as np
import pyarrow as pa


def read_data_task(node, log):
    """Task that reads incoming events."""
    while (event := node.next()) is not None:
        if event["type"] == "INPUT":
            print(f"info {event['value'].to_numpy()}")
        del event
    log.log(logging.INFO, "read_data_task done!")


def publish_task(node, log):
    """Task that publishes to a topic."""
    while True:
        time.sleep(1)  # Publish every 1s
        now = time.perf_counter_ns()
        node.send_output("data", pa.array([np.uint64(now)]))


def main():
    node = Node()
    log = logging.getLogger(__name__)
    
    # Create thread for read task
    read_thread = threading.Thread(target=read_data_task, args=(node, log))
    read_thread.start()
    
    # Run publish task in a daemon thread (so it doesn't block main thread)
    publish_thread = threading.Thread(target=publish_task, args=(node, log), daemon=True)
    publish_thread.start()
    
    # Wait for read thread to complete
    read_thread.join()
    
    log.log(logging.INFO, "done!")


if __name__ == "__main__":
    main()