#!/usr/bin/env python3
import time
import pyarrow as pa
from dora import Node

def main():
    node = Node("dummy-textlog")
    counter = 0
    while True:
        msg = f"Test log message {counter}"
        # Create a PyArrow array containing the message
        arr = pa.array([msg])
        # Send the message on the "textlog" channel
        node.send_output("textlog", arr)
        counter += 1
        time.sleep(1)

if __name__ == "__main__":
    main()
