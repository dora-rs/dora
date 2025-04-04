#!/usr/bin/env python3
import time
import numpy as np
import pyarrow as pa
from dora import Node

def main():
    node = Node("dummy-arm")
    while True:
        # Define 5 joints, each with (x, y, z) coordinates (15 floats total)
        joints = np.array([
            0.0, 0.0, 0.0,
            0.1, 0.0, 0.0,
            0.2, 0.1, 0.0,
            0.3, 0.1, 0.0,
            0.4, 0.2, 0.0
        ], dtype=np.float32)
        # Add a slight time-dependent variation to simulate motion
        joints += np.sin(time.time()) * 0.01
        # Convert the joint data into a PyArrow array
        arr = pa.array(joints)
        # Use send_output to publish the data on the "arm" channel
        node.send_output("arm", arr)
        time.sleep(1)

if __name__ == "__main__":
    main()
