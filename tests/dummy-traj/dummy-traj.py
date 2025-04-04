#!/usr/bin/env python3
import time
import numpy as np
import pyarrow as pa
from dora import Node

def main():
    node = Node("dummy-traj")
    while True:
        # Generate 10 waypoints from (0,0,0) to (1,1,0)
        waypoints = np.linspace([0, 0, 0], [1, 1, 0], 10, dtype=np.float32)
        # Flatten the array into 1D
        waypoints_flat = waypoints.reshape(-1)
        arr = pa.array(waypoints_flat)
        # Send the trajectory data on the "trajectory" channel
        node.send_output("trajectory", arr)
        time.sleep(1)

if __name__ == "__main__":
    main()
