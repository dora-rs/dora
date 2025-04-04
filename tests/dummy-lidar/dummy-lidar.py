#!/usr/bin/env python3
import time
import numpy as np
import pyarrow as pa
from dora import Node

def main():
    node = Node("dummy-lidar")
    while True:
        # Create a 10x10 grid in x and y over [-1, 1], z fixed to 0.
        xs, ys = np.meshgrid(np.linspace(-1, 1, 10), np.linspace(-1, 1, 10))
        zs = np.zeros_like(xs)
        # Stack and flatten the grid to a 1D array of floats
        points = np.stack([xs, ys, zs], axis=-1).reshape(-1).astype(np.float32)
        arr = pa.array(points)
        # Send the point cloud data on the "pointcloud" channel
        node.send_output("pointcloud", arr)
        time.sleep(1)

if __name__ == "__main__":
    main()
