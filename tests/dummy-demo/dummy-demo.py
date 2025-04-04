#!/usr/bin/env python3
import time
import numpy as np
import pyarrow as pa
from dora import Node

def send_arm_data(node: Node) -> None:
    # Simulate 5 joints (15 floats: x, y, z for each joint)
    joints = np.array([
        0.0, 0.0, 0.0,
        0.1, 0.0, 0.0,
        0.2, 0.1, 0.0,
        0.3, 0.1, 0.0,
        0.4, 0.2, 0.0
    ], dtype=np.float32)
    # Add a small time-varying component
    joints += np.sin(time.time()) * 0.01
    arr = pa.array(joints)
    node.send_output("arm", arr)

def send_pointcloud_data(node: Node) -> None:
    # Create a 10x10 grid over [-1,1] in x and y; z=0
    xs, ys = np.meshgrid(np.linspace(-1, 1, 10), np.linspace(-1, 1, 10))
    zs = np.zeros_like(xs)
    # Flatten to a 1D array of floats
    points = np.stack([xs, ys, zs], axis=-1).reshape(-1).astype(np.float32)
    arr = pa.array(points)
    node.send_output("pointcloud", arr)

def send_trajectory_data(node: Node) -> None:
    # Create 10 waypoints linearly spaced between (0,0,0) and (1,1,0)
    waypoints = np.linspace([0, 0, 0], [1, 1, 0], 10, dtype=np.float32)
    arr = pa.array(waypoints.reshape(-1))
    node.send_output("trajectory", arr)

def send_textlog_data(node: Node, counter: int) -> None:
    msg = f"Demo log message {counter}"
    arr = pa.array([msg])
    node.send_output("textlog", arr)

def send_image_data(node: Node) -> None:
    # Create a dummy 100x100 grayscale image (each pixel is 1 byte)
    width, height, depth = 100, 100, 1
    # Create an array filled with 128 (mid-gray)
    pixels = np.full((height * width * depth,), 128, dtype=np.uint8)
    arr = pa.array(pixels)
    node.send_output("image", arr)

def main():
    node = Node("dummy-demo")
    counter = 0
    while True:
        send_arm_data(node)
        send_pointcloud_data(node)
        send_trajectory_data(node)
        send_textlog_data(node, counter)
        # Optionally, you can send image data as well:
        # send_image_data(node)
        counter += 1
        time.sleep(1)

if __name__ == "__main__":
    main()
