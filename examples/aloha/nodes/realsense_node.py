"""Module for handling Intel RealSense camera data.

This module provides functionality for capturing and processing data from
Intel RealSense cameras, including depth and color images.
"""

import os

import cv2
import numpy as np
import pyarrow as pa
import pyrealsense2 as rs
from dora import Node

IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", "640"))
IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", "480"))
FPS = 30
CAMERA_ID = os.getenv("CAMERA_ID")
print("camera ID:", CAMERA_ID)
pipe = rs.pipeline()
config = rs.config()
config.enable_device(CAMERA_ID)
config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.bgr8, FPS)
profile = pipe.start(config)

node = Node()

for event in node:
    frames = pipe.wait_for_frames()
    color_frame = frames.get_color_frame()
    color_images = np.asanyarray(color_frame.get_data())
    node.send_output("image", pa.array(color_images.ravel()))
    cv2.imshow(CAMERA_ID, color_images)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
