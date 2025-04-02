"""TODO: Add docstring."""

import os

import cv2
import numpy as np
import pyrealsense2 as rs

CAMERA_ID = os.getenv("CAMERA_ID")
pipe = rs.pipeline()
config = rs.config()
config.enable_device(CAMERA_ID)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 60)
profile = pipe.start(config)


try:
    for i in range(1000):
        frames = pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_images = np.asanyarray(color_frame.get_data())
        color_images = cv2.cvtColor(color_images, cv2.COLOR_BGR2RGB)
        cv2.imshow("frame", color_images)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    pipe.stop()
