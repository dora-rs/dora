"""Webcam node: captures frames from a USB camera and sends them as PyArrow arrays.

Supports hot-plug recovery: if the camera fails for more than 10 consecutive
frames the node releases and re-opens the device automatically.
"""

import os

import cv2
import pyarrow as pa
from dora import Node

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", 0))

font = cv2.FONT_HERSHEY_SIMPLEX


def main():
    node = Node()
    video_capture = cv2.VideoCapture(CAMERA_INDEX)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    failure_count = 0

    for event in node:
        if event["type"] == "INPUT":
            ret, frame = video_capture.read()
            if ret:
                frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
                failure_count = 0
            elif failure_count > 10:
                # Re-open the camera to recover from a disconnection (hot-plug support)
                video_capture.release()
                video_capture = cv2.VideoCapture(CAMERA_INDEX)
                video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
                video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
                failure_count = 0
                continue
            else:
                failure_count += 1
                continue

            node.send_output(
                "image",
                pa.array(frame.ravel()),
                metadata={
                    "width": CAMERA_WIDTH,
                    "height": CAMERA_HEIGHT,
                    "encoding": "bgr8",
                },
            )

        elif event["type"] == "STOP":
            break

    video_capture.release()


if __name__ == "__main__":
    main()
