import os
import argparse
import cv2

import numpy as np
import pyarrow as pa

from dataclasses import dataclass
from dora import Node


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the width and height of the image.
    parser = argparse.ArgumentParser(
        description="OpenCV Video Capture: This node is used to capture video from a camera.")

    parser.add_argument("--name", type=str, required=False, help="The name of the node in the dataflow.",
                        default="opencv-video-capture")
    parser.add_argument("--camera-id", type=int, required=False, help="The index of the camera.", default=0)
    parser.add_argument("--camera-width", type=int, required=False, help="The width of the camera.", default=640)
    parser.add_argument("--camera-height", type=int, required=False, help="The height of the camera.", default=480)

    args = parser.parse_args()

    camera_id = os.getenv("CAMERA_ID", args.camera_id)

    if isinstance(camera_id, str) and camera_id.isnumeric():
        camera_id = int(camera_id)

    camera_width = os.getenv("CAMERA_WIDTH", args.camera_width)
    camera_height = os.getenv("CAMERA_HEIGHT", args.camera_height)

    if camera_width is not None:
        if isinstance(camera_width, str) and camera_width.isnumeric():
            camera_width = int(camera_width)

    if camera_height is not None:
        if isinstance(camera_height, str) and camera_height.isnumeric():
            camera_height = int(camera_height)

    video_capture = cv2.VideoCapture(camera_id)
    node = Node(args.name)

    pa.array([])  # initialize pyarrow array

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                ret, frame = video_capture.read()

                # resize the frame
                frame = cv2.resize(frame, (camera_width, camera_height))

                if not ret:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        frame,
                        "No Webcam was found at index %d" % camera_id,
                        (int(30), int(30)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.75,
                        (255, 255, 255),
                        2,
                        1,
                    )

                node.send_output(
                    "image",
                    pa.array(frame.ravel()),
                    event["metadata"],
                )

        elif event_type == "STOP":
            break
        elif event_type == "ERROR":
            raise Exception(event["error"])


if __name__ == "__main__":
    main()
