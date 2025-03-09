import os
import argparse
import cv2

import numpy as np
import pyarrow as pa

from dora import Node


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same values as the ENV variables.
    parser = argparse.ArgumentParser(
        description="OpenCV Video Capture: This node is used to capture video from a camera."
    )

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="opencv-video-capture",
    )
    parser.add_argument(
        "--path",
        type=int,
        required=False,
        help="The path of the device to capture (e.g. /dev/video1, or an index like 0, 1...",
        default=0,
    )
    parser.add_argument(
        "--image-width",
        type=int,
        required=False,
        help="The width of the image output. Default is the camera width.",
        default=None,
    )
    parser.add_argument(
        "--image-height",
        type=int,
        required=False,
        help="The height of the camera. Default is the camera height.",
        default=None,
    )

    args = parser.parse_args()

    video_capture_path = os.getenv("CAPTURE_PATH", args.path)

    if isinstance(video_capture_path, str) and video_capture_path.isnumeric():
        video_capture_path = int(video_capture_path)

    print(type(video_capture_path))

    image_width = os.getenv("IMAGE_WIDTH", args.image_width)
    image_height = os.getenv("IMAGE_HEIGHT", args.image_height)

    if image_width is not None:
        if isinstance(image_width, str) and image_width.isnumeric():
            image_width = int(image_width)

    if image_height is not None:
        if isinstance(image_height, str) and image_height.isnumeric():
            image_height = int(image_height)

    video_capture = cv2.VideoCapture(video_capture_path)
    node = Node(args.name)

    pa.array([])  # initialize pyarrow array

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                ret, frame = video_capture.read()

                if not ret:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        frame,
                        f"Error: Could not read frame from camera at path {video_capture_path}.",
                        (int(30), int(30)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.50,
                        (255, 255, 255),
                        1,
                        1,
                    )

                # resize the frame
                if image_width is not None and image_height is not None:
                    frame = cv2.resize(frame, (image_width, image_height))

                image = {
                    "width": pa.scalar(frame.shape[1], type=pa.uint32()),
                    "height": pa.scalar(frame.shape[0], type=pa.uint32()),
                    "channels": pa.scalar(frame.shape[2], type=pa.uint8()),
                    "data": frame.ravel(),
                }

                node.send_output("image", pa.array([image]), event["metadata"])

        elif event_type == "ERROR":
            raise Exception(event["error"])


if __name__ == "__main__":
    main()
