import argparse
import os
import time

import cv2
import numpy as np
import pyarrow as pa

from dora import Node

RUNNER_CI = True if os.getenv("CI") == "true" else False

FLIP = os.getenv("FLIP", "")


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
    encoding = os.getenv("ENCODING", "bgr8")

    if isinstance(video_capture_path, str) and video_capture_path.isnumeric():
        video_capture_path = int(video_capture_path)

    video_capture = cv2.VideoCapture(video_capture_path)

    image_width = os.getenv("IMAGE_WIDTH", args.image_width)

    if image_width is not None:
        if isinstance(image_width, str) and image_width.isnumeric():
            image_width = int(image_width)
        video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)

    image_height = os.getenv("IMAGE_HEIGHT", args.image_height)
    if image_height is not None:
        if isinstance(image_height, str) and image_height.isnumeric():
            image_height = int(image_height)
        video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)

    node = Node(args.name)
    start_time = time.time()

    pa.array([])  # initialize pyarrow array

    for event in node:

        # Run this example in the CI for 10 seconds only.
        if RUNNER_CI and time.time() - start_time > 10:
            break

        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "tick":
                ret, frame = video_capture.read()

                if not ret:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(
                        frame,
                        f"Error: no frame for camera at path {video_capture_path}.",
                        (int(30), int(30)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.50,
                        (255, 255, 255),
                        1,
                        1,
                    )

                if FLIP == "VERTICAL":
                    frame = cv2.flip(frame, 0)
                elif FLIP == "HORIZONTAL":
                    frame = cv2.flip(frame, 1)
                elif FLIP == "BOTH":
                    frame = cv2.flip(frame, -1)

                # resize the frame
                if (
                    image_width is not None
                    and image_height is not None
                    and (
                        frame.shape[1] != image_width or frame.shape[0] != image_height
                    )
                ):
                    frame = cv2.resize(frame, (image_width, image_height))

                metadata = event["metadata"]
                metadata["encoding"] = encoding
                metadata["width"] = int(frame.shape[1])
                metadata["height"] = int(frame.shape[0])

                # Get the right encoding
                if encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                elif encoding in ["jpeg", "jpg", "jpe", "bmp", "webp", "png"]:
                    ret, frame = cv2.imencode("." + encoding, frame)
                    if not ret:
                        print("Error encoding image...")
                        continue

                storage = pa.array(frame.ravel())

                node.send_output("image", storage, metadata)

        elif event_type == "ERROR":
            raise RuntimeError(event["error"])


if __name__ == "__main__":
    main()
