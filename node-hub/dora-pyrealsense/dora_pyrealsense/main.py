import argparse
import os
import time

import cv2
import numpy as np
import pyarrow as pa

from dora import Node

RUNNER_CI = True if os.getenv("CI") == "true" else False
import pyrealsense2 as rs

FLIP = os.getenv("FLIP", "")
DEVICE_ID = os.getenv("DEVICE_ID", "")
pipeline = rs.pipeline()
config = rs.config()
config.enable_device(DEVICE_ID)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
align_to = rs.stream.color
align = rs.align(align_to)
pipeline.start(config)


def main():
    node = Node()
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
                frames = pipeline.wait_for_frames()
                # 使用线程锁来确保只有一个线程能访问数据流
                # 获取各种数据流
                color_frames = frames.get_color_frame()
                frame = np.asanyarray(color_frames.get_data())

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
