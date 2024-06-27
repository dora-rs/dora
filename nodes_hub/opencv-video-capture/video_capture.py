import os
import time

import cv2
import numpy as np
import pyarrow as pa

from dora import Node

CAM_INDEX = int(os.getenv("CAM_INDEX", "0"))
IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", "640"))
IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", "480"))
MAX_DURATION = int(os.getenv("DURATION", "20"))
FONT = cv2.FONT_HERSHEY_SIMPLEX


start = time.time()


if __name__ == "__main__":

    video_capture = cv2.VideoCapture(CAM_INDEX)
    node = Node()

    while time.time() - start < MAX_DURATION:
        event = node.next()
        if event is None:
            break
        if event is not None:
            event_type = event["type"]
            if event_type == "INPUT":
                ret, frame = video_capture.read()

                # Fail to read camera
                if not ret:
                    frame = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype=np.uint8)
                    cv2.putText(
                        frame,
                        "No Webcam was found at index %d" % (CAM_INDEX),
                        (int(30), int(30)),
                        FONT,
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
