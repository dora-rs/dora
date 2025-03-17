"""TODO: Add docstring."""

import os
import time

import cv2
import numpy as np
import pyarrow as pa
from dora import DoraStatus

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", 0))
CI = os.environ.get("CI")

font = cv2.FONT_HERSHEY_SIMPLEX


class Operator:
    """Sending image from webcam to the dataflow."""

    def __init__(self):
        """TODO: Add docstring."""
        self.video_capture = cv2.VideoCapture(CAMERA_INDEX)
        self.start_time = time.time()
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.failure_count = 0

    def on_event(
        self,
        dora_event: str,
        send_output,
    ) -> DoraStatus:
        """TODO: Add docstring."""
        event_type = dora_event["type"]
        if event_type == "INPUT":
            ret, frame = self.video_capture.read()
            if ret:
                frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
                self.failure_count = 0
            ## Push an error image in case the camera is not available.
            elif self.failure_count > 10:
                frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
                cv2.putText(
                    frame,
                    f"No Webcam was found at index {CAMERA_INDEX}",
                    (30, 30),
                    font,
                    0.75,
                    (255, 255, 255),
                    2,
                    1,
                )
            else:
                self.failure_count += 1
                return DoraStatus.CONTINUE

            send_output(
                "image",
                pa.array(frame.ravel()),
                dora_event["metadata"],
            )
        elif event_type == "STOP":
            print("received stop")
        else:
            print("received unexpected event:", event_type)

        if time.time() - self.start_time < 20 or CI != "true":
            return DoraStatus.CONTINUE
        return DoraStatus.STOP

    def __del__(self):
        """TODO: Add docstring."""
        self.video_capture.release()
