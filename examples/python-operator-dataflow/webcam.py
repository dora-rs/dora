
import os
import numpy as np
import pyarrow as pa
from dora import DoraStatus

# HEADLESS FIX: Must be set BEFORE cv2 import
if os.environ.get("GITHUB_ACTIONS") == "true":
    os.environ["QT_QPA_PLATFORM"] = "offscreen"

import cv2  # noqa: E402

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", 0))
font = cv2.FONT_HERSHEY_SIMPLEX


class Operator:
    """Captures and streams webcam frames."""

    def __init__(self):
        """Initializes the webcam capture at the specified camera index."""
        self.video_capture = cv2.VideoCapture(CAMERA_INDEX)
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        self.failure_count = 0

    def on_event(
        self,
        dora_event: str,
        send_output,
    ) -> DoraStatus:
        """Capture a frame from the webcam and push it to the dataflow.

        Args:
            dora_event (dict): The event from dora-rs.
            send_output (Callable): Callback to emit the captured image frame.

        Returns:
            DoraStatus: CONTINUE to keep capturing, or STOP if the runtime
                signals a shutdown.
        """
        event_type = dora_event["type"]
        if event_type == "INPUT":
            ret, frame = self.video_capture.read()
            if ret:
                frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
                self.failure_count = 0
            elif self.failure_count > 10:
                frame = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
                cv2.putText(
                    frame, "No Webcam Found", (30, 30), font, 0.75, (255, 255, 255), 2
                )
            else:
                self.failure_count += 1
                return DoraStatus.CONTINUE
            # Clean metadata {} fixes the 'could not convert type' error
            send_output("image", pa.array(frame.ravel()), {})
        elif dora_event["type"] == "STOP":
            return DoraStatus.STOP
        return DoraStatus.CONTINUE

    def __del__(self):
        """Releases the webcam resources upon operator destruction."""
        self.video_capture.release()