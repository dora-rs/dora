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

# When no real camera is available (e.g. a headless CI runner), run only this
# long before stopping. `read()` is skipped in that case so the operator stays
# responsive, and the window keeps us well inside the daemon's `--stop-after`
# (20s) so the node self-exits cleanly instead of being force-killed at the
# grace period (ExitCode(1) on Windows, reddening the nightly — #2742). Long
# enough for the downstream YOLO node to warm up and run inference on a few
# frames; short enough to leave a wide margin under the stop-after.
CI_HEADLESS_RUN_SECS = 10

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
        # Whether the camera actually opened. On a headless runner it does not,
        # and `read()` on some backends (Windows MSMF/obsensor) blocks for
        # seconds per call with no device — long enough that the operator can't
        # observe STOP within the daemon's grace period and gets force-killed
        # (#2742). Gate on this so we never call the blocking read() when there
        # is no camera. The log line is the CI evidence that the gate fired.
        self.camera_available = self.video_capture.isOpened()
        print(
            f"webcam: camera_available={self.camera_available} (index {CAMERA_INDEX})"
        )

    def on_event(
        self,
        dora_event: str,
        send_output,
    ) -> DoraStatus:
        """TODO: Add docstring."""
        event_type = dora_event["type"]
        if event_type == "INPUT":
            frame = None
            if self.camera_available:
                read_start = time.time()
                ret, captured = self.video_capture.read()
                read_secs = time.time() - read_start
                # Surface a slow read so a future regression is visible in the
                # logs rather than only as a grace-period force-kill (#2742).
                if read_secs > 1.0:
                    print(f"webcam: cv2 read() took {read_secs:.1f}s")
                if ret:
                    frame = cv2.resize(captured, (CAMERA_WIDTH, CAMERA_HEIGHT))
                    self.failure_count = 0
                elif self.failure_count <= 10:
                    self.failure_count += 1
                    return DoraStatus.CONTINUE

            # No usable camera frame (no device, or read() keeps failing): push
            # a placeholder so downstream nodes still run. When there is no
            # camera, read() above is skipped, keeping the operator responsive.
            headless = frame is None
            if headless:
                frame = self._placeholder_frame()

            send_output(
                "image",
                pa.array(frame.ravel()),
                dora_event["metadata"],
            )

            # Stop promptly under CI when there is no real camera, so the node
            # exits well inside the daemon's `--stop-after`/grace instead of
            # being force-killed at the grace period (#2742).
            if (
                headless
                and CI == "true"
                and time.time() - self.start_time > CI_HEADLESS_RUN_SECS
            ):
                return DoraStatus.STOP
        elif event_type == "STOP":
            print("received stop")
            return DoraStatus.STOP
        else:
            print("received unexpected event:", event_type)

        if time.time() - self.start_time < 20 or CI != "true":
            return DoraStatus.CONTINUE
        return DoraStatus.STOP

    def _placeholder_frame(self):
        """Synthetic frame shown when no camera is available."""
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
        return frame

    def __del__(self):
        """TODO: Add docstring."""
        self.video_capture.release()
