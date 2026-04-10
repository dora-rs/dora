"""Camera node: captures frames from a USB camera and sends them as PyArrow arrays.

Waits for a ready signal from the recorder node before starting capture,
ensuring the recorder is initialized before any frames are sent.
"""

import os

import cv2
import pyarrow as pa
from dora import Node

CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", 0))
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


def main():
    node = Node()

    # Handshake: wait for recorder to signal it is ready before capturing
    print("Waiting for recorder ready signal...", flush=True)
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "recorder_status":
            print(f"Recorder ready: {event['value'][0].as_py()}", flush=True)
            break
        elif event["type"] == "STOP":
            return

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    print("Starting camera stream...", flush=True)
    frame_id = 0

    try:
        for event in node:
            if event["type"] == "INPUT" and event["id"] == "tick":
                ret, frame = cap.read()
                if not ret:
                    print("Camera read failed, skipping frame", flush=True)
                    continue

                frame = cv2.resize(frame, (CAMERA_WIDTH, CAMERA_HEIGHT))
                node.send_output(
                    "image",
                    pa.array(frame.ravel()),
                    metadata={
                        "width": CAMERA_WIDTH,
                        "height": CAMERA_HEIGHT,
                        "encoding": "bgr8",
                        "frame_id": frame_id,
                    },
                )
                frame_id += 1
            elif event["type"] == "STOP":
                break
    finally:
        cap.release()
        print(f"Camera node finished. Total frames sent: {frame_id}", flush=True)


if __name__ == "__main__":
    main()
