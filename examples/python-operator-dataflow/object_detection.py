import os
import time
import numpy as np
import pyarrow as pa
from dora import DoraStatus
from ultralytics import YOLO

# Configuration
YOLO_MODEL_PATH = os.getenv("YOLO_MODEL", "yolov8n.pt")
CAMERA_WIDTH = 640
# Must be a multiple of 32 for optimal YOLO performance
IMGSZ = 320


class Operator:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)

    def on_event(self, dora_event, send_output) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            if dora_event["id"] != "image":
                return DoraStatus.CONTINUE

            start_inference = time.perf_counter()

            # Dynamic reshape — works on any camera resolution, not hardcoded to 480
            frame = dora_event["value"].to_numpy().reshape((-1, CAMERA_WIDTH, 3)).copy()

            # No BGR->RGB flip needed — Ultralytics handles color internally
            results = self.model(
                frame,
                verbose=False,
                imgsz=IMGSZ,
                conf=0.25,
            )

            latency = (time.perf_counter() - start_inference) * 1000

            # Extract boxes, confidence scores, and class labels
            boxes = np.array(results[0].boxes.xyxy.cpu())
            conf = np.array(results[0].boxes.conf.cpu())
            label = np.array(results[0].boxes.cls.cpu())

            # FIXED: np.zeros((0,6)) so plot.py reshape(-1,6) never fails on empty
            if len(boxes) > 0:
                arrays = np.concatenate(
                    (boxes, conf[:, None], label[:, None]), axis=1
                ).astype(np.float32)
            else:
                arrays = np.zeros((0, 6), dtype=np.float32)

            # Send outputs with clean metadata
            send_output("bbox", pa.array(arrays.ravel()), {})
            send_output("latency", pa.array([latency]), {})

        return DoraStatus.CONTINUE