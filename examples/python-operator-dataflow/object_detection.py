import os
import time
import numpy as np
import pyarrow as pa
from dora import DoraStatus
from ultralytics import YOLO

# Configuration
YOLO_MODEL_PATH = os.getenv("YOLO_MODEL", "yolov8n.pt")
# FIX 1: Ensure imgsz is a multiple of 32 (320 or 256 are great)
IMGSZ = 320 

class Operator:
    def __init__(self):
        self.model = YOLO(YOLO_MODEL_PATH)

    def on_event(self, dora_event, send_output) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            # FIX 2: We use a try/except or a check to ignore non-image metadata
            if dora_event["id"] != "image":
                return DoraStatus.CONTINUE

            start_inference = time.perf_counter()
            
            # Convert Arrow array to numpy
            frame = dora_event["value"].to_numpy().reshape((480, 640, 3)).copy()
            frame = frame[:, :, ::-1] # BGR to RGB

            # Run inference with the fixed IMGSZ
            results = self.model(
                frame, 
                verbose=False, 
                imgsz=IMGSZ, 
                conf=0.25 # Lowered to see more objects like the mouse!
            )

            latency = (time.perf_counter() - start_inference) * 1000
            
            # Process results...
            boxes = np.array(results[0].boxes.xyxy.cpu())
            conf = np.array(results[0].boxes.conf.cpu())
            label = np.array(results[0].boxes.cls.cpu())
            arrays = np.concatenate((boxes, conf[:, None], label[:, None]), axis=1) if len(boxes) > 0 else np.array([], dtype=np.float32)

            # Send outputs with CLEAN metadata
            send_output("bbox", pa.array(arrays.ravel()), {})
            send_output("latency", pa.array([latency]), {})

        return DoraStatus.CONTINUE