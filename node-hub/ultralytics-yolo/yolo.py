## Imports
import os

import numpy as np
import pyarrow as pa
from ultralytics import YOLO

from dora import Node

## OS Environment variable
IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", "640"))
IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", "480"))
MODEL = os.getenv("MODEL", "yolov8n.pt")

if __name__ == "__main__":

    model = YOLO(MODEL)

    node = Node("object_detection")

    for event in node:
        event_type = event["type"]
        if event_type == "INPUT":
            event_id = event["id"]
            if event_id == "image":
                frame = (
                    event["value"].to_numpy().reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 3))
                )
                frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                results = model(frame, verbose=False)  # includes NMS
                # Process results
                bboxes = np.array(results[0].boxes.xyxy.cpu())
                conf = np.array(results[0].boxes.conf.cpu())
                labels = np.array(results[0].boxes.cls.cpu())
                names = [model.names.get(label) for label in labels]

                node.send_output(
                    "bbox",
                    pa.array(
                        [
                            {
                                "bbox": bboxes.ravel(),
                                "conf": conf,
                                "labels": labels,
                                "names": names,
                            }
                        ]
                    ),
                    event["metadata"],
                )
