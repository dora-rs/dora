"""Object detection node: runs YOLOv8-nano on incoming frames and outputs bounding boxes.

Forces CPU inference to avoid GPU memory contention when multiple detection
nodes run in parallel (e.g. in the multi-camera dataflow).
"""

import numpy as np
import pyarrow as pa
from dora import Node
from ultralytics import YOLO

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Force CPU to prevent SIGSEGV from GPU memory contention in multi-process setups
model = YOLO("yolov8n.pt")
model.to("cpu")


def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            raw = event["value"].to_numpy()
            meta = event["metadata"]
            w = meta.get("width", CAMERA_WIDTH)
            h = meta.get("height", CAMERA_HEIGHT)

            # Guard against partial frames that would cause a reshape error
            if raw.size != h * w * 3:
                continue

            frame = raw.reshape((h, w, 3))
            frame = frame[:, :, ::-1]  # BGR to RGB

            results = model(frame, verbose=False, device="cpu")

            boxes = np.array(results[0].boxes.xyxy.cpu())
            conf = np.array(results[0].boxes.conf.cpu())
            label = np.array(results[0].boxes.cls.cpu())

            if len(boxes) > 0:
                arrays = np.concatenate((boxes, conf[:, None], label[:, None]), axis=1)
            else:
                arrays = np.empty((0, 6), dtype=np.float32)

            node.send_output("bbox", pa.array(arrays.ravel()), event["metadata"])

        elif event["type"] == "STOP":
            break


if __name__ == "__main__":
    main()
