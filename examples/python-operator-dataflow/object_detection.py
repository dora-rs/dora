"""Object detection operator for dora-rs dataflow using YOLOv8.

This operator receives image frames, performs object detection using the
ultralytics YOLOv8 model, and emits bounding box coordinates, confidence
scores, and class labels for detected objects.
"""

import numpy as np
import pyarrow as pa
from dora import DoraStatus
from ultralytics import YOLO

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


model = YOLO("yolov8n.pt")


class Operator:
    """Inferring object from images."""

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        """Process incoming image frames and perform object detection.

        Args:
            dora_event (dict): The event from dora-rs, expected to contain
                a raw image buffer.
            send_output (Callable): Callback to emit detection results
                (bounding boxes, scores, and labels).

        Returns:
            DoraStatus: CONTINUE to allow further image processing.
        """
        if dora_event["type"] == "INPUT":
            frame = (
                dora_event["value"].to_numpy().reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
            )
            frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
            results = model(frame, verbose=False)  # includes NMS
            # Process results
            boxes = np.array(results[0].boxes.xyxy.cpu())
            conf = np.array(results[0].boxes.conf.cpu())
            label = np.array(results[0].boxes.cls.cpu())
            # concatenate them together
            arrays = np.concatenate((boxes, conf[:, None], label[:, None]), axis=1)

            send_output("bbox", pa.array(arrays.ravel()), dora_event["metadata"])

        return DoraStatus.CONTINUE
