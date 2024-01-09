#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import pyarrow as pa

from dora import DoraStatus
from ultralytics import YOLO

pa.array([])

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class Operator:
    """
    Infering object from images
    """

    def __init__(self):
        self.model = YOLO("yolov8n.pt")

    def on_event(
        self,
        dora_event,
        send_output,
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input,
        send_output,
    ) -> DoraStatus:
        """Handle image
        Args:
            dora_input (dict) containing the "id", value, and "metadata"
            send_output Callable[[str, bytes | pa.Array, Optional[dict]], None]:
                Function for sending output to the dataflow:
                - First argument is the `output_id`
                - Second argument is the data as either bytes or `pa.Array`
                - Third argument is dora metadata dict
                e.g.: `send_output("bbox", pa.array([100], type=pa.uint8()), dora_event["metadata"])`
        """

        frame = dora_input["value"].to_numpy().reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
        results = self.model(frame)  # includes NMS
        # Process results
        boxes = np.array(results[0].boxes.xyxy.cpu())
        conf = np.array(results[0].boxes.conf)
        label = np.array(results[0].boxes.cls)
        # concatenate them together
        arrays = np.concatenate((boxes, conf[:, None], label[:, None]), axis=1)

        send_output("bbox", pa.array(arrays.ravel()), dora_input["metadata"])
        return DoraStatus.CONTINUE
