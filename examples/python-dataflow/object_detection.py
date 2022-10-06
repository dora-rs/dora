from enum import Enum
from typing import Callable

import cv2
import numpy as np
import torch


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Infering object from images
    """

    def __init__(self):
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5n")

    def on_input(
        self,
        input: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """Handle image

        Args:
            input_id (str): Id of the input declared in the yaml configuration
            value (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """

        frame = np.frombuffer(input["data"], dtype="uint8")
        frame = cv2.imdecode(frame, -1)
        frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)

        results = self.model(frame)  # includes NMS
        arrays = np.array(results.xyxy[0].cpu()).tobytes()
        send_output("bbox", arrays, input["metadata"])
        return DoraStatus.CONTINUE
