#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import Callable

import cv2
import numpy as np
import pyarrow as pa
import torch

from dora import DoraStatus

pa.array([])

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class Operator:
    """
    Infering object from images
    """

    def __init__(self):
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5n")

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """Handle image
        Args:
            dora_input (dict): Dict containing the "id", "data", and "metadata"
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """

        frame = (
            dora_input["value"]
            .to_numpy()
            .reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
        )
        frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
        results = self.model(frame)  # includes NMS
        arrays = pa.array(
            np.array(results.xyxy[0].cpu()).ravel().view(np.uint8)
        )
        send_output("bbox", arrays, dora_input["metadata"])
        return DoraStatus.CONTINUE
