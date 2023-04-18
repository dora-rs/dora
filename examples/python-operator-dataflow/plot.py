#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from typing import Callable

import cv2
import numpy as np
import pyarrow as pa
from utils import LABELS

from dora import DoraStatus

pa.array([])

CI = os.environ.get("CI")
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

font = cv2.FONT_HERSHEY_SIMPLEX


class Operator:
    """
    Plot image and bounding box
    """

    def __init__(self):
        self.image = []
        self.bboxs = []
        self.bounding_box_messages = 0
        self.image_messages = 0

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
        """
        Put image and bounding box on cv2 window.

        Args:
            dora_input["id"] (str): Id of the dora_input declared in the yaml configuration
            dora_input["data"] (bytes): Bytes message of the dora_input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """
        if dora_input["id"] == "image":
            frame = (
                dora_input["value"]
                .to_numpy()
                .reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3))
            )
            self.image = frame

            self.image_messages += 1
            print("received " + str(self.image_messages) + " images")

        elif dora_input["id"] == "bbox" and len(self.image) != 0:
            bboxs = dora_input["value"].to_numpy().view(np.float32)
            self.bboxs = np.reshape(bboxs, (-1, 6))

            self.bounding_box_messages += 1
            print(
                "received "
                + str(self.bounding_box_messages)
                + " bounding boxes"
            )

        for bbox in self.bboxs:
            [
                min_x,
                min_y,
                max_x,
                max_y,
                confidence,
                label,
            ] = bbox
            cv2.rectangle(
                self.image,
                (int(min_x), int(min_y)),
                (int(max_x), int(max_y)),
                (0, 255, 0),
                2,
            )

            cv2.putText(
                self.image,
                LABELS[int(label)] + f", {confidence:0.2f}",
                (int(max_x), int(max_y)),
                font,
                0.75,
                (0, 255, 0),
                2,
                1,
            )

        if CI != "true":
            cv2.imshow("frame", self.image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return DoraStatus.STOP

        return DoraStatus.CONTINUE

    def __del__(self):
        cv2.destroyAllWindows()
