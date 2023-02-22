#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from typing import Callable
from dora import Node
from dora import DoraStatus

import cv2
import numpy as np
from utils import LABELS

CI = os.environ.get("CI")

font = cv2.FONT_HERSHEY_SIMPLEX
class Plotter:
    """
    Plot image and bounding box
    """

    def __init__(self):
        self.image = []
        self.bboxs = []

    def on_input(
        self,
        dora_input: dict,
    ) -> DoraStatus:
        """
        Put image and bounding box on cv2 window.

        Args:
            dora_input["id"] (str): Id of the dora_input declared in the yaml configuration
            dora_input["data"] (bytes): Bytes message of the dora_input
        """
        if dora_input["id"] == "image":
            frame = np.frombuffer(dora_input["data"], dtype="uint8")
            frame = cv2.imdecode(frame, -1)
            self.image = frame

        elif dora_input["id"] == "bbox" and len(self.image) != 0:
            bboxs = np.frombuffer(dora_input["data"], dtype="float32")
            self.bboxs = np.reshape(bboxs, (-1, 6))
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



plotter = Plotter()
node = Node()

for event in node:
    match event["type"]:
        case "INPUT":
            status = plotter.on_input(event)
            match status:
                case DoraStatus.CONTINUE:
                    pass
                case DoraStatus.STOP:
                    print("plotter returned stop status")
                    break
        case "STOP":
            print("received stop")
        case other:
            print("received unexpected event:", other)
