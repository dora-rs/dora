#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
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
        dora_input,
    ) -> DoraStatus:
        """
        Put image and bounding box on cv2 window.

        Args:
            dora_input["id"] (str): Id of the dora_input declared in the yaml configuration
            dora_input["value"] (arrow array): message of the dora_input
        """
        if dora_input["id"] == "image":
            frame = dora_input["value"].to_numpy()
            frame = cv2.imdecode(frame, -1)
            self.image = frame

        elif dora_input["id"] == "bbox" and len(self.image) != 0:
            bboxs = dora_input["value"].to_numpy()
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

try:
    node = Node()
except RuntimeError as e:
    print("Dataflow initialization failed: ", e)
    exit(0)

for event in node:
    event_type = event["type"]
    if event_type == "INPUT":
        status = plotter.on_input(event)
        if status == DoraStatus.CONTINUE:
            pass
        elif status == DoraStatus.STOP:
            print("plotter returned stop status")
            break
    elif event_type == "STOP":
        print("received stop")
    else:
        print("received unexpected event:", event_type)
