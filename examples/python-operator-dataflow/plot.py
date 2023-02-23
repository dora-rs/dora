import os
from enum import Enum
from typing import Callable

import cv2
import numpy as np

from utils import LABELS

CI = os.environ.get("CI")

font = cv2.FONT_HERSHEY_SIMPLEX


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Plot image and bounding box
    """

    def __init__(self):
        self.image = []
        self.bboxs = []

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)

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

    def __del__(self):
        cv2.destroyAllWindows()
