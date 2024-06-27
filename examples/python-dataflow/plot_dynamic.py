import os
from dataclasses import dataclass

import cv2
import numpy as np

from dora import Node

CI = os.environ.get("CI")

IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", "640"))
IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", "480"))

FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class Plotter:
    frame: np.array = np.array([])
    bboxes: np.array = np.array([])


if __name__ == "__main__":
    plotter = Plotter()
    node = Node("plot")

    for event in node:
        event_type = event["type"]
        if event_type == "INPUT":
            if event["id"] == "image":
                frame = event["value"].to_numpy()
                frame = (
                    event["value"].to_numpy().reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 3))
                )
                plotter.frame = frame

            elif event["id"] == "bbox" and len(plotter.frame) != 0:
                bboxs = event["value"].to_numpy()
                plotter.bboxes = np.reshape(bboxs, (-1, 6))
            for bbox in plotter.bboxs:
                [
                    min_x,
                    min_y,
                    max_x,
                    max_y,
                    confidence,
                    label,
                ] = bbox
                cv2.rectangle(
                    plotter.frame,
                    (int(min_x), int(min_y)),
                    (int(max_x), int(max_y)),
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    plotter.frame,
                    LABELS[int(label)] + f", {confidence:0.2f}",
                    (int(max_x), int(max_y)),
                    FONT,
                    0.75,
                    (0, 255, 0),
                    2,
                    1,
                )

            if CI != "true":
                cv2.imshow("frame", plotter.frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
