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
    bboxes: np.array = np.array([[]])
    conf: np.array = np.array([])
    label: np.array = np.array([])


if __name__ == "__main__":
    plotter = Plotter()
    node = Node()
    for event in node:
        event_type = event["type"]
        if event_type == "INPUT":
            if event["id"] == "image":
                frame = event["value"].to_numpy()
                frame = frame.reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 3)).copy()
                plotter.frame = frame

            elif event["id"] == "bbox":
                bboxes = event["value"][0]["bbox"].values.to_numpy()
                conf = event["value"][0]["conf"].values.to_numpy()
                label = event["value"][0]["names"].values.to_pylist()
                plotter.bboxes = np.reshape(bboxes, (-1, 4))
                plotter.conf = conf
                plotter.label = label
                continue

            for bbox in zip(plotter.bboxes, plotter.conf, plotter.label):
                [
                    [min_x, min_y, max_x, max_y],
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
                    f"{label}, {confidence:0.2f}",
                    (int(max_x) - 120, int(max_y) - 10),
                    FONT,
                    0.5,
                    (0, 255, 0),
                    2,
                    1,
                )

            if CI != "true":
                cv2.imshow("frame", plotter.frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
