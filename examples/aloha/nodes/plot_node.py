import os

import cv2
from dora import Node

IMAGE_WIDTH = int(os.getenv("IMAGE_WIDTH", "1280"))
IMAGE_HEIGHT = int(os.getenv("IMAGE_HEIGHT", "720"))
FONT = cv2.FONT_HERSHEY_SIMPLEX


node = Node()

joint = None
text = None

for event in node:
    if event["type"] == "INPUT":
        dora_id = event["id"]

        if dora_id == "position":
            joint = event["value"].to_numpy()
        if "text" in dora_id:
            text = event["value"][0].as_py()

        if dora_id == "image":
            image = (
                event["value"].to_numpy().reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 3)).copy()
            )
            if text is not None:
                cv2.putText(
                    image,
                    f"Speech: {text}",
                    (20, 40),
                    FONT,
                    0.5,
                    (190, 250, 0),
                    2,
                )

            if joint is not None:
                cv2.putText(
                    image,
                    f"pos: {joint}",
                    (20, 20),
                    FONT,
                    0.5,
                    (190, 250, 100),
                    2,
                )

            cv2.imshow("frame", image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
