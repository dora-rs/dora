import os
import cv2
import time

from dora import DoraStatus
from utils import LABELS


CI = os.environ.get("CI")

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

FONT = cv2.FONT_HERSHEY_SIMPLEX


class Operator:
    """
    Plot image and bounding box
    """

    def __init__(self):
        self.bboxs = []
        self.buffer = ""
        self.submitted = []
        self.lines = []

    def on_event(
        self,
        dora_event,
        send_output,
    ):
        if dora_event["type"] == "INPUT":
            id = dora_event["id"]
            value = dora_event["value"]
            if id == "image":

                image = (
                    value.to_numpy().reshape((CAMERA_HEIGHT, CAMERA_WIDTH, 3)).copy()
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
                        image,
                        (int(min_x), int(min_y)),
                        (int(max_x), int(max_y)),
                        (0, 255, 0),
                    )
                    cv2.putText(
                        image,
                        f"{LABELS[int(label)]}, {confidence:0.2f}",
                        (int(max_x), int(max_y)),
                        FONT,
                        0.5,
                        (0, 255, 0),
                    )

                cv2.putText(
                    image, self.buffer, (20, 14 + 21 * 14), FONT, 0.5, (190, 250, 0), 1
                )

                i = 0
                for text in self.submitted[::-1]:
                    color = (
                        (0, 255, 190)
                        if text["role"] == "user_message"
                        else (0, 190, 255)
                    )
                    cv2.putText(
                        image,
                        text["content"],
                        (
                            20,
                            14 + (19 - i) * 14,
                        ),
                        FONT,
                        0.5,
                        color,
                        1,
                    )
                    i += 1

                for line in self.lines:
                    cv2.line(
                        image,
                        (int(line[0]), int(line[1])),
                        (int(line[2]), int(line[3])),
                        (0, 0, 255),
                        2,
                    )

                if CI != "true":
                    cv2.imshow("frame", image)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        return DoraStatus.STOP
            elif id == "bbox":
                self.bboxs = value.to_numpy().reshape((-1, 6))
            elif id == "keyboard_buffer":
                self.buffer = value[0].as_py()
            elif id == "line":
                self.lines += [value.to_pylist()]
            elif "message" in id:
                self.submitted += [
                    {
                        "role": id,
                        "content": value[0].as_py(),
                    }
                ]

        return DoraStatus.CONTINUE
