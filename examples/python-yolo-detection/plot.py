"""Plot node: overlays bounding boxes on camera frames and displays them in a window.

Caches the latest bounding-box list so it can be drawn even when a new image
arrives before the next detection result (async frame/bbox streams).
"""

import os

import cv2
from adora import Node
from utils import LABELS

WINDOW_NAME = os.environ.get("WINDOW_NAME", "frame")
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
FONT = cv2.FONT_HERSHEY_SIMPLEX


def main():
    node = Node()
    bboxs = []

    for event in node:
        if event["type"] == "INPUT":
            event_id = event["id"]
            value = event["value"]

            if event_id == "image":
                meta = event["metadata"]
                w = meta.get("width", CAMERA_WIDTH)
                h = meta.get("height", CAMERA_HEIGHT)
                image = value.to_numpy().reshape((h, w, 3)).copy()

                for bbox in bboxs:
                    [min_x, min_y, max_x, max_y, confidence, label] = bbox
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

                cv2.imshow(WINDOW_NAME, image)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            elif event_id == "bbox":
                arr = value.to_numpy()
                bboxs = arr.reshape((-1, 6)) if arr.size > 0 else []

        elif event["type"] == "STOP":
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
