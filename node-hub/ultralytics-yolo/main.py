import os
import argparse

import numpy as np
import pyarrow as pa

from dora import Node
from ultralytics import YOLO

LABELS = [
    "ABC",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
]


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same values as the ENV variables.
    parser = argparse.ArgumentParser(
        description="UltraLytics YOLO: This node is used to perform object detection using the UltraLytics YOLO model.")

    parser.add_argument("--name", type=str, required=False, help="The name of the node in the dataflow.",
                        default="opencv-video-capture")
    parser.add_argument("--model", type=str, required=False,
                        help="The name of the model file (e.g. yolov8n.pt).", default="yolov8n.pt")

    args = parser.parse_args()

    model_path = os.getenv("MODEL", args.model)

    model = YOLO(model_path)
    node = Node(args.name)

    pa.array([])  # initialize pyarrow array

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "image":
                arrow_image = event["value"][0]
                image = {
                    "width": np.uint32(arrow_image["width"].as_py()),
                    "height": np.uint32(arrow_image["height"].as_py()),
                    "channels": np.uint8(arrow_image["channels"].as_py()),
                    "data": arrow_image["data"].values.to_numpy().astype(np.uint8)
                }

                frame = image["data"].reshape((image["height"], image["width"], 3))

                frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                results = model(frame, verbose=False)  # includes NMS

                bboxes = np.array(results[0].boxes.xyxy.cpu())
                conf = np.array(results[0].boxes.conf.cpu())
                labels = np.array(results[0].boxes.cls.cpu())

                names = [LABELS[int(label)] for label in labels]

                bbox = {
                    "bbox": bboxes.ravel(),
                    "conf": conf,
                    "names": names,
                }

                node.send_output(
                    "bbox",
                    pa.array([bbox]),
                    event["metadata"],
                )

        elif event_type == "STOP":
            break
        elif event_type == "ERROR":
            raise Exception(event["error"])


if __name__ == "__main__":
    main()
