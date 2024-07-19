import os
import argparse

import numpy as np
import pyarrow as pa

from dora import Node
from ultralytics import YOLO


def main():
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same values as the ENV variables.
    parser = argparse.ArgumentParser(
        description="UltraLytics YOLO: This node is used to perform object detection using the UltraLytics YOLO model."
    )

    parser.add_argument(
        "--name",
        type=str,
        required=False,
        help="The name of the node in the dataflow.",
        default="ultralytics-yolo",
    )
    parser.add_argument(
        "--model",
        type=str,
        required=False,
        help="The name of the model file (e.g. yolov8n.pt).",
        default="yolov8n.pt",
    )

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
                encoding = arrow_image["encoding"].as_py()

                if encoding == "bgr8":
                    channels = 3
                    storage_type = np.uint8
                else:
                    raise Exception(f"Unsupported image encoding: {encoding}")

                image = {
                    "width": np.uint32(arrow_image["width"].as_py()),
                    "height": np.uint32(arrow_image["height"].as_py()),
                    "encoding": encoding,
                    "channels": channels,
                    "data": arrow_image["data"].values.to_numpy().astype(storage_type),
                }

                frame = image["data"].reshape(
                    (image["height"], image["width"], image["channels"])
                )

                if encoding == "bgr8":
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)

                results = model(frame, verbose=False)  # includes NMS

                bboxes = np.array(results[0].boxes.xyxy.cpu())
                conf = np.array(results[0].boxes.conf.cpu())
                labels = np.array(results[0].boxes.cls.cpu())

                names = [model.names.get(label) for label in labels]

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

        elif event_type == "ERROR":
            raise Exception(event["error"])


if __name__ == "__main__":
    main()
