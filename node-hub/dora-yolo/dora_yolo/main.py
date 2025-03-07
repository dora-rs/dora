import argparse
import os

import numpy as np
import pyarrow as pa
from dora import Node
from ultralytics import YOLO


def main() -> None:
    # Handle dynamic nodes, ask for the name of the node in the dataflow, and the same values as the ENV variables.
    parser = argparse.ArgumentParser(
        description="UltraLytics YOLO: This node is used to perform object detection using the UltraLytics YOLO model.",
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
    bbox_format = os.getenv("FORMAT", "xyxy")

    model = YOLO(model_path)
    node = Node(args.name)

    pa.array([])  # initialize pyarrow array

    for event in node:
        event_type = event["type"]

        if event_type == "INPUT":
            event_id = event["id"]

            if event_id == "image":
                storage = event["value"]
                metadata = event["metadata"]
                encoding = metadata["encoding"]
                width = metadata["width"]
                height = metadata["height"]

                if encoding in ("bgr8", "rgb8"):
                    channels = 3
                    storage_type = np.uint8
                else:
                    msg = f"Unsupported image encoding: {encoding}"
                    raise RuntimeError(msg)

                frame = (
                    storage.to_numpy()
                    .astype(storage_type)
                    .reshape((height, width, channels))
                )
                if encoding == "bgr8":
                    frame = frame[:, :, ::-1]  # OpenCV image (BGR to RGB)
                elif encoding == "rgb8":
                    pass
                else:
                    msg = f"Unsupported image encoding: {encoding}"
                    raise RuntimeError(msg)

                results = model(frame, verbose=False)  # includes NMS

                if bbox_format == "xyxy":
                    bboxes = np.array(results[0].boxes.xyxy.cpu())
                elif bbox_format == "xywh":
                    bboxes = np.array(results[0].boxes.xywh.cpu())
                else:
                    msg = f"Unsupported bbox format: {bbox_format}"
                    raise RuntimeError(msg)

                conf = np.array(results[0].boxes.conf.cpu())
                labels = np.array(results[0].boxes.cls.cpu())

                names = [model.names.get(label) for label in labels]

                bbox = {
                    "bbox": bboxes.ravel(),
                    "conf": conf,
                    "labels": names,
                }
                bbox = pa.array([bbox])

                metadata = event["metadata"]
                metadata["format"] = bbox_format

                node.send_output(
                    "bbox",
                    bbox,
                    metadata,
                )

        elif event_type == "ERROR":
            pass


if __name__ == "__main__":
    main()
