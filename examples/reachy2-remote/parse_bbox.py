"""TODO: Add docstring."""

import json
import os

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()

IMAGE_RESIZE_RATIO = float(os.getenv("IMAGE_RESIZE_RATIO", "1.0"))


def extract_bboxes(json_text):
    """Extract bounding boxes from a JSON string with markdown markers and return them as a NumPy array.

    Parameters
    ----------
    json_text : str
        JSON string containing bounding box data, including ```json markers.

    Returns
    -------
    np.ndarray: NumPy array of bounding boxes.

    """
    # Ensure all lines are stripped of whitespace and markers
    lines = json_text.strip().splitlines()

    # Filter out lines that are markdown markers
    clean_lines = [line for line in lines if not line.strip().startswith("```")]

    # Join the lines back into a single string
    clean_text = "\n".join(clean_lines)
    # Parse the cleaned JSON text
    try:
        data = json.loads(clean_text)

        # Extract bounding boxes
        bboxes = [item["bbox_2d"] for item in data]
        labels = [item["label"] for item in data]

        return np.array(bboxes), np.array(labels)
    except Exception as _e:  # noqa
        pass
    return None, None


for event in node:
    if event["type"] == "INPUT":
        if len(event["value"]) == 0:
            node.send_output("bbox_track", pa.array([]))
            continue

        text = event["value"][0].as_py()
        metadata = event["metadata"]
        image_id = event["metadata"]["image_id"]

        bboxes, labels = extract_bboxes(text)
        if bboxes is not None and len(bboxes) > 0:
            bboxes = bboxes * int(1 / IMAGE_RESIZE_RATIO)
            metadata["image_id"] = image_id
            metadata["encoding"] = "xyxy"
            if image_id == "image_left":
                node.send_output(
                    "bbox_track",
                    pa.array(bboxes.ravel()),
                    metadata,
                )
            elif image_id == "image_depth":
                node.send_output(
                    "bbox_grab",
                    pa.array(bboxes.ravel()),
                    metadata,
                )
