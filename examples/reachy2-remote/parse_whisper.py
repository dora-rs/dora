"""TODO: Add docstring."""

import json
import os
import time

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
        text = event["value"][0].as_py().lower()

        if "stop" in text:
            node.send_output("points", pa.array([], type=pa.float64()))
        elif "follow" in text:
            text = f"Given the prompt: {text}. Output the bounding boxes for the given followed object"
            node.send_output("text", pa.array([text]), {"image_id": "image_left"})
        elif "grab" in text:
            text = f"Given the prompt: {text}. Output the bounding boxes for the given grabbed object"
            node.send_output("text", pa.array([text]), {"image_id": "image_depth"})
        elif "left" in text:
            action = pa.array([0.0, 0, 0, 0, 0, np.deg2rad(160)])
            node.send_output("action", action)
            time.sleep(0.25)
            action = pa.array([0.0, 0, 0, 0, 0, np.deg2rad(160)])
            node.send_output("action", action)
            time.sleep(0.25)
            action = pa.array([0.0, 0, 0, 0, 0, np.deg2rad(160)])
            node.send_output("action", action)
            node.send_output("points", pa.array([]))
        elif "right" in text:
            action = pa.array([0.0, 0, 0, 0, 0, -np.deg2rad(160)])
            node.send_output("action", action)
            time.sleep(0.25)
            action = pa.array([0.0, 0, 0, 0, 0, -np.deg2rad(160)])
            node.send_output("action", action)
            time.sleep(0.25)
            action = pa.array([0.0, 0, 0, 0, 0, -np.deg2rad(160)])
            node.send_output("action", action)
            node.send_output("points", pa.array([]))
