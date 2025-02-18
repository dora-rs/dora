import json

import numpy as np
import pyarrow as pa
from dora import Node

node = Node()


def extract_bboxes(json_text):
    """
    Extracts bounding boxes from a JSON string with markdown markers and returns them as a NumPy array.

    Parameters:
    json_text (str): JSON string containing bounding box data, including ```json markers.

    Returns:
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
        text = event["value"][0].as_py()

        bboxes, labels = extract_bboxes(text)
        if bboxes is not None and len(bboxes) > 0:
            bboxes = bboxes  # * 2
            if "human" in labels[0] or "head" in labels[0]:
                node.send_output("bbox_face", pa.array(bboxes.ravel()))
            else:
                node.send_output("bbox", pa.array(bboxes.ravel()))
