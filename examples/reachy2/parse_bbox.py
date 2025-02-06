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

        return np.array(bboxes)
    except:
        pass


for event in node:
    if event["type"] == "INPUT":
        text = event["value"][0].as_py()

        bboxes = extract_bboxes(text)
        if bboxes is not None:
            bboxes = bboxes * 2
            node.send_output("bbox", pa.array(bboxes.ravel()))
            # 748 170 842 272
            # Angle is -38
            #  -0.116853796, 0.12726586, 0.57557464

            # h: -0.38  == np.sin(np.deg2rad(-38)) * 0.11 + np.cos(np.deg2rad(-38)) * 0.57
            # y: -0.12
            # x: 0.26 == np.cos(np.deg2rad(-38)) * 0.11 + np.sin(np.deg2rad(-38)) * 0.57
