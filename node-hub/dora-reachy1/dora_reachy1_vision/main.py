"""TODO: Add docstring."""

from reachy_sdk import ReachySDK
import os
from dora import Node
import pyarrow as pa


def main():
    """TODO: Add docstring."""
    node = Node()

    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.24")
    CAMERA = os.getenv("CAMERA", "right")

    reachy = ReachySDK(ROBOT_IP, with_mobile_base=False)

    for event in node:
        if event["type"] == "INPUT":
            if CAMERA == "right":
                frame = reachy.right_camera.last_frame
            else:
                frame = reachy.left_camera.last_frame
            encoding = "bgr8"
            metadata = {}
            metadata["width"] = int(frame.shape[1])
            metadata["height"] = int(frame.shape[0])
            metadata["encoding"] = encoding

            storage = pa.array(frame.ravel())
            node.send_output("image", storage, metadata)
