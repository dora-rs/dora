"""TODO: Add docstring."""

import os

import pyarrow as pa
from dora import Node
from reachy_sdk import ReachySDK


def main():
    """TODO: Add docstring."""
    node = Node()

    robot_ip = os.getenv("ROBOT_IP", "10.42.0.24")
    camera = os.getenv("CAMERA", "right")

    reachy = ReachySDK(robot_ip, with_mobile_base=False)

    for event in node:
        if event["type"] == "INPUT":
            if camera == "right":
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
