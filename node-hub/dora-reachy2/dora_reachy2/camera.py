"""TODO: Add docstring."""

import os

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView


def main():
    """TODO: Add docstring."""
    robot_ip = os.getenv("ROBOT_IP", "10.42.0.80")

    for _ in range(10):
        reachy = ReachySDK(robot_ip)
        try:
            reachy.cameras.teleop.get_frame(view=CameraView.LEFT)
            params = reachy.cameras.depth.get_parameters(view=CameraView.DEPTH)
            if params is not None:
                break
        except Exception as e:
            print(e)
        import time

        time.sleep(1)

    reachy.cameras.teleop.get_frame(view=CameraView.LEFT)
    params = reachy.cameras.depth.get_parameters(view=CameraView.DEPTH)
    height, width, _distortion_model, _d, k, _r, _p = params

    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "tick":
                (image_left, _) = reachy.cameras.teleop.get_frame(view=CameraView.LEFT)

                if image_left is int:
                    continue

                node.send_output(
                    "image_left",
                    pa.array(image_left.ravel()),
                    metadata={
                        "encoding": "bgr8",
                        "width": image_left.shape[1],
                        "height": image_left.shape[0],
                        "primitive": "image",
                    },
                )

                (image_right, _) = reachy.cameras.teleop.get_frame(
                    view=CameraView.RIGHT,
                )

                if image_right is int:
                    continue

                node.send_output(
                    "image_right",
                    pa.array(image_right.ravel()),
                    metadata={
                        "encoding": "bgr8",
                        "width": image_right.shape[1],
                        "height": image_right.shape[0],
                        "primitive": "image",
                    },
                )

                (depth_image, _) = reachy.cameras.depth.get_frame()

                node.send_output(
                    "image_depth",
                    pa.array(depth_image.ravel()),
                    metadata={
                        "encoding": "bgr8",
                        "width": depth_image.shape[1],
                        "height": depth_image.shape[0],
                        "primitive": "image",
                    },
                )

                (depth_frame, _) = reachy.cameras.depth.get_depth_frame()

                if params is not None and depth_frame is not None:
                    depth_frame = depth_frame.ravel().astype(np.float64) / 1_000.0

                    node.send_output(
                        "depth",
                        pa.array(depth_frame),
                        metadata={
                            "width": width,
                            "height": height,
                            "focal": [int(k[0, 0]), int(k[1, 1])],
                            "resolution": [int(k[0, 2]), int(k[1, 2])],
                            "primitive": "depth",
                        },
                    )


if __name__ == "__main__":
    main()
