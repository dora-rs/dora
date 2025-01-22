import os

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "172.17.134.85")

    reachy = ReachySDK(ROBOT_IP)

    reachy.turn_on()

    params = reachy.cameras.depth.get_parameters(CameraView.DEPTH)

    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "tick":
                (image_left, _) = reachy.cameras.teleop.get_frame(view=CameraView.LEFT)
                node.send_output(
                    "image_left",
                    pa.array(image_left.ravel()),
                    metadata={
                        "encoding": "bgr8",
                        "width": image_left.shape[1],
                        "height": image_left.shape[0],
                    },
                )

                (image_right, _) = reachy.cameras.teleop.get_frame(
                    view=CameraView.RIGHT
                )

                node.send_output(
                    "image_right",
                    pa.array(image_right.ravel()),
                    metadata={
                        "encoding": "bgr8",
                        "width": image_right.shape[1],
                        "height": image_right.shape[0],
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
                    },
                )

                (depth_frame, _) = reachy.cameras.depth.get_depth_frame()

                if params is not None and depth_frame is not None:
                    height, width, _distortion_model, _D, K, _R, _P = params
                    depth_frame = depth_frame.ravel().astype(np.float64()) / 1_000.0

                    node.send_output(
                        "depth",
                        pa.array(depth_frame),
                        metadata={
                            "width": width,
                            "height": height,
                            "focal": [int(K[0, 0]), int(K[1, 1])],
                            "resolution": [int(K[0, 2]), int(K[1, 2])],
                        },
                    )

                position = reachy.mobile_base.get_current_odometry()
                position = [
                    position["x"],
                    position["y"],
                    0,
                    0,
                    0,
                    position["theta"],
                ]
                node.send_output("position", pa.array(position), metadata={})

            elif event["id"] == "action_base":
                # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                [x, y, _z, _rx, _ry, rz] = event["value"].to_numpy()
                reachy.mobile_base.rotate_by(np.rad2deg(rz))
                reachy.mobile_base.translate_by(x, y)

            elif event["id"] == "action_head":
                # Warning: Make sure to add my_output_id and my_input_id within the dataflow.
                [x, y, _z, _rx, _ry, rz] = event["value"].to_numpy()
                reachy.mobile_base.rotate_by(np.rad2deg(rz))
                reachy.mobile_base.translate_by(x, y)
    reachy.turn_off_smoothly()


if __name__ == "__main__":
    main()
