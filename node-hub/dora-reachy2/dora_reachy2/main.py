import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView


def main():
    freq = 30

    ROBOT_IP = "172.17.134.85"
    # ROBOT_IP = "localhost"

    reachy = ReachySDK(ROBOT_IP)

    SIMULATION = False

    reachy.turn_on()

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
