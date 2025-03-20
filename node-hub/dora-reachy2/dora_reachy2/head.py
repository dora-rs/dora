"""TODO: Add docstring."""

import os

import numpy as np
from dora import Node
from reachy2_sdk import ReachySDK


def main():
    """TODO: Add docstring."""
    robot_ip = os.getenv("ROBOT_IP", "10.42.0.80")

    for _i in range(5):
        reachy = ReachySDK(robot_ip)

        if reachy.head is not None:
            reachy.head.turn_on()
            reachy.head.goto([0, 0, 0])
            break
    fov_h = 107
    fov_v = 91
    resolution = [720, 960]

    roll, _pitch, yaw = reachy.head.get_current_positions()
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "boxes2d":
                boxes = event["value"].to_numpy()
                # [x, y, z, _rx, ry, rz] = event["value"].to_numpy()
                x_min = boxes[0]
                y_min = boxes[1]
                x_max = boxes[2]
                y_max = boxes[3]
                x = (
                    x_min + x_max
                ) / 2 - 10  # Deviate a bit to take into account the off centered camera
                y = (3 * y_min + y_max) / 4
                ry = (x - resolution[1] / 2) * fov_h / 2 / resolution[1]
                rz = (y - resolution[0] / 2) * fov_v / 2 / resolution[0]
                if np.abs(yaw) > 45 and yaw * -ry > 0:
                    reachy.head.cancel_all_goto()
                    roll, _pitch, yaw = reachy.head.get_current_positions()
                    reachy.head.rotate_by(pitch=0, yaw=0, roll=-roll, duration=1.5)
                else:
                    reachy.head.cancel_all_goto()
                    roll, _pitch, yaw = reachy.head.get_current_positions()
                    reachy.head.rotate_by(pitch=rz, yaw=-ry, roll=-roll, duration=1.5)
                [roll, _pitch, yaw] = reachy.head.get_current_positions()
            if "look" in event["id"]:
                [x, y, z] = event["value"].to_numpy()
                reachy.head.cancel_all_goto()
                reachy.head.look_at(x, y, z, duration=0.5)

    if reachy.head is not None:
        reachy.head.turn_off()


if __name__ == "__main__":
    main()
