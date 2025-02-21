import os

import numpy as np
from dora import Node
from reachy2_sdk import ReachySDK


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")

    for _i in range(5):
        reachy = ReachySDK(ROBOT_IP)

        if reachy.head is not None:
            reachy.head.turn_on()
            reachy.head.goto([0, 0, 0])
            break
    FOV_H = 107
    FOV_V = 91
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
                ry = (x - resolution[1] / 2) * FOV_H / 2 / resolution[1]
                rz = (y - resolution[0] / 2) * FOV_V / 2 / resolution[0]
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
