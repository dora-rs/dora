import os

from dora import Node
from reachy2_sdk import ReachySDK


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")

    reachy = ReachySDK(ROBOT_IP)
    FOV_H = 107
    FOV_V = 91
    resolution = [720, 960]

    if reachy.head is not None:
        reachy.head.turn_on()
        reachy.head.goto([0, 0, 0])

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
                ) / 2 - 10  # Deviate a bit to take into accound the off centered camera
                y = (3 * y_min + y_max) / 4
                ry = (x - resolution[1] / 2) * FOV_H / 2 / resolution[1]
                rz = (y - resolution[0] / 2) * FOV_V / 2 / resolution[0]
                reachy.head.rotate_by(pitch=rz, yaw=-ry, wait=True)
                [r, p, y] = reachy.head.get_current_positions()
                if r != 0:
                    reachy.head.rotate_by(roll=-r, wait=True)

    if reachy.head is not None:
        reachy.head.turn_off()


if __name__ == "__main__":
    main()
