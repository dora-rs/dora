import os

import numpy as np
from dora import Node
from reachy2_sdk import ReachySDK


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")

    reachy = ReachySDK(ROBOT_IP)

    reachy.turn_on()
    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_on()
    if reachy.head is not None:
        reachy.head.turn_on()
        reachy.head.goto([0, 0, 0])

    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "action_base":
                [x, y, _z, _rx, _ry, rz] = event["value"].to_numpy()
                reachy.mobile_base.rotate_by(np.rad2deg(rz))
                reachy.mobile_base.translate_by(x, y)

    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_off()

    if reachy.head is not None:
        reachy.head.turn_off()


if __name__ == "__main__":
    main()
