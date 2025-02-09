import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")

    reachy = ReachySDK(ROBOT_IP)

    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_on()
    reachy.mobile_base.reset_odometry()
    grabbing = False

    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "action_base":
                [x, y, _z, _rx, _ry, rz] = event["value"].to_numpy()
                reachy.mobile_base.rotate_by(np.rad2deg(rz))
                reachy.mobile_base.translate_by(x, y)
            if event["id"] == "rotate":
                text = event["value"][0].as_py()
                if text == "right":
                    if grabbing:
                        node.send_output("pause", pa.array([True]))
                        # node.send_output("text_ts", pa.array([""]))
                        reachy.mobile_base.goto(0, 0, 0)
                        node.send_output("action_arm", pa.array(["release"]))
                        grabbing = False
                elif text == "left":
                    if not grabbing:
                        reachy.mobile_base.goto(-0.4, 0, 90)
                        grabbing = True
                    node.send_output("pause", pa.array([False]))
    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_off()


if __name__ == "__main__":
    main()
