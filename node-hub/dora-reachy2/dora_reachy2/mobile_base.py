"""TODO: Add docstring."""

import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK


def main():
    """TODO: Add docstring."""
    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")

    reachy = ReachySDK(ROBOT_IP)

    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_on()
    reachy.mobile_base.reset_odometry()

    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "action_base":
                [x, y, _z, _rx, _ry, rz] = event["value"].to_numpy()
                reachy.mobile_base.rotate_by(np.rad2deg(rz))
                reachy.mobile_base.translate_by(x, y)
                time.sleep(0.5)
                node.send_output("response_base", pa.array([True]))
    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_off()


if __name__ == "__main__":
    main()
