"""TODO: Add docstring."""

import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK


def main():
    """TODO: Add docstring."""
    robot_ip = os.getenv("ROBOT_IP", "127.0.0.1")

    reachy = ReachySDK(robot_ip)

    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_on()
    reachy.mobile_base.reset_odometry()

    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "action_base":
                [x, y, _z, _rx, _ry, rz] = event["value"].to_numpy()
                reachy.mobile_base.set_goal_speed(vx=x, vy=y, vtheta=np.rad2deg(rz) * 1.5)
                reachy.mobile_base.send_speed_command()
                node.send_output("response_base", pa.array([True]))
    if reachy.mobile_base is not None:
        reachy.mobile_base.turn_off()


if __name__ == "__main__":
    main()
