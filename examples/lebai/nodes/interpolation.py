"""Module for interpolating between robot configurations.

This module provides functionality for creating smooth transitions between
robot configurations and movements.
"""

from enum import Enum

import pyarrow as pa
from dora import Node

node = Node()


class Action(Enum):
    """Action abstraction for robot movement commands."""

    YAW_RIGHT = ("yaw right", "movej", [0, 0, 0, 0, -0.1, 0, 0.1])
    YAW_LEFT = ("yaw left", "movej", [0, 0, 0, 0, 0.1, 0, 0.1])
    PITCH_UP = ("pitch up", "movej", [0, 0, 0, -0.1, 0, 0, 0.1])
    PITCH_DOWN = ("pitch down", "movej", [0, 0, 0.1, 0, 0, 0, 0.1])
    ROLL_LEFT = ("roll left", "movej", [0, 0, 0, 0, 0, 0.1, 0.1])
    ROLL_RIGHT = ("roll right", "movej", [0, 0, 0, 0, 0, -0.1, 0.1])
    YAW_SHOULDER_RIGHT = (
        "yaw shoulder right",
        "movej",
        [-0.1, 0, 0, 0, 0, 0, 0.1],
    )
    YAW_SHOULDER_LEFT = (
        "yaw shoulder left",
        "movej",
        [0.1, 0, 0, 0, 0, 0, 0.1],
    )
    FORWARD = ("forward", "movec", [-0.02, 0, 0, 0, 0, 0, 0.1])
    BACK = ("back", "movec", [0.02, 0, 0, 0, 0, 0, 0.1])
    LEFT = ("left", "movec", [0, -0.02, 0, 0, 0, 0, 0.1])
    RIGHT = ("right", "movec", [0, 0.02, 0, 0, 0, 0, 0.1])
    UP = ("up", "movec", [0, 0, 0.02, 0, 0, 0, 0.1])
    DOWN = ("down", "movec", [0, 0, -0.02, 0, 0, 0, 0.1])
    CLOSE = ("close", "claw", [0])
    OPEN = ("open", "claw", [100])
    STOP = ("stop", "stop", [])
    SAVE = ("save", "save", [])
    GO_TO = ("go", "go_to", [])
    END_TEACH = ("end of teach", "end_teach", [])
    TEACH = ("teach", "teach", [])


for event in node:
    if event["type"] == "INPUT":
        text = event["value"][0].as_py().lower()
        text = text.replace(".", "")
        text = text.replace(".", "")

        if "save" in text:
            node.send_output("save", pa.array([text.replace("save ", "")]))
        elif "go" in text:
            node.send_output("go_to", pa.array([text.replace("go ", "")]))
        elif "go to " in text:
            node.send_output("go_to", pa.array([text.replace("go to ", "")]))
        else:
            for action in Action:
                if action.value[0] in text:
                    node.send_output(action.value[1], pa.array(action.value[2]))
                    break
