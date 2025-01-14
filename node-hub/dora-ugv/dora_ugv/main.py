try:
    import ugv_sdk_py
    from ugv_sdk_py import hunter_robot
except ImportError as err:
    print(
        "Please install ugv_sdk_py first by following the instruction at: https://github.com/westonrobot/ugv_sdk/tree/main?tab=readme-ov-file#build-the-package-as-a-python-package"
    )
    raise err

import os

import pyarrow as pa
from dora import Node


def main():
    # Create an instance of HunterRobot
    robot = hunter_robot.HunterRobot(ugv_sdk_py.ProtocolVersion.AGX_V2)

    # Connect to the robot
    can = os.getenv("CAN_BUS", "can0")
    robot.connect(can)
    print("Connection to the robot established.")

    print("protocol version: ", robot.get_parser_protocol_version())
    print("robot version: ", robot.request_version())

    # Enable commanded mode
    robot.enable_commanded_mode()
    node = Node()
    # Set motion command
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "action":
                lin_vel, ang_vel = event["value"].to_numpy()
                robot.set_motion_command(lin_vel, ang_vel)
            else:
                state = robot.get_robot_state()
                node.send_output(
                    "velocity",
                    pa.array(
                        [
                            state.motion_state.linear_velocity,
                            state.motion_state.angular_velocity,
                        ]
                    ),
                )


if __name__ == "__main__":
    main()
