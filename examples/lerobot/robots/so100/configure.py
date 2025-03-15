"""
SO100 Auto Configure: This program is used to automatically configure the SO-ARM100 (SO100) for the user.

The program will:
1. Disable all torque motors of provided SO100.
2. Ask the user to move the SO100 to the position 1 (see CONFIGURING.md for more details).
3. Record the position of the SO100.
4. Ask the user to move the SO100 to the position 2 (see CONFIGURING.md for more details).
5. Record the position of the SO100.
8. Calculate the offset and inverted mode of the SO100.
9. Let the user verify in real time that the SO100 is working properly.

It will also enable all appropriate operating modes for the SO100.
"""

import argparse
import time
import json

import pyarrow as pa

from bus import FeetechBus, TorqueMode, OperatingMode
from pwm_position_control.transform import pwm_to_logical_arrow, wrap_joints_and_values

from pwm_position_control.tables import (
    construct_logical_to_pwm_conversion_table_arrow,
    construct_pwm_to_logical_conversion_table_arrow,
)

from pwm_position_control.functions import construct_control_table

FULL_ARM = pa.array(
    [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ],
    type=pa.string(),
)

ARM_WITHOUT_GRIPPER = pa.array(
    ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"],
    type=pa.string(),
)

GRIPPER = pa.array(["gripper"], type=pa.string())


def pause():
    input("Press Enter to continue...")


def configure_servos(bus: FeetechBus):
    bus.write_torque_enable(
        wrap_joints_and_values(FULL_ARM, [TorqueMode.DISABLED.value] * 6)
    )

    bus.write_operating_mode(
        wrap_joints_and_values(FULL_ARM, [OperatingMode.ONE_TURN.value] * 6)
    )

    bus.write_max_angle_limit(
        wrap_joints_and_values(FULL_ARM, [pa.scalar(0, pa.uint32())] * 6)
    )

    bus.write_min_angle_limit(
        wrap_joints_and_values(FULL_ARM, [pa.scalar(0, pa.uint32())] * 6)
    )


def main():
    parser = argparse.ArgumentParser(
        description="SO100 Auto Configure: This program is used to automatically configure the Low Cost Robot (SO100) "
        "for the user."
    )

    parser.add_argument(
        "--port", type=str, required=True, help="The port of the SO100."
    )
    parser.add_argument(
        "--right",
        action="store_true",
        help="If the SO100 is on the right side of the user.",
    )
    parser.add_argument(
        "--left",
        action="store_true",
        help="If the SO100 is on the left side of the user.",
    )

    args = parser.parse_args()

    if args.right and args.left:
        raise ValueError("You cannot specify both --right and --left.")

    args = parser.parse_args()

    targets = (
        wrap_joints_and_values(FULL_ARM, [0, -90, 90, 0, -90, 0]),
        wrap_joints_and_values(FULL_ARM, [90, 0, 0, 90, 0, -90]),
    )

    arm = FeetechBus(
        args.port,
        {
            "shoulder_pan": (1, "st3215"),
            "shoulder_lift": (2, "st3215"),
            "elbow_flex": (3, "st3215"),
            "wrist_flex": (4, "st3215"),
            "wrist_roll": (5, "st3215"),
            "gripper": (6, "st3215"),
        },
    )

    configure_servos(arm)

    print("Please move the SO100 to the first position.")
    pause()
    pwm_position_1 = arm.read_position(FULL_ARM)["values"].values

    print("Please move the SO100 to the second position.")
    pause()
    pwm_position_2 = arm.read_position(FULL_ARM)["values"].values

    print("Configuration completed.")

    pwm_positions = (pwm_position_1, pwm_position_2)

    pwm_to_logical_conversion_table = construct_pwm_to_logical_conversion_table_arrow(
        pwm_positions, targets
    )
    logical_to_pwm_conversion_table = construct_logical_to_pwm_conversion_table_arrow(
        pwm_positions, targets
    )

    control_table_json = {}
    for i in range(len(FULL_ARM)):
        control_table_json[FULL_ARM[i].as_py()] = {
            "id": i + 1,
            "model": "sts3215",
            "torque": True,
            "pwm_to_logical": pwm_to_logical_conversion_table[FULL_ARM[i].as_py()],
            "logical_to_pwm": logical_to_pwm_conversion_table[FULL_ARM[i].as_py()],
        }

    left = "left" if args.left else "right"
    path = (
        input(
            f"Please enter the path of the configuration file (default is ./robots/so100/configs/follower.{left}.json): "
        )
        or f"./robots/so100/configs/follower.{left}.json"
    )

    with open(path, "w") as file:
        json.dump(control_table_json, file)

    control_table = construct_control_table(
        pwm_to_logical_conversion_table, logical_to_pwm_conversion_table
    )

    while True:
        try:
            pwm_position = arm.read_position(FULL_ARM)
            logical_position = pwm_to_logical_arrow(
                pwm_position, control_table, ranged=True
            ).field("values")

            print(f"Logical Position: {logical_position}")

        except ConnectionError:
            print(
                "Connection error occurred. Please check the connection and try again."
            )

        time.sleep(0.5)


if __name__ == "__main__":
    main()
