"""
LCR Configuration Tool: This program is used to automatically configure the Low Cost Robot (LCR) for the user.

The program will:
1. Disable all torque motors of provided LCR.
2. Ask the user to move the LCR to the position 1 (see CONFIGURING.md for more details).
3. Record the position of the LCR.
4. Ask the user to move the LCR to the position 2 (see CONFIGURING.md for more details).
5. Record the position of the LCR.
8. Calculate interpolation functions.
9. Let the user verify in real time that the LCR is working properly.

It will also enable all appropriate operating modes for the LCR.
"""

import argparse
import time
import json

import pyarrow as pa

from bus import DynamixelBus, TorqueMode, OperatingMode

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


def configure_servos(bus: DynamixelBus):
    bus.write_torque_enable(
        wrap_joints_and_values(FULL_ARM, [TorqueMode.DISABLED.value] * 6)
    )

    bus.write_operating_mode(
        wrap_joints_and_values(
            ARM_WITHOUT_GRIPPER, [OperatingMode.EXTENDED_POSITION.value] * 5
        )
    )

    bus.write_operating_mode(
        wrap_joints_and_values(
            GRIPPER, [OperatingMode.CURRENT_CONTROLLED_POSITION.value]
        )
    )


def main():
    parser = argparse.ArgumentParser(
        description="LCR Auto Configure: This program is used to automatically configure the Low Cost Robot (LCR) for "
        "the user."
    )

    parser.add_argument("--port", type=str, required=True, help="The port of the LCR.")
    parser.add_argument(
        "--right",
        action="store_true",
        help="If the LCR is on the right side of the user.",
    )
    parser.add_argument(
        "--left",
        action="store_true",
        help="If the LCR is on the left side of the user.",
    )
    parser.add_argument(
        "--follower",
        action="store_true",
        help="If the LCR is the follower of the user.",
    )
    parser.add_argument(
        "--leader", action="store_true", help="If the LCR is the leader of the user."
    )

    args = parser.parse_args()

    if args.right and args.left:
        raise ValueError("You cannot specify both --right and --left.")

    if args.follower and args.leader:
        raise ValueError("You cannot specify both --follower and --leader.")

    targets = (
        wrap_joints_and_values(FULL_ARM, [0, -90, 90, 0, -90, 0]),
        wrap_joints_and_values(FULL_ARM, [90, 0, 0, 90, 0, -90]),
    )

    arm = DynamixelBus(
        args.port,
        {
            "shoulder_pan": (1, "x_series"),
            "shoulder_lift": (2, "x_series"),
            "elbow_flex": (3, "x_series"),
            "wrist_flex": (4, "x_series"),
            "wrist_roll": (5, "x_series"),
            "gripper": (6, "x_series"),
        },
    )

    configure_servos(arm)

    print("Please move the LCR to the first position.")
    pause()
    pwm_position_1 = arm.read_position(FULL_ARM)

    print("Please move the LCR to the second position.")
    pause()
    pwm_position_2 = arm.read_position(FULL_ARM)

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
        model = (
            "xl430-w250"
            if i <= 1 and args.follower
            else "xl330-m288" if args.follower else "xl330-m077"
        )

        control_table_json[FULL_ARM[i].as_py()] = {
            "id": i + 1,
            "model": model,
            "torque": (
                True if args.follower else True if args.leader and i == 5 else False
            ),
            "goal_current": (
                500
                if args.follower and i == 5
                else 40 if args.leader and i == 5 else None
            ),
            "goal_position": -40.0 if args.leader and i == 5 else None,
            "pwm_to_logical": pwm_to_logical_conversion_table[FULL_ARM[i].as_py()],
            "logical_to_pwm": logical_to_pwm_conversion_table[FULL_ARM[i].as_py()],
            "P": (
                640
                if model == "xl430-w250"
                else 1500 if model == "xl330-m288" and i != 5 else 250
            ),
            "I": 0,
            "D": 3600 if model == "xl430-w250" else 600,
        }

    left = "left" if args.left else "right"
    leader = "leader" if args.leader else "follower"

    path = (
        input(
            f"Please enter the path of the configuration file (default is ./robots/alexk-lcr/configs/{leader}.{left}.json): "
        )
        or f"./robots/alexk-lcr/configs/{leader}.{left}.json"
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
