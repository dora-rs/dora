import os
import time

import numpy as np
from dora import Node
from reachy_sdk import ReachySDK
from reachy_sdk.trajectory import goto


def r_arm_inverse_kinematics(reachy, pose, action) -> list:
    A = np.array(
        [
            [0, 0, -1, pose[0] + action[0]],
            [0, 1, 0, pose[1] + action[1]],
            [1, 0, 0, pose[2] + action[2]],
            [0, 0, 0, 1],
        ]
    )
    return reachy.r_arm.inverse_kinematics(A)


def happy_antennas(reachy):
    reachy.head.l_antenna.speed_limit = 480.0
    reachy.head.r_antenna.speed_limit = 480.0

    for _ in range(1):
        reachy.head.l_antenna.goal_position = 10.0
        reachy.head.r_antenna.goal_position = -10.0

        time.sleep(0.1)

        reachy.head.l_antenna.goal_position = -10.0
        reachy.head.r_antenna.goal_position = 10.0

        time.sleep(0.1)

    reachy.head.l_antenna.goal_position = 0.0
    reachy.head.r_antenna.goal_position = 0.0


def sad_antennas(reachy):
    reachy.head.l_antenna.speed_limit = 360.0
    reachy.head.r_antenna.speed_limit = 360.0

    reachy.head.l_antenna.goal_position = 140.0
    reachy.head.r_antenna.goal_position = -140.0


def main():
    node = Node()

    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.24")

    reachy = ReachySDK(ROBOT_IP, with_mobile_base=False)
    reachy.turn_on("r_arm")
    reachy.turn_on("head")

    r_arm_pose = [0.2, -0.46, -0.42]
    head_pose = [
        reachy.head.neck_roll.present_position,
        reachy.head.neck_yaw.present_position,
        reachy.head.neck_pitch.present_position,
    ]

    default_pose = r_arm_inverse_kinematics(reachy, r_arm_pose, [0, 0, 0])

    goto(
        {joint: pos for joint, pos in zip(reachy.r_arm.joints.values(), default_pose)},
        duration=3,
    )

    for event in node:
        if event["type"] != "INPUT":
            continue

        if event["id"] == "r_arm_action":
            action = event["value"].to_pylist()
            joint_pose = r_arm_inverse_kinematics(reachy, r_arm_pose, action)
            goto(
                {
                    joint: pos
                    for joint, pos in zip(reachy.r_arm.joints.values(), joint_pose)
                },
                duration=0.200,
            )
            r_arm_pose = np.array(r_arm_pose) + np.array(action)
        elif event["id"] == "head_action":
            action = event["value"].to_pylist()
            for i in range(5):
                head_pose = np.array(head_pose) + np.array(action) / 5
                reachy.head.neck_roll.goal_position = head_pose[0]
                reachy.head.neck_yaw.goal_position = head_pose[1]
                reachy.head.neck_pitch.goal_position = head_pose[2]
                time.sleep(0.03)
        elif event["id"] == "antenna_action":
            text = event["value"].to_pylist()[0]
            if text == "smile":
                happy_antennas(reachy)
            elif text == "cry":
                sad_antennas(reachy)
        elif event["id"] == "gripper_action":
            action = event["value"].to_pylist()[0]
            step = (action - reachy.joints.r_gripper.present_position) / 10
            goal = reachy.joints.r_gripper.present_position
            for i in range(10):
                goal += step
                goal = np.clip(goal, -100, 100)
                reachy.joints.r_gripper.goal_position = goal
                time.sleep(0.02)

            # When opening the gripper always go to default pose
            if action == -100:
                goto(
                    {
                        joint: pos
                        for joint, pos in zip(
                            reachy.r_arm.joints.values(), default_pose
                        )
                    },
                    duration=3,
                )
                r_arm_pose = [0.2, -0.46, -0.42]
    reachy.turn_off_smoothly("r_arm")
    reachy.turn_off_smoothly("head")
