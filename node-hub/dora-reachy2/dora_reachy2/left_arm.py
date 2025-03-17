"""TODO: Add docstring."""

import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from scipy.spatial.transform import Rotation as R

ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")


l_default_pose = [
    42.212611297240635,
    -16.95827541661092,
    15.241872783848812,
    -131.11770715700908,
    0.1682905250638251,
    -1.6613469324618695,
    2.1666679127563904,
]


def l_arm_go_to_mixed_angles(reachy, x, y, z):
    """TODO: Add docstring."""
    for theta in range(-80, -60, 10):
        r = R.from_euler("zyx", [0, theta, 0], degrees=True)
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.l_arm.inverse_kinematics(transform)

        except ValueError:
            continue

    for yaw in range(0, 90, 30):

        ## First try turning left
        pitch = -90
        r = R.from_euler("ZYX", (-yaw, 0, 0), degrees=True) * R.from_euler(
            "ZYX",
            (0, pitch, 0),
            degrees=True,
        )
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.l_arm.inverse_kinematics(transform)
        except ValueError:
            pass

        try:
            return reachy.l_arm.inverse_kinematics(transform)

        except ValueError:
            continue

    # Fallback to default pose if we need the arm to be within x and z limits.
    if x < 0.3 and z > -0.2:
        return l_default_pose

    print("Left arm: No solution found for x, y, z: ", x, y, z)
    return []


def manage_gripper(reachy, gripper, grasp):
    """TODO docstring."""
    if (gripper == 100 and reachy.r_arm.gripper.get_current_opening() == 100) or (
        gripper == 0.0
        and (
            reachy.r_arm.gripper.get_current_opening() < 98
            and reachy.r_arm.gripper.get_current_opening() > 2
        )
        and grasp
    ):
        return True
    if gripper == 0.0:
        reachy.l_arm.gripper.close()
        time.sleep(0.3)
        if grasp:
            half_open = reachy.l_arm.gripper.get_current_opening() > 2
            if not half_open:
                return False
    elif gripper == 100.0:
        reachy.l_arm.gripper.open()
        time.sleep(0.3)
    return True


def main():
    """TODO: Add docstring."""
    reachy = ReachySDK(ROBOT_IP)

    if reachy.l_arm is not None:
        reachy.l_arm.turn_on()
        reachy.l_arm.gripper.turn_on()
    node = Node()

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "pose":

                values: np.array = event["value"].to_numpy(zero_copy_only=False)
                encoding = event["metadata"]["encoding"]
                wait = event["metadata"].get("wait", True)
                duration = float(event["metadata"].get("duration", 1))
                grasp = event["metadata"].get("grasp", True)
                if encoding == "xyzrpy":
                    values = values.reshape((-1, 7))
                    joint_values = []
                    for value in values:
                        x = value[0]
                        y = value[1]
                        z = value[2]
                        _r = value[3]
                        _p = value[4]
                        _y = value[5]
                        gripper = value[6]
                        joints = l_arm_go_to_mixed_angles(reachy, x, y, z)
                        response_ik = len(joints) > 0
                        if response_ik:
                            joint_values.append((joints, gripper))
                        else:

                            break

                    if not response_ik:
                        node.send_output(
                            "response_l_arm",
                            pa.array([False]),
                            metadata={"error": f"IK Failed for x: {x}, y: {y}, z: {z}"},
                        )
                    else:
                        for joint, gripper in joint_values:
                            reachy.l_arm.goto(
                                joint,
                                duration=duration,
                                wait=wait,
                                interpolation_mode="linear",
                            )
                            response_gripper = manage_gripper(reachy, gripper, grasp)
                            if not response_gripper:
                                node.send_output(
                                    "response_l_arm",
                                    pa.array([False]),
                                    metadata={"error": "Failed to grasp"},
                                )
                                break
                        if response_gripper:
                            node.send_output("response_l_arm", pa.array([True]))

                elif encoding == "jointstate":
                    values = values.reshape((-1, 8))
                    for value in values:
                        joints = value[:7].tolist()
                        gripper = value[7]

                        reachy.l_arm.goto(
                            joints,
                            duration=duration,
                            wait=wait,
                            interpolation_mode="linear",
                        )
                        manage_gripper(reachy, gripper, grasp)
                    node.send_output("response_l_arm", pa.array([True]))


if __name__ == "__main__":
    main()
