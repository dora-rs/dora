import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from scipy.spatial.transform import Rotation as R

init_pos = [
    -5.60273587426976,
    10.780818397272316,
    -27.868146823156042,
    -126.15650363072193,
    3.961108018106834,
    -35.43682799906162,
    350.9236448374495,
]

release_pos = [
    -27.671138554283488,
    12.227397821967656,
    -1.7328297713245189,
    -98.23576947359867,
    0.8473595116515701,
    29.972034625269437,
    0.5821498829257334,
]
ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()


def go_to_mixed_angles(reachy, x, y, z, no_angle=False):
    for theta in range(-80, -50, 10):
        if no_angle:
            theta = -90
        r = R.from_euler("zyx", [0, theta, 0], degrees=True)
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.r_arm.inverse_kinematics(transform)

        except ValueError:
            continue

    for yaw in range(0, 30, 10):

        ## First try turning left
        pitch = -90
        roll = 0
        r = R.from_euler("ZYX", (yaw, 0, 0), degrees=True) * R.from_euler(
            "ZYX", (0, pitch, 0), degrees=True
        )
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.r_arm.inverse_kinematics(transform)
        except ValueError:
            pass

        try:
            return reachy.r_arm.inverse_kinematics(transform)

        except ValueError:
            continue
    print("No solution found for x, y, z: ", x, y, z)
    return []
    # for theta in range(-90, 0):
    #     cos = np.cos(np.deg2rad(theta))
    #     sin = np.sin(np.deg2rad(theta))
    #     a = np.array(
    #         [
    #             [sin, 0.0, -cos, -x],
    #             [0.0, 1.0, 0.0, -y],
    #             [cos, 0.0, sin, 0.2],
    #             [0.0, 0.0, 0.0, 1.0],
    #         ]
    #     )
    #     try:
    #         reachy.r_arm.goto(reachy.r_arm.inverse_kinematics(a))
    #         a = np.array(
    #             [
    #                 [sin, 0.0, -cos, -x],
    #                 [0.0, 1.0, 0.0, -y],
    #                 [cos, 0.0, sin, -0.4],
    #                 [0.0, 0.0, 0.0, 1.0],
    #             ]
    #         )
    #         reachy.r_arm.goto(reachy.r_arm.inverse_kinematics(a))
    #         break
    #     except ValueError:
    #         pass


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")

    reachy = ReachySDK(ROBOT_IP)

    if reachy.r_arm is not None:
        reachy.r_arm.turn_on()
        reachy.r_arm.goto(init_pos)
        reachy.r_arm.gripper.turn_on()
    node = Node()
    grabbed = False

    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "pose":
                if grabbed:
                    continue
                else:
                    node.send_output("pause", pa.array([False]))
                    reachy.r_arm.gripper.open()
                values: np.array = event["value"].to_numpy()
                x = values[0]
                y = values[1]
                if y >= 0:
                    continue
                z = values[2]
                node.send_output("look", pa.array([x, y, z]))
                z = np.clip(z + 0.01, -0.32, -0.22)

                joint_upper = go_to_mixed_angles(reachy, x + 0.06, y + 0.03, -0.16)
                joint_lower = go_to_mixed_angles(reachy, x + 0.06, y + 0.03, z)

                if len(joint_upper) and len(joint_lower):
                    node.send_output("pause", pa.array([True]))
                    reachy.r_arm.goto(joint_upper, duration=0.75, wait=True)
                    reachy.r_arm.goto(joint_lower, duration=0.75, wait=True)
                    reachy.r_arm.gripper.close()
                    time.sleep(0.5)
                    if (
                        reachy.r_arm.gripper.get_current_opening() > 2
                        and reachy.r_arm.gripper.get_current_opening() < 97
                    ):
                        reachy.r_arm.goto(joint_upper, duration=0.75, wait=True)
                        if (
                            reachy.r_arm.gripper.get_current_opening() > 2
                            and reachy.r_arm.gripper.get_current_opening() < 97
                        ):
                            grabbed = True
                            node.send_output("look", pa.array([0.3, 0, 0.0]))
                            node.send_output("rotate", pa.array(["right"]))
                            reachy.r_arm.goto(release_pos, duration=1.5, wait=False)
                            continue
                    node.send_output("pause", pa.array([False]))
                reachy.r_arm.goto(init_pos, duration=1.3, wait=True)
                time.sleep(0.4)

            if "action_arm" in event["id"]:
                text = event["value"][0].as_py()
                if reachy.r_arm.is_off():
                    reachy.r_arm.turn_on()
                    reachy.r_arm.gripper.turn_on()
                if text == "cancel":
                    reachy.r_arm.cancel_all_goto()
                if text == "release":
                    time.sleep(0.5)
                    reachy.r_arm.gripper.open()
                    reachy.r_arm.goto(init_pos, duration=1)
                if text == "wait":
                    pass

    # reachy.r_arm.turn_off_smoothly()


if __name__ == "__main__":
    main()

import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView
from scipy.spatial.transform import Rotation as R

ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")

reachy = ReachySDK(ROBOT_IP)
# reachy.r_arm.turn_on()

# x = 0.25
# y = -0.0

# yaw = 30
# pitch = -90

# r = R.from_euler("ZYX", (yaw, 0, 0), degrees=True) * R.from_euler(
#     "ZYX", (0, pitch, 0), degrees=True
# )
# transform = np.eye(4)
# transform[:3, :3] = r.as_matrix()
# transform[:3, 3] = [x, y, -0.32]

# a = reachy.r_arm.inverse_kinematics(transform)
# reachy.r_arm.goto(a)
# 1th is front-back: go from 0.5 to 0.3
# 2th is right-left: go from -0.5 to -0.03
# 3th s height -0.3 and -0.4 but does not seem to work going high
