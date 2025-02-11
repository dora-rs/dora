import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from scipy.spatial.transform import Rotation as R

ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()

l_init_pos = [
    -7.0631310641087435,
    -10.432298603362307,
    24.429809104404114,
    -132.15000828778648,
    -1.5494749438811133,
    -21.749917789205202,
    8.099312596108344,
]
l_release_pos = [
    -30.04330081906935,
    -7.415231584691132,
    3.6972339048071468,
    -97.7274736257555,
    12.996718740452982,
    30.838020649757016,
    -1.5572310505704858,
]
r_init_pos = [
    -5.60273587426976,
    10.780818397272316,
    -27.868146823156042,
    -126.15650363072193,
    3.961108018106834,
    -35.43682799906162,
    350.9236448374495,
]
r_release_pos = [
    -26.1507947940993,
    12.16735021387949,
    -2.2657319092611976,
    -97.63648867582175,
    -19.91084837404425,
    22.10184328619011,
    366.71351223614494,
]

r_default_pos = [
    38.058172640242475,
    0.07798708660299236,
    2.0084781702579564,
    -129.76629958820868,
    4.428130313456095,
    -9.272674208719419,
    354.280491569214,
]

l_default_pos = [
    42.212611297240635,
    -16.95827541661092,
    15.241872783848812,
    -131.11770715700908,
    0.1682905250638251,
    -1.6613469324618695,
    2.1666679127563904,
]

hand_wave_pos1 = [
    -34.9845055514667,
    -4.256103461472862,
    37.1917528987426,
    -132.63047111476183,
    42.11556113932392,
    5.328614342780937,
    28.77740038848836,
]
hand_wave_pos2 = [
    -39.50665085752708,
    4.08569792564386,
    -17.53783575070672,
    -132.6317825110825,
    -17.97474587643951,
    -3.758287258250598,
    -31.217505960042924,
]


def l_arm_go_to_mixed_angles(reachy, x, y, z, no_angle=False):
    for theta in range(-80, -50, 10):
        if no_angle:
            theta = -90
        r = R.from_euler("zyx", [0, theta, 0], degrees=True)
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.l_arm.inverse_kinematics(transform)

        except ValueError:
            continue
    for yaw in range(0, 90, 30):

        ## Then try turning right
        pitch = -90
        r = R.from_euler("ZYX", (-yaw, 0, 0), degrees=True) * R.from_euler(
            "ZYX", (0, pitch, 0), degrees=True
        )
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.l_arm.inverse_kinematics(transform)

        except ValueError:
            continue
    print("Left arm: No solution found for x, y, z: ", x, y, z)
    return []


def r_arm_go_to_mixed_angles(reachy, x, y, z, no_angle=False):
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

    for yaw in range(0, 90, 30):

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
    print("Right arm: No solution found for x, y, z: ", x, y, z)
    return []


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")
    reachy = ReachySDK(ROBOT_IP)
    last_handwave = time.time()

    if reachy.l_arm is not None:
        reachy.l_arm.turn_on()
        reachy.l_arm.goto(l_init_pos)
        reachy.l_arm.gripper.turn_on()

    if reachy.r_arm is not None:
        reachy.r_arm.turn_on()
        reachy.r_arm.goto(r_init_pos)
        reachy.r_arm.gripper.turn_on()
    node = Node()
    grabbed = False
    started = False
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "pose":
                if grabbed:
                    continue
                else:
                    node.send_output("pause", pa.array([False]))
                    reachy.l_arm.gripper.open()
                    reachy.r_arm.gripper.open()
                values: np.array = event["value"].to_numpy()
                x = values[0]
                y = values[1]
                z = values[2]
                node.send_output("look", pa.array([x, y, z]))
                x = x + 0.04
                z = np.clip(z + 0.01, -0.32, -0.22)
                if y < 0:
                    y = y
                    reachy_arm = reachy.r_arm
                    joint_upper = r_arm_go_to_mixed_angles(reachy, x, y, -0.16)
                    joint_lower = r_arm_go_to_mixed_angles(reachy, x, y, z)
                else:
                    y = y + 0.03
                    reachy_arm = reachy.l_arm
                    joint_upper = l_arm_go_to_mixed_angles(reachy, x, y, -0.16)
                    joint_lower = l_arm_go_to_mixed_angles(reachy, x, y, z)

                if len(joint_lower):
                    if not len(joint_upper):
                        if reachy_arm == reachy.l_arm:
                            joint_upper = l_default_pos
                        else:
                            joint_upper = r_default_pos
                    reachy_arm.goto(joint_upper, duration=0.75, wait=True)
                    reachy_arm.goto(joint_lower, duration=0.75, wait=True)
                    reachy_arm.gripper.close()
                    time.sleep(0.5)
                    if (
                        reachy_arm.gripper.get_current_opening() > 2
                        and reachy_arm.gripper.get_current_opening() < 97
                    ):
                        reachy_arm.goto(joint_upper, duration=0.75, wait=True)
                        if (
                            reachy_arm.gripper.get_current_opening() > 2
                            and reachy_arm.gripper.get_current_opening() < 97
                        ):
                            grabbed = True
                            node.send_output("look", pa.array([0.3, 0, 0.0]))
                            node.send_output("rotate", pa.array(["right"]))
                            if reachy_arm == reachy.l_arm:
                                reachy_arm.goto(l_release_pos, duration=1.5, wait=False)
                            else:
                                reachy_arm.goto(r_release_pos, duration=1.5, wait=False)
                            continue
                    node.send_output("pause", pa.array([False]))
                if reachy_arm == reachy.l_arm:
                    reachy_arm.goto(l_init_pos, duration=1.3, wait=True)
                else:
                    reachy_arm.goto(r_init_pos, duration=1.3, wait=True)
                time.sleep(0.4)

            elif event["id"] == "text":
                text = event["value"][0].as_py()
                print("text: ", text, flush=True)
                words = text.lower().split()
                if (
                    len(ACTIVATION_WORDS) > 0
                    and any(word in ACTIVATION_WORDS for word in words)
                    and not started
                ):
                    started = True
                    node.send_output("rotate", pa.array(["left"]))
                    node.send_output("look", pa.array([0.3, 0, -0.1]))
                    reachy.l_arm.goto(l_init_pos, duration=1.0, wait=True)
                    reachy.r_arm.goto(r_init_pos, duration=1.0, wait=True)

            elif "action_arm" in event["id"]:
                text = event["value"][0].as_py()
                if text == "cancel":
                    reachy.l_arm.cancel_all_goto()
                    reachy.r_arm.cancel_all_goto()
                elif text == "release":
                    time.sleep(0.5)
                    reachy.l_arm.gripper.open()
                    reachy.r_arm.gripper.open()
                    reachy.l_arm.goto(l_init_pos, duration=1.0, wait=False)
                    reachy.r_arm.goto(r_init_pos, duration=1.0, wait=True)

                    started = False
                    grabbed = False
                elif text == "handwave":
                    if started:
                        continue

                    if time.time() - last_handwave < 5:
                        continue
                    last_handwave = time.time()

                    reachy.l_arm.goto(hand_wave_pos1, duration=0.75, wait=True)
                    reachy.l_arm.goto(hand_wave_pos2, duration=0.75, wait=True)
                    reachy.l_arm.goto(hand_wave_pos1, duration=0.75, wait=True)
                    reachy.l_arm.goto(l_init_pos, duration=1, wait=True)


if __name__ == "__main__":
    main()
