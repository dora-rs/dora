import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from scipy.spatial.transform import Rotation as R

grab_pos = [
    -35.622223238797865,
    -20.328451262540707,
    -0.8628232733758905,
    -79.19811297361636,
    18.120519188800507,
    21.44259513504777,
    -13.68669505889756,
]
middle_pos = [
    17.173603960384742,
    -13.860769260084506,
    5.1834147663102215,
    -125.26299194387921,
    -8.21038513239109,
    23.27173004006094,
    0.19036855126693597,
]
init_pos = [
    -7.0631310641087435,
    -10.432298603362307,
    24.429809104404114,
    -132.15000828778648,
    -1.5494749438811133,
    -21.749917789205202,
    8.099312596108344,
]

release_pos = [
    -73.23388250733451,
    -12.958696784890813,
    7.140870569694536,
    -51.761260153148065,
    18.644490320788865,
    31.462105279504836,
    -6.320949902290464,
]

hold_pos = [
    -26.335267958297592,
    -5.826892437545823,
    19.403437035698587,
    -116.4365024040118,
    4.197998188429917,
    44.08779533763803,
    -1.8953070583527227,
]

ACTIVATION_WORDS = os.getenv("ACTIVATION_WORDS", "").split()


def go_to_mixed_angles(reachy, x, y, z):
    for theta in range(-80, -60, 10):
        r = R.from_euler("zyx", [0, theta, 0], degrees=True)
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.l_arm.inverse_kinematics(transform)

        except ValueError:
            continue
    for yaw in range(0, 30, 10):

        ## First try turning left
        pitch = -90
        roll = 0
        r = (
            R.from_euler("ZYX", (yaw, 0, 0), degrees=True)
            * R.from_euler("ZYX", (0, pitch, 0), degrees=True)
            * R.from_euler("ZYX", (0, 0, roll), degrees=True)
        )
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.l_arm.inverse_kinematics(transform)
        except ValueError:
            pass

        ## Then try turning right
        pitch = -90
        roll = 0
        r = (
            R.from_euler("ZYX", (-yaw, 0, 0), degrees=True)
            * R.from_euler("ZYX", (0, pitch, 0), degrees=True)
            * R.from_euler("ZYX", (0, 0, roll), degrees=True)
        )
        transform = np.eye(4)
        transform[:3, :3] = r.as_matrix()
        transform[:3, 3] = [x, y, z]

        try:
            return reachy.l_arm.inverse_kinematics(transform)

        except ValueError:
            continue
    print("No solution found")
    return []


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "172.17.134.85")

    reachy = ReachySDK(ROBOT_IP)

    if reachy.l_arm is not None:
        reachy.l_arm.turn_on()
        reachy.l_arm.goto(init_pos)
        reachy.l_arm.gripper.turn_on()
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
                values: np.array = event["value"].to_numpy()
                x = values[0]
                y = values[1]
                z = values[2]
                z = np.clip(z + 0.01, -0.33, -0.2)
                print("pose: ", values, flush=True)

                joint_upper = go_to_mixed_angles(reachy, x + 0.06, y + 0.03, -0.15)
                joint_lower = go_to_mixed_angles(reachy, x + 0.06, y + 0.03, z)
                joint_higher = go_to_mixed_angles(reachy, x + 0.06, y + 0.03, -0.1)
                if len(joint_upper) and len(joint_lower) and len(joint_higher):
                    reachy.l_arm.goto(joint_upper, duration=0.75, wait=True)
                    reachy.l_arm.goto(joint_lower, duration=0.75, wait=True)
                    reachy.l_arm.gripper.close()
                    time.sleep(0.5)
                    if (
                        reachy.l_arm.gripper.get_current_opening() > 2
                        and reachy.l_arm.gripper.get_current_opening() < 97
                    ):
                        reachy.l_arm.goto(joint_higher, duration=0.75, wait=True)
                        if (
                            reachy.l_arm.gripper.get_current_opening() > 2
                            and reachy.l_arm.gripper.get_current_opening() < 97
                        ):
                            grabbed = True
                            print("grabbed", flush=True)
                            node.send_output("look", pa.array([0, 0, 0]))
                            node.send_output("rotate", pa.array(["right"]))
                            reachy.l_arm.goto(release_pos, duration=1.5, wait=False)
                            node.send_output("pause", pa.array([True]))
                            continue

                reachy.l_arm.goto(init_pos, duration=1)

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
                    node.send_output("look", pa.array([0, 25, 0]))
                    reachy.l_arm.goto(init_pos, duration=1.0, wait=True)

            elif "action_arm" in event["id"]:
                text = event["value"][0].as_py()
                if text == "cancel":
                    reachy.l_arm.cancel_all_goto()
                elif text == "grab":
                    if reachy.l_arm.gripper.get_current_opening() > 99:
                        reachy.l_arm.gripper.close()
                    reachy.l_arm.goto(text, middle_pos, wait=True)
                    reachy.l_arm.goto(text, grab_pos, wait=True)
                    reachy.l_arm.gripper.open()
                    time.sleep(1)
                    reachy.l_arm.gripper.close()
                    reachy.l_arm.goto(text, middle_pos, wait=True)
                if text == "release":
                    time.sleep(0.5)
                    reachy.l_arm.gripper.open()
                    reachy.l_arm.goto(init_pos, duration=1)

                    started = False
                    grabbed = False
                if text == "wait":
                    pass

    # reachy.l_arm.turn_off_smoothly()


if __name__ == "__main__":
    main()
