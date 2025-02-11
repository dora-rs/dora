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
    -30.04330081906935,
    -7.415231584691132,
    3.6972339048071468,
    -97.7274736257555,
    12.996718740452982,
    30.838020649757016,
    -1.5572310505704858,
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


def go_to_mixed_angles(reachy, x, y, z, no_angle=False):
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
    for yaw in range(0, 30, 10):

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
    print("No solution found for x, y, z: ", x, y, z)
    return []


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "10.42.0.80")

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
                if y < 0:
                    continue
                z = values[2]
                node.send_output("look", pa.array([x, y, z]))
                z = np.clip(z + 0.01, -0.32, -0.22)

                joint_upper = go_to_mixed_angles(reachy, x + 0.06, y + 0.03, -0.16)
                joint_lower = go_to_mixed_angles(reachy, x + 0.06, y + 0.03, z)

                if len(joint_upper) and len(joint_lower):
                    node.send_output("pause", pa.array([True]))
                    reachy.l_arm.goto(joint_upper, duration=0.75, wait=True)
                    reachy.l_arm.goto(joint_lower, duration=0.75, wait=True)
                    reachy.l_arm.gripper.close()
                    time.sleep(0.5)
                    if (
                        reachy.l_arm.gripper.get_current_opening() > 2
                        and reachy.l_arm.gripper.get_current_opening() < 97
                    ):
                        reachy.l_arm.goto(joint_upper, duration=0.75, wait=True)
                        if (
                            reachy.l_arm.gripper.get_current_opening() > 2
                            and reachy.l_arm.gripper.get_current_opening() < 97
                        ):
                            grabbed = True
                            node.send_output("look", pa.array([0.3, 0, 0]))
                            node.send_output("rotate", pa.array(["right"]))
                            reachy.l_arm.goto(release_pos, duration=1.5, wait=False)
                            continue
                    node.send_output("pause", pa.array([False]))
                reachy.l_arm.goto(init_pos, duration=1.3, wait=True)
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
                    node.send_output("look", pa.array([0.3, 0, 0.0]))
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
