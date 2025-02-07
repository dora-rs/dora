import os
import time

import numpy as np
import pyarrow as pa
from dora import Node
from reachy2_sdk import ReachySDK
from reachy2_sdk.media.camera import CameraView

l_pos = [
    21.92770419820605,
    -15.859152837636007,
    1.933394967929064,
    -84.40842032912303,
    -4.907079399756668,
    -14.262566237067908,
    91.60527454434958,
]
r_pos = [
    20.822557392383185,
    11.04315827932969,
    4.5280773253545386,
    -89.76466150643621,
    5.045636190372724,
    -19.29307313154223,
    -87.43405752765528,
]
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
fistbump_pos = [
    -60.50857072506209,
    15.288403447569523,
    -23.22067096092051,
    -25.439633790170284,
    -4.034621344143681,
    -10.79858115798258,
    -76.35394024095301,
]
hand_wave_pos1 = [
    -55.72474038133101,
    -12.946062642473581,
    -38.32685520671785,
    -105.7722343547366,
    -5.224672523991683,
    -4.172396504986594,
    -5.414316568278495,
]
hand_wave_pos2 = [
    -44.38270924562084,
    -30.137864788128176,
    20.102804010478287,
    -114.34005097035727,
    -5.214963836973333,
    -4.169346825522183,
    -5.413211358293423,
]
hand_shake_pos1 = [
    -18.494675243986514,
    -13.983687905708994,
    1.8276602244826816,
    -95.83702069746853,
    1.614473692065599,
    24.50849117609537,
    -0.6833807035838743,
]
hand_shake_pos2 = [
    -7.451341087285697,
    -12.443260828044032,
    2.233412948212578,
    -74.49182576753243,
    2.3681770628485728,
    -11.323536690985067,
    -0.9424216042721315,
]
pick_pos1 = [
    9.844628273400764,
    -19.463764681542727,
    9.71784800957428,
    -93.66575137164567,
    -11.96666474191253,
    -7.695120784047207,
    85.23235287889085,
]

pick_pos2 = [
    -82.72212858531692,
    33.91639373887093,
    -100.79539652543929,
    -124.57793763050528,
    -39.342354072137006,
    -5.594463661411228,
    83.75117854604939,
]
pick_pos3 = [
    -56.39551276915045,
    13.304733805037523,
    -121.41232253512278,
    -103.22085134122418,
    -1.9579418140324436,
    26.933823048475197,
    58.16491425477727,
]

pick_pos4 = [
    -18.26771488805616,
    -10.94845599894017,
    11.591648936664747,
    -85.61138739669154,
    2.060616419287249,
    -13.576030626417722,
    10.177961995464896,
]

init_pose = [
    46.078474461805754,
    -8.044685596900228,
    -4.860684912932533,
    -104.6900523524506,
    -1.7181514813547498,
    -21.771478988867642,
    2.786325085818161,
]


def go_to_mixed_angles(reachy, x, y):
    for theta in range(-90, 0, 10):
        cos = np.float64(np.cos(np.deg2rad(theta)))
        sin = np.float64(np.sin(np.deg2rad(theta)))
        a = np.array(
            [
                [sin, 0.0, -cos, np.float64(x)],
                [0.0, 1.0, 0.0, np.float64(y)],
                [cos, 0.0, sin, -0.32],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        try:
            return reachy.l_arm.inverse_kinematics(a)
        except ValueError:
            continue
    print("could not solve for x: ", x, "y: ", y)
    return []


def main():
    ROBOT_IP = os.getenv("ROBOT_IP", "172.17.134.85")

    reachy = ReachySDK(ROBOT_IP)

    reachy.turn_on()
    if reachy.l_arm is not None:
        reachy.l_arm.turn_on()
        reachy.l_arm.goto(middle_pos)
        reachy.l_arm.gripper.turn_on()
    node = Node()

    def la_goto(action, pos, time_sleep=0.2, duration=2):
        reachy.l_arm.goto(pos, duration=duration)
        move_time = time.time()
        double_action = action
        while time.time() - move_time < duration + time_sleep:
            event = node.next(timeout=duration + time_sleep)

            if (
                event is not None
                and event["type"] == "INPUT"
                and event["id"] == "action_arm"
                and (
                    event["value"][0].as_py() == action
                    or event["value"][0].as_py() == "wait"
                )
            ):
                continue
            elif event["type"] == "ERROR":
                continue
            else:
                if event is None or (
                    event["type"] == "INPUT"
                    and double_action == event["value"][0].as_py()
                ):
                    reachy.l_arm.cancel_all_goto()
                    return False
                else:
                    if event is not None and event["type"] == "INPUT":
                        double_action = event["value"][0].as_py()
                    else:
                        return False
                    continue
        return True

    holding = False
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "pose":
                if (
                    reachy.l_arm.gripper.get_current_opening() > 30
                    and reachy.l_arm.gripper.get_current_opening() < 60
                ):
                    holding = True
                    continue
                else:
                    reachy.l_arm.gripper.open()

                values = event["value"].to_numpy()
                x = values[0]
                y = values[1]

                joint = go_to_mixed_angles(reachy, x + 0.06, y + 0.03)
                if len(joint):
                    la_goto("pose", joint, time_sleep=0.5)
                    # reachy.l_arm.gripper.close()

                la_goto("pose", init_pose, time_sleep=0.5)

            elif event["id"] == "action_arm":
                text = event["value"][0].as_py()
                if reachy.l_arm.is_off():
                    reachy.l_arm.turn_on()
                    reachy.l_arm.gripper.turn_on()
                if text == "cancel":
                    reachy.l_arm.cancel_all_goto()
                elif len(reachy.l_arm.get_goto_queue()) == 0:
                    if text == "grab":
                        if reachy.l_arm.gripper.get_current_opening() > 99:
                            reachy.l_arm.gripper.close()
                        la_goto(text, middle_pos, time_sleep=0.3)
                        la_goto(text, grab_pos, time_sleep=0.3)
                        reachy.l_arm.gripper.open()
                        time.sleep(1)
                        reachy.l_arm.gripper.close()
                        la_goto(text, middle_pos)
                    if text == "release":
                        if reachy.l_arm.gripper.get_current_opening() > 99:
                            reachy.l_arm.gripper.close()
                        ok = (
                            la_goto(text, grab_pos, time_sleep=1)
                            and la_goto(text, middle_pos)
                            and la_goto(text, l_pos)
                            and reachy.l_arm.gripper.open() is None
                        )

                    if text == "napkin":
                        ok = (
                            la_goto(text, pick_pos1, duration=2, time_sleep=0.5)
                            and reachy.l_arm.gripper.open() is None
                            and la_goto(text, pick_pos2, duration=2)
                            and la_goto(text, pick_pos3, duration=1, time_sleep=0.5)
                            and reachy.l_arm.gripper.close() is None
                            and la_goto(text, pick_pos2, duration=1)
                            and la_goto(text, pick_pos1, duration=2)
                            and la_goto(text, grab_pos, duration=2, time_sleep=1)
                            and reachy.l_arm.gripper.open() is None
                            and la_goto(text, l_pos)
                        )
                        node.send_output(
                            "text", pa.array(["Respond to people without moving."])
                        )
                    if text == "wait":
                        pass

    # reachy.l_arm.turn_off_smoothly()


if __name__ == "__main__":
    main()
