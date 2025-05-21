"""TODO: Add docstring."""

import json
import os
import time

import lebai_sdk
import numpy as np
from dora import Node


def load_json_file(file_path):
    """Load JSON file and return the dictionary."""
    if os.path.exists(file_path):
        with open(file_path) as file:
            data = json.load(file)
    else:
        # Return an empty dictionary if file does not exist
        data = {"recording": {}, "pose": {}}
    return data


def save_json_file(file_path, data):
    """Save the dictionary back to the JSON file."""
    with open(file_path, "w") as file:
        json.dump(data, file, indent=4)


SAVED_POSE_PATH = "pose_library.json"

lebai_sdk.init()
ROBOT_IP = os.getenv(
    "LEBAI_IP",
    "10.42.0.253",
)  # 设定机器人ip地址，需要根据机器人实际ip地址修改


def main():
    """TODO: Add docstring."""
    # Load the JSON file
    pose_library = load_json_file(SAVED_POSE_PATH)
    lebai = lebai_sdk.connect(ROBOT_IP, False)  # 创建实例

    lebai.start_sys()  # 启动手臂
    node = Node()
    recording = False
    teaching = False
    recording_name = None
    data = lebai.get_kin_data()
    [x, y, z, rx, ry, rz] = list(data["actual_tcp_pose"].values())
    joint_position = data["actual_joint_pose"]
    t = 0.15

    for event in node:
        if event["type"] == "INPUT":
            event_id = event["id"]
            if event_id == "claw":
                [claw] = event["value"].tolist()
                lebai.set_claw(10, claw)
            elif event_id == "movec":
                if teaching:
                    continue
                [dx, dy, dz, drx, dry, drz, t] = event["value"].tolist()

                cartesian_pose = {
                    "x": x + dx,
                    "y": y + dy,
                    "z": z + dz,
                    "rx": rx + drx,
                    "ry": ry + dry,
                    "rz": rz + drz,
                }  # 目标位姿笛卡尔数据

                t = 0.25  # 运动时间 (s)。 当 t > 0 时，参数速度 v 和加速度 a 无效
                try:
                    joint_position = lebai.kinematics_inverse(cartesian_pose)
                except TypeError:
                    print("could not compute inverse kinematics")
                    continue
                [x, y, z, rx, ry, rz] = list(cartesian_pose.values())
                lebai.move_pvat(
                    joint_position,
                    [0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
                    [0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
                    t,
                )  # 直线运动 https://help.lebai.ltd/sdk/motion.html#%E7%9B%B4%E7%BA%BF%E8%BF%90%E5%8A%A8
            elif event_id == "movej":
                if teaching:
                    continue
                relative_joint_position = event["value"].to_numpy()
                joint_position = np.array(joint_position)
                joint_position += np.array(relative_joint_position[:6])
                cartesian_pose = lebai.kinematics_forward(list(joint_position))
                [x, y, z, rx, ry, rz] = list(cartesian_pose.values())

                t = 0.15  # 运动时间 (s)。 当 t > 0 时，参数速度 v 和加速度 a 无效

                lebai.move_pvat(
                    list(joint_position),
                    [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                    [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                    t,
                )  # 直线运动 https://help.lebai.ltd/sdk/motion.html#%E7%9B%B4%E7%BA%BF%E8%BF%90%E5%8A%A8
            elif event_id == "stop":
                lebai.stop_move()
                data = lebai.get_kin_data()
                [x, y, z, rx, ry, rz] = list(data["actual_tcp_pose"].values())
                joint_position = list(data["actual_joint_pose"])
            elif event_id == "save":
                name = event["value"][0].as_py()
                lebai.stop_move()
                data = lebai.get_kin_data()
                [x, y, z, rx, ry, rz] = list(data["actual_tcp_pose"].values())
                joint_position = list(data["actual_joint_pose"])
                pose_library["pose"][name] = list(joint_position)
            elif event_id == "go_to":
                if teaching:
                    continue
                name = event["value"][0].as_py()
                lebai.stop_move()
                retrieved_pose = pose_library["pose"].get(name)
                if retrieved_pose is not None:
                    joint_position = retrieved_pose
                    t = 2
                    lebai.move_pvat(
                        list(joint_position),
                        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                        [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                        t,
                    )  # 直线运动 https://help.lebai.ltd/sdk/motion.html#%E7%9B%B4%E7%BA%BF%E8%BF%90%E5%8A%A8
                    lebai.wait_move()
                    data = lebai.get_kin_data()
                    [x, y, z, rx, ry, rz] = list(data["actual_tcp_pose"].values())
                    joint_position = list(data["actual_joint_pose"])
            elif event_id == "record":
                name = event["value"][0].as_py()
                recording = True

                recording_name = name
                pose_library["recording"][recording_name] = []
                start_time = time.time()
                data = lebai.get_kin_data()
                [x, y, z, rx, ry, rz] = list(data["actual_tcp_pose"].values())
                joint_position = list(data["actual_joint_pose"])
            elif event_id == "cut":
                recording = False
            elif event_id == "teach":
                if teaching:
                    teaching = False
                    continue
                lebai.teach_mode()
                teaching = True
            elif event_id == "end_teach":
                teaching = False
                lebai.end_teach_mode()
            elif event_id == "play":
                name = event["value"][0].as_py()
                if name in pose_library["recording"]:
                    for event in pose_library["recording"][name]:
                        print(event, flush=True)
                        lebai.move_pvat(
                            list(event["joint_position"]),
                            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                            [0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
                            event["t"],
                        )
                        event = node.next(timeout=event["duration"])
                        if event is not None:
                            print(event)
                            if event["type"] == "INPUT" and event["id"] == "stop":
                                lebai.stop_move()
                                break

            else:
                pass
            if recording and (
                event_id == "movej" or event_id == "movec" or event_id == "go_to"
            ):
                if len(pose_library["recording"][recording_name]) == 0:
                    t = 2
                pose_library["recording"][recording_name] += [
                    {
                        "duration": time.time() - start_time,
                        "joint_position": joint_position,
                        "t": t * 2 if t == 0.1 else t,
                    },
                ]
                start_time = time.time()

        save_json_file(SAVED_POSE_PATH, pose_library)
