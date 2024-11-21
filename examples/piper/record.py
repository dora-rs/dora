import h5py

import os
import datetime

from dora import Node
import numpy as np
from convert import (
    convert_euler_to_rotation_matrix,
    compute_ortho6d_from_rotation_matrix,
)

STATE_VEC_IDX_MAPPING = {
    # [0, 10): right arm joint positions
    **{"arm_joint_{}_pos".format(i): i for i in range(10)},
    **{"right_arm_joint_{}_pos".format(i): i for i in range(10)},
    # [10, 15): right gripper joint positions
    **{"gripper_joint_{}_pos".format(i): i + 10 for i in range(5)},
    **{"right_gripper_joint_{}_pos".format(i): i + 10 for i in range(5)},
    "gripper_open": 10,  # alias of right_gripper_joint_0_pos
    "right_gripper_open": 10,
    # [15, 25): right arm joint velocities
    **{"arm_joint_{}_vel".format(i): i + 15 for i in range(10)},
    **{"right_arm_joint_{}_vel".format(i): i + 15 for i in range(10)},
    # [25, 30): right gripper joint velocities
    **{"gripper_joint_{}_vel".format(i): i + 25 for i in range(5)},
    **{"right_gripper_joint_{}_vel".format(i): i + 25 for i in range(5)},
    "gripper_open_vel": 25,  # alias of right_gripper_joint_0_vel
    "right_gripper_open_vel": 25,
    # [30, 33): right end effector positions
    "eef_pos_x": 30,
    "right_eef_pos_x": 30,
    "eef_pos_y": 31,
    "right_eef_pos_y": 31,
    "eef_pos_z": 32,
    "right_eef_pos_z": 32,
    # [33, 39): right end effector 6D pose
    "eef_angle_0": 33,
    "right_eef_angle_0": 33,
    "eef_angle_1": 34,
    "right_eef_angle_1": 34,
    "eef_angle_2": 35,
    "right_eef_angle_2": 35,
    "eef_angle_3": 36,
    "right_eef_angle_3": 36,
    "eef_angle_4": 37,
    "right_eef_angle_4": 37,
    "eef_angle_5": 38,
    "right_eef_angle_5": 38,
    # [39, 42): right end effector velocities
    "eef_vel_x": 39,
    "right_eef_vel_x": 39,
    "eef_vel_y": 40,
    "right_eef_vel_y": 40,
    "eef_vel_z": 41,
    "right_eef_vel_z": 41,
    # [42, 45): right end effector angular velocities
    "eef_angular_vel_roll": 42,
    "right_eef_angular_vel_roll": 42,
    "eef_angular_vel_pitch": 43,
    "right_eef_angular_vel_pitch": 43,
    "eef_angular_vel_yaw": 44,
    "right_eef_angular_vel_yaw": 44,
    # [45, 50): reserved
    # [50, 60): left arm joint positions
    **{"left_arm_joint_{}_pos".format(i): i + 50 for i in range(10)},
    # [60, 65): left gripper joint positions
    **{"left_gripper_joint_{}_pos".format(i): i + 60 for i in range(5)},
    "left_gripper_open": 60,  # alias of left_gripper_joint_0_pos
    # [65, 75): left arm joint velocities
    **{"left_arm_joint_{}_vel".format(i): i + 65 for i in range(10)},
    # [75, 80): left gripper joint velocities
    **{"left_gripper_joint_{}_vel".format(i): i + 75 for i in range(5)},
    "left_gripper_open_vel": 75,  # alias of left_gripper_joint_0_vel
    # [80, 83): left end effector positions
    "left_eef_pos_x": 80,
    "left_eef_pos_y": 81,
    "left_eef_pos_z": 82,
    # [83, 89): left end effector 6D pose
    "left_eef_angle_0": 83,
    "left_eef_angle_1": 84,
    "left_eef_angle_2": 85,
    "left_eef_angle_3": 86,
    "left_eef_angle_4": 87,
    "left_eef_angle_5": 88,
    # [89, 92): left end effector velocities
    "left_eef_vel_x": 89,
    "left_eef_vel_y": 90,
    "left_eef_vel_z": 91,
    # [92, 95): left end effector angular velocities
    "left_eef_angular_vel_roll": 92,
    "left_eef_angular_vel_pitch": 93,
    "left_eef_angular_vel_yaw": 94,
    # [95, 100): reserved
    # [100, 102): base linear velocities
    "base_vel_x": 100,
    "base_vel_y": 101,
    # [102, 103): base angular velocities
    "base_angular_vel": 102,
    # [103, 128): reserved
}
STATE_VEC_LEN = 128


now = datetime.datetime.now()

ROOT_DIR = os.getenv("DATA_DIR", "/home/agilex/Desktop")
DATA_DIR = ROOT_DIR + now.strftime("%Y.%m.%d")
os.makedirs(DATA_DIR, exist_ok=True)

## Make data dir if it does not exist
if not os.path.exists(DATA_DIR):
    os.makedirs(DATA_DIR)


def save_data(data_dict, dataset_path, data_size):
    with h5py.File(dataset_path + ".hdf5", "w", rdcc_nbytes=1024**2 * 2) as root:
        root.attrs["sim"] = False
        root.attrs["compress"] = False

        obs = root.create_group("observations")
        variable_length = h5py.vlen_dtype(np.dtype("uint8"))
        image = obs.create_group("images")
        _ = image.create_dataset(
            "cam_high",
            (data_size,),
            dtype=variable_length,
        )
        _ = image.create_dataset(
            "cam_left_wrist",
            (data_size,),
            dtype=variable_length,
        )
        _ = image.create_dataset(
            "cam_right_wrist",
            (data_size,),
            dtype=variable_length,
        )

        _ = obs.create_dataset("qpos", (data_size, 128))
        _ = root.create_dataset("action", (data_size, 128))

        # data_dict write into h5py.File
        for name, array in data_dict.items():
            print(name)
            if "images" in name:
                image[name][...] = array
            else:
                root[name][...] = array


data_dict = {
    "/observations/qpos": [],
    "/observations/images/cam_high": [],
    "/observations/images/cam_left_wrist": [],
    "/observations/images/cam_right_wrist": [],
    "/action": [],
}


node = Node()

LEAD_CAMERA = "/observations/images/cam_high"

tmp_dict = {}

i = 0

start = False
for event in node:
    if event["type"] == "INPUT":
        if "save" in event["id"]:
            char = event["value"][0].as_py()
            if char == "p":
                if start == False:
                    continue

                save_data(
                    data_dict,
                    f"{DATA_DIR}/episode_{i}",
                    len(data_dict["/observations/qpos"]),
                )

                # Reset dict
                data_dict = {
                    "/observations/qpos": [],
                    "/observations/images/cam_high": [],
                    "/observations/images/cam_left_wrist": [],
                    "/observations/images/cam_right_wrist": [],
                    "/action": [],
                }
                i += 1
                start = False
            elif char == "s":
                start = True

        elif "image" in event["id"]:
            tmp_dict[event["id"]] = event["value"].to_numpy()
        elif "qpos" in event["id"]:
            tmp_dict[event["id"]] = event["value"].to_numpy()
        elif "pose" in event["id"]:
            value = event["value"].to_numpy()
            euler = value[None, 3:6]  # Add batch dimension
            rotmat = convert_euler_to_rotation_matrix(euler)
            ortho6d = compute_ortho6d_from_rotation_matrix(rotmat)[0]
            values = np.array(
                [
                    value[0],
                    value[1],
                    value[2],
                    ortho6d[0],
                    ortho6d[1],
                    ortho6d[2],
                    ortho6d[3],
                    ortho6d[4],
                    ortho6d[5],
                ]
            )
            tmp_dict[event["id"]] = values
        elif "base_vel" in event["id"]:
            tmp_dict[event["id"]] = event["value"].to_numpy()

        # Check if tmp dict is full
        if len(tmp_dict) != 8:
            continue
        elif event["id"] == LEAD_CAMERA and start == True:
            values = np.concatenate(
                [
                    tmp_dict["/observations/qpos_left"],
                    tmp_dict["/observations/qpos_right"],
                    tmp_dict["/observations/pose_left"],
                    tmp_dict["/observations/pose_right"],
                    # tmp_dict["/observations/base_vel"],
                ]
            )
            UNI_STATE_INDICES = (
                [STATE_VEC_IDX_MAPPING[f"left_arm_joint_{i}_pos"] for i in range(6)]
                + [STATE_VEC_IDX_MAPPING["left_gripper_open"]]
                + [STATE_VEC_IDX_MAPPING[f"right_arm_joint_{i}_pos"] for i in range(6)]
                + [STATE_VEC_IDX_MAPPING["right_gripper_open"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_pos_x"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_pos_y"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_pos_z"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_angle_0"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_angle_1"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_angle_2"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_angle_3"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_angle_4"]]
                + [STATE_VEC_IDX_MAPPING["left_eef_angle_5"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_pos_x"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_pos_y"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_pos_z"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_angle_0"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_angle_1"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_angle_2"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_angle_3"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_angle_4"]]
                + [STATE_VEC_IDX_MAPPING["right_eef_angle_5"]]
                # + [STATE_VEC_IDX_MAPPING["base_vel_x"]]
                # + [STATE_VEC_IDX_MAPPING["base_angular_vel"]],
            )
            universal_vec = np.zeros(STATE_VEC_LEN)
            universal_vec[UNI_STATE_INDICES] = values
            data_dict["/observations/qpos"].append(universal_vec)
            # We reproduce obs and action
            data_dict["/action"].append(universal_vec)
            data_dict["/observations/images/cam_high"].append(
                tmp_dict["/observations/images/cam_high"]
            )
            data_dict["/observations/images/cam_left_wrist"].append(
                tmp_dict["/observations/images/cam_left_wrist"]
            )
            data_dict["/observations/images/cam_right_wrist"].append(
                tmp_dict["/observations/images/cam_right_wrist"]
            )
