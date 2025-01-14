from dora import Node


import h5py

f = h5py.File("data/episode_0.hdf5", "r")

data = f["action"][:]


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

import time
import pyarrow as pa

node = Node()
LEFT_UNI_STATE_INDICES = [
    STATE_VEC_IDX_MAPPING[f"left_arm_joint_{i}_pos"] for i in range(6)
] + [STATE_VEC_IDX_MAPPING["left_gripper_open"]]
RIGHT_UNI_STATE_INDICES = [
    STATE_VEC_IDX_MAPPING[f"right_arm_joint_{i}_pos"] for i in range(6)
] + [STATE_VEC_IDX_MAPPING["right_gripper_open"]]
MOBILE_BASE_UNI_STATE_INDICES = [STATE_VEC_IDX_MAPPING["base_vel_x"]] + [
    STATE_VEC_IDX_MAPPING["base_angular_vel"]
]

for joint in data:
    node.send_output(
        "jointstate_left", pa.array(joint[LEFT_UNI_STATE_INDICES], type=pa.float32())
    )
    node.send_output(
        "jointstate_right", pa.array(joint[RIGHT_UNI_STATE_INDICES], type=pa.float32())
    )
    node.send_output(
        "mobile_base", pa.array(joint[MOBILE_BASE_UNI_STATE_INDICES], type=pa.float32())
    )
    time.sleep(0.05)
