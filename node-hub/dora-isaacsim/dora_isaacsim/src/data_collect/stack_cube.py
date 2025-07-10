import os

import numpy as np
from isaacsim.core.utils.rotations import euler_angles_to_quat


from .base import BaseDataCollect


class StackCubeDataCollect(BaseDataCollect):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # 设定随机范围
        x_range = (0.5, 0.6)  # X 轴范围
        x_range_shifted = (x_range[0] + 0.05, x_range[1] + 0.05)
        y_range = (-0.15, 0.15)  # Y 轴范围

        # 随机 yaw 角度（单位：弧度）
        yaw = np.random.uniform(-np.pi / 4, np.pi / 4)

        # 欧拉角（roll, pitch, yaw） -> 四元数 [w, x, y, z]
        quat = euler_angles_to_quat([0, 0, yaw])  # 只绕 Z 轴旋转

        pos_red = np.array([np.random.uniform(*x_range), np.random.uniform(*y_range), 0.12])

        pos_green = np.array(
            [np.random.uniform(*x_range_shifted), np.random.uniform(*y_range), 0.12]
        )

        while np.linalg.norm(pos_red - pos_green) < 0.1:
            pos_green = np.array([np.random.uniform(*x_range), np.random.uniform(*y_range), 0.12])

        self.cube_red_cfg = {
            "name": "cube",
            "position": pos_red,
            "orientation": quat,
            "prim_path": "/World/Cube",
            "scale": np.array([0.05, 0.05, 0.05]),
            "size": 1.0,
            "color": np.array([1, 0, 0]),
        }

        self.cube_green_cfg = {
            "name": "cube2",
            "position": pos_green,
            "orientation": quat,
            "prim_path": "/World/Cube2",
            "scale": np.array([0.05, 0.05, 0.05]),
            "size": 1.0,
            "color": np.array([0, 0, 1]),
        }

        states = self.robot.get_ee_pose()
        init_pos = states[:3]
        init_orientation = states[3:7]
        gripper_quat = np.array(
            [
                -quat[1],
                quat[0],
                quat[3],
                -quat[2],
            ]
        )

        trajectory = [
            {"t": 0, "xyz": init_pos, "quat": init_orientation, "gripper": 1.0},  # Start
            {
                "t": 40,
                "xyz": [pos_red[0], pos_red[1], pos_red[2] + 0.20],
                "quat": init_orientation,
                "gripper": 1.0,
            },  # 靠近
            {
                "t": 60,
                "xyz": [pos_red[0], pos_red[1], pos_red[2] - 0.02],
                "quat": gripper_quat,
                "gripper": 1.0,
            },  # 下沉
            {
                "t": 68,
                "xyz": [pos_red[0], pos_red[1], pos_red[2] - 0.02],
                "quat": gripper_quat,
                "gripper": 0.1,
            },  # 抓取
            {
                "t": 93,
                "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.20],
                "quat": gripper_quat,
                "gripper": 0.1,
            },  # 移动
            {
                "t": 112,
                "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.10],
                "quat": gripper_quat,
                "gripper": 0.1,
            },  # 下沉
            {
                "t": 125,
                "xyz": [pos_green[0], pos_green[1], pos_green[2] + 0.10],
                "quat": gripper_quat,
                "gripper": 1.0,
            },  # 释放
        ]

        self.scenary.add_cube(self.cube_red_cfg)
        self.scenary.add_cube(self.cube_green_cfg)
        self.robot.set_trajectory(trajectory)
        self.reset()

    def save_data(self, output_dir, data_list, cube_pose1=None, cube_pose2=None):
        cube_pose1 = np.concatenate(
            [self.cube_red_cfg["position"], self.cube_red_cfg["orientation"]]
        )
        cube_pose2 = np.concatenate(
            [self.cube_green_cfg["position"], self.cube_green_cfg["orientation"]]
        )

        # 创建目录结构
        meta_dir = os.path.join(output_dir, "meta")
        data_dir = os.path.join(output_dir, "data")
        os.makedirs(meta_dir, exist_ok=True)
        os.makedirs(data_dir, exist_ok=True)

        ee_pose_list = []
        joint_pos_list = []
        gripper_list = []
        action_ee_pose_list = []
        action_joint_pose_list = []
        action_gripper_list = []

        for t in range(len(data_list)):
            ee_pose = data_list[t]["robot"]["states"]["ee_pose"]
            joint_pos = data_list[t]["robot"]["states"]["joint_pos"]
            action_ee_pose = data_list[t]["action"]["ee_pose"]
            action_joint_pose = data_list[t]["action"]["joint_pos"]

            ee_pose_list.append(ee_pose)
            joint_pos_list.append(joint_pos)
            gripper_list.append(joint_pos[-1])

            action_ee_pose_list.append(action_ee_pose)
            action_joint_pose_list.append(action_joint_pose)
            action_gripper_list.append(action_joint_pose[-1])

        # 存储机器人初始关节pos数据
        np.savetxt(os.path.join(data_dir, "ee_pose.txt"), ee_pose_list, fmt="%.6f")
        np.savetxt(os.path.join(data_dir, "joint_pos.txt"), joint_pos_list, fmt="%.6f")
        np.savetxt(os.path.join(data_dir, "gripper_width.txt"), gripper_list, fmt="%.6f")

        # 存储动作
        np.savetxt(os.path.join(data_dir, "action_ee_pose.txt"), action_ee_pose_list, fmt="%.6f")
        np.savetxt(
            os.path.join(data_dir, "action_joint_pos.txt"), action_joint_pose_list, fmt="%.6f"
        )
        np.savetxt(
            os.path.join(data_dir, "action_gripper_width.txt"), action_gripper_list, fmt="%.6f"
        )

        if cube_pose1 is not None:
            np.savetxt(os.path.join(data_dir, "cube_red"), cube_pose1.reshape(1, -1), fmt="%.6f")
        if cube_pose2 is not None:
            np.savetxt(os.path.join(data_dir, "cube_blue"), cube_pose2.reshape(1, -1), fmt="%.6f")

        print(f"Data saved successfully in {output_dir}")
