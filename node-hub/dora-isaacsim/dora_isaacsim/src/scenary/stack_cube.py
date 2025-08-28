import numpy as np
from isaacsim.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.objects import DynamicCuboid

from src.utils import to_numpy_recursive

from .base import BaseScenary


class StackCubeScenary(BaseScenary):
    def __init__(self, src_cube_cfg=None, target_cube_cfg=None, random=False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if random:
            src_cube_cfg, target_cube_cfg = self.random_gen()
        if src_cube_cfg is not None:
            self.source_cube = self.add_cube(src_cube_cfg)
        if target_cube_cfg is not None:
            self.target_cube = self.add_cube(target_cube_cfg)

    def add_cube(self, cube_cfg):
        self.load_stage()
        cube = self.world.scene.add(DynamicCuboid(**to_numpy_recursive(cube_cfg)))

        return cube

    def random_gen(self):
        # 随机生成位置
        # 设定随机范围
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
        # 保证两个物体不重叠
        while np.linalg.norm(pos_red - pos_green) < 0.1:
            pos_green = np.array([np.random.uniform(*x_range), np.random.uniform(*y_range), 0.12])

        cube_red_cfg = {
            "name": "cube",
            "position": pos_red,
            "orientation": quat,
            "prim_path": "/World/Cube",
            "scale": np.array([0.05, 0.05, 0.05]),
            "size": 1.0,
            "color": np.array([1, 0, 0]),
        }

        cube_green_cfg = {
            "name": "cube2",
            "position": pos_green,
            "orientation": quat,
            "prim_path": "/World/Cube2",
            "scale": np.array([0.05, 0.05, 0.05]),
            "size": 1.0,
            "color": np.array([0, 0, 1]),
        }

        return cube_red_cfg, cube_green_cfg
