import numpy as np
from omegaconf.dictconfig import DictConfig
from omegaconf.listconfig import ListConfig
from omni.isaac.dynamic_control import _dynamic_control
from scipy.spatial.transform import Rotation

from omni.isaac.core.prims import XFormPrim
import random
import torch

dc = _dynamic_control.acquire_dynamic_control_interface()

def debug():
    import debugpy

    # 启动调试器，指定调试端口
    debugpy.listen(5679)

    print("Waiting for debugger to attach...")

    # 在这里设置断点
    debugpy.wait_for_client()


def to_numpy_recursive(cfg):
    # 将 OmegaConf 里面的所有 list 转换成 numpy
    if isinstance(cfg, dict) or isinstance(cfg, DictConfig):
        return {k: to_numpy_recursive(v) for k, v in cfg.items()}
    elif isinstance(cfg, ListConfig):
        return np.array(cfg)
    else:
        return cfg


def set_seed(seed, deterministic=False):
    random.seed(seed)  # Python 内置随机模块
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)  # 为当前GPU设置种子
    torch.cuda.manual_seed_all(seed)  # 如果使用多个GPU
    if deterministic:
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False


def quaternion_to_rotvec(quat):
    """
    将四元数 (w, x, y, z) 转换为旋转向量 (Rodrigues 旋转公式).

    参数:
        quat (array-like): 长度为 4 的数组，表示四元数 (w, x, y, z).

    返回:
        np.ndarray: 旋转向量 (3D).
    """
    # scipy 需要 [x, y, z, w] 格式，因此调整顺序
    rot = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
    return rot.as_rotvec()


def set_prim_transform(stage, prim_path, translate=None, rotate=None, scale=None):
    """
    修改指定 USD 元素的局部变换。

    参数：
        stage (Usd.Stage)：当前打开的 USD Stage。
        prim_path (str)：元素路径，例如 "/World/Cube"。
        translate (tuple or list of 3 floats, optional)：平移量 (x, y, z)。
        rotate (tuple or list of 3 floats, optional)：旋转角度 (X, Y, Z)，单位：度。
        scale (tuple or list of 3 floats, optional)：缩放比例 (X, Y, Z)。
    """
    prim = stage.GetPrimAtPath(prim_path)

    if not prim or not prim.IsValid():
        print(f"未找到 {prim_path}，请检查路径是否正确！")
        return

    xprim = XFormPrim(prim_path=prim_path)
    xprim.set_world_pose(translate, rotate)

# def draw_single_point(pos, color="expert", size=15):
#     """
#     实时绘制单个轨迹点。

#     Args:
#         pos: (x, y, z) 的三元组坐标
#         color: "expert"（绿色）或 "model"（蓝色），也支持自定义 RGB 元组
#         size: 点的大小
#     """
#     if color == "expert":
#         rgba = (0, 1, 0, 1)  # 绿色
#     elif color == "model":
#         rgba = (0, 0, 1, 1)  # 蓝色
#     elif isinstance(color, tuple) and len(color) in (3, 4):
#         if len(color) == 3:
#             rgba = (*color, 1.0)
#         else:
#             rgba = color
#     else:
#         raise ValueError("Unsupported color type")

#     _draw.draw_points([pos], [rgba], [size])