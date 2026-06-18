"""Genesis backend (https://github.com/Genesis-Embodied-AI/Genesis).

Genesis is a GPU-accelerated simulator and is **not runnable on a CPU-only
host** (this environment, standard CI). The adapter is written against the real
Genesis API and is exercised on a GPU host; here it is import-guarded so the
registry simply reports it unavailable rather than crashing.

Genesis exposes a batched env API; we use ``n_envs=1`` and squeeze the leading
batch dimension to fit the single-robot :class:`RobotState` contract. Genesis
loads the same quad12 MJCF as MuJoCo, so the morphology is identical.
"""

from __future__ import annotations

import numpy as np

from ..config import RobotCfg
from .base import Simulator
from .state import RobotState, quat_to_projected_gravity, world_velocities_to_base


class GenesisSimulator(Simulator):
    name = "genesis"

    def __init__(self) -> None:
        self._gs = None
        self.scene = None
        self.robot = None
        self._dof_idx: list[int] = []
        self._dt = 0.005
        self.robot_cfg: RobotCfg | None = None
        self._tau = None
        self._initialized = False

    @staticmethod
    def is_available() -> bool:
        # Require both the package and a CUDA device. Importing genesis is heavy
        # and may init the GPU, so we only probe specs here (no side effects).
        import importlib.util

        if importlib.util.find_spec("genesis") is None:
            return False
        try:
            import torch

            return bool(torch.cuda.is_available())
        except Exception:
            return False

    def load(self, robot_cfg: RobotCfg, *, render: bool = False, seed: int = 0) -> None:
        import genesis as gs

        self._gs = gs
        self.robot_cfg = robot_cfg
        if not self._initialized:
            gs.init(backend=gs.gpu, seed=seed)
            self._initialized = True

        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=self._dt),
            show_viewer=render,
        )
        self.scene.add_entity(gs.morphs.Plane())
        self.robot = self.scene.add_entity(
            gs.morphs.MJCF(file=robot_cfg.resolve(robot_cfg.mjcf_path))
        )
        self.scene.build(n_envs=1)

        # Map canonical joint names -> local DOF indices (enforce joint order).
        self._dof_idx = [self.robot.get_joint(name).dof_idx_local for name in robot_cfg.joint_names]
        self._tau = np.zeros(robot_cfg.n_dof, dtype=np.float32)

    @property
    def dt(self) -> float:
        return self._dt

    def total_mass(self) -> float:
        try:
            return float(self.robot.get_mass())
        except Exception:
            return 1.0

    def reset(self) -> RobotState:
        import torch

        q = torch.tensor(self.robot_cfg.default_joint_pos, dtype=torch.float32).unsqueeze(0)
        self.robot.set_dofs_position(q, self._dof_idx, envs_idx=[0])
        self.robot.set_pos(
            torch.tensor([[0.0, 0.0, self.robot_cfg.base_height_init]], dtype=torch.float32)
        )
        self.robot.set_quat(torch.tensor([[1.0, 0.0, 0.0, 0.0]], dtype=torch.float32))
        self.robot.zero_all_dofs_velocity()
        return self.get_state()

    def apply_torques(self, tau: np.ndarray) -> None:
        self._tau = np.asarray(tau, dtype=np.float32).ravel()

    def step(self) -> None:
        import torch

        t = torch.tensor(self._tau, dtype=torch.float32).unsqueeze(0)
        self.robot.control_dofs_force(t, self._dof_idx, envs_idx=[0])
        self.scene.step()

    def get_state(self) -> RobotState:
        base_pos = _np(self.robot.get_pos())[0]
        quat = _np(self.robot.get_quat())[0]  # (w, x, y, z)
        lin_world = _np(self.robot.get_vel())[0]
        ang_world = _np(self.robot.get_ang())[0]

        base_lin_vel, base_ang_vel = world_velocities_to_base(quat, lin_world, ang_world)

        joint_pos = _np(self.robot.get_dofs_position(self._dof_idx))[0]
        joint_vel = _np(self.robot.get_dofs_velocity(self._dof_idx))[0]
        return RobotState(
            base_pos=base_pos,
            base_quat=quat,
            base_lin_vel=base_lin_vel,
            base_ang_vel=base_ang_vel,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            projected_gravity=quat_to_projected_gravity(quat),
            sim_time=0.0,
        )


def _np(x) -> np.ndarray:
    """Convert a torch tensor (Genesis returns tensors) to a 2D-ish numpy array."""
    arr = x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)
    return np.atleast_2d(arr).astype(np.float32)
