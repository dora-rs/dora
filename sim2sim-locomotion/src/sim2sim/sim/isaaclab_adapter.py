"""Isaac Lab backend (https://isaac-sim.github.io/IsaacLab/).

Isaac Lab runs on the NVIDIA Omniverse / Isaac Sim stack and requires an NVIDIA
GPU. It is installed out-of-band (not via this project's pip extras) and is
**not runnable on a CPU-only host** (this environment, standard CI). The adapter
targets the Isaac Lab ``Articulation`` API and is validated only on a GPU host;
here it is import-guarded so the registry reports it unavailable.

Key Isaac Lab specifics handled below:
- The Omniverse app MUST be launched (``AppLauncher``) before importing any
  ``isaaclab`` simulation modules, so those imports live inside ``load()``.
- Tensors are GPU torch tensors with a leading env dimension; we use a single
  environment and squeeze it.
- A USD asset is required. If ``robot_cfg.usd_path`` is unset we convert the
  bundled URDF to USD via Isaac Lab's ``UrdfConverter``.
- Isaac quaternions are ``(w, x, y, z)`` (matching this framework).
"""

from __future__ import annotations

import numpy as np

from ..config import RobotCfg
from .base import Simulator
from .state import RobotState, quat_to_projected_gravity, world_velocities_to_base


class IsaacLabSimulator(Simulator):
    name = "isaaclab"

    def __init__(self) -> None:
        self._app = None
        self.sim = None
        self.robot = None  # isaaclab Articulation
        self._dt = 0.005
        self._dof_order: list[int] | None = None  # canonical -> Isaac dof index
        self.robot_cfg: RobotCfg | None = None
        self._tau = None

    @staticmethod
    def is_available() -> bool:
        import importlib.util

        if importlib.util.find_spec("isaaclab") is None:
            return False
        try:
            import torch

            return bool(torch.cuda.is_available())
        except Exception:
            return False

    def load(self, robot_cfg: RobotCfg, *, render: bool = False, seed: int = 0) -> None:
        # 1) Launch the Omniverse app FIRST (headless unless rendering).
        from isaaclab.app import AppLauncher

        self._app = AppLauncher(headless=not render).app

        # 2) Now Isaac sim modules are importable.
        import isaaclab.sim as sim_utils
        from isaaclab.assets import Articulation, ArticulationCfg

        self.robot_cfg = robot_cfg
        self.sim = sim_utils.SimulationContext(
            sim_utils.SimulationCfg(dt=self._dt, device="cuda:0")
        )
        sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())

        usd_path = self._ensure_usd(robot_cfg, sim_utils)
        robot_cfg_il = ArticulationCfg(
            prim_path="/World/Robot",
            spawn=sim_utils.UsdFileCfg(usd_path=usd_path),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.0, robot_cfg.base_height_init),
                joint_pos={
                    name: float(q)
                    for name, q in zip(
                        robot_cfg.joint_names, robot_cfg.default_joint_pos, strict=True
                    )
                },
            ),
        )
        self.robot = Articulation(robot_cfg_il)
        self.sim.reset()

        # Map canonical joint order -> Isaac's internal dof ordering.
        isaac_names = self.robot.joint_names
        self._dof_order = [isaac_names.index(n) for n in robot_cfg.joint_names]
        self._tau = np.zeros(robot_cfg.n_dof, dtype=np.float32)

    def _ensure_usd(self, robot_cfg: RobotCfg, sim_utils) -> str:
        if robot_cfg.usd_path:
            return robot_cfg.resolve(robot_cfg.usd_path)
        # Convert the bundled URDF to USD on the fly.
        from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

        cfg = UrdfConverterCfg(
            asset_path=robot_cfg.resolve(robot_cfg.urdf_path),
            usd_dir="/tmp/sim2sim_isaac",
            fix_base=False,
        )
        return UrdfConverter(cfg).usd_path

    @property
    def dt(self) -> float:
        return self._dt

    def total_mass(self) -> float:
        try:
            return float(self.robot.root_physx_view.get_masses().sum())
        except Exception:
            return 1.0

    def reset(self) -> RobotState:
        self.robot.reset()
        # Push the configured default state into the sim buffers.
        self.robot.write_data_to_sim()
        self.sim.step(render=False)
        self.robot.update(self._dt)
        return self.get_state()

    def apply_torques(self, tau: np.ndarray) -> None:
        self._tau = np.asarray(tau, dtype=np.float32).ravel()

    def step(self) -> None:
        import torch

        # Reorder canonical torques into Isaac dof order before applying.
        tau_isaac = np.zeros_like(self._tau)
        for canon_i, isaac_i in enumerate(self._dof_order):
            tau_isaac[isaac_i] = self._tau[canon_i]
        t = torch.tensor(tau_isaac, dtype=torch.float32, device="cuda:0").unsqueeze(0)
        self.robot.set_joint_effort_target(t)
        self.robot.write_data_to_sim()
        self.sim.step(render=False)
        self.robot.update(self._dt)

    def get_state(self) -> RobotState:
        data = self.robot.data
        root = _np(data.root_state_w)[0]  # [px py pz, qw qx qy qz, vx vy vz, wx wy wz]
        base_pos = root[0:3]
        quat = root[3:7]
        lin_world = root[7:10]
        ang_world = root[10:13]

        base_lin_vel, base_ang_vel = world_velocities_to_base(quat, lin_world, ang_world)

        # Reorder Isaac dof state back into canonical joint order.
        jp_isaac = _np(data.joint_pos)[0]
        jv_isaac = _np(data.joint_vel)[0]
        joint_pos = np.array([jp_isaac[i] for i in self._dof_order], dtype=np.float32)
        joint_vel = np.array([jv_isaac[i] for i in self._dof_order], dtype=np.float32)
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

    def close(self) -> None:
        if self._app is not None:
            try:
                self._app.close()
            except Exception:
                pass
            self._app = None


def _np(x) -> np.ndarray:
    arr = x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)
    return np.atleast_2d(arr).astype(np.float32)
