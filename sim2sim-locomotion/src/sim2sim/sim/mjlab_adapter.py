"""mjlab backend (https://github.com/mujocolab/mjlab).

mjlab is the GPU-accelerated simulator (MuJoCo-Warp / MJWarp, Isaac Lab-style
manager API) that the **LeRobot legged** stack — ``lerobot-legged-zoo`` and
``unitree_rl_mjlab`` — trains policies in. Including it as a backend makes this a
true sim-to-sim check: train in mjlab, then evaluate the exported policy here in
mjlab *and* MuJoCo/PyBullet/Genesis/Isaac.

mjlab **requires an NVIDIA GPU + CUDA** and is not runnable on a CPU host (this
environment, standard CI). The adapter targets the real ``mjlab.sim.Simulation``
API and is validated on a GPU host; here it is import-guarded so the registry
reports it unavailable rather than crashing.

We drive the low-level ``Simulation`` directly (num_envs=1) rather than the
manager-based RL env, so the shared observation/control pipeline is reused
unchanged. mjlab loads the same MJCF as MuJoCo, so the morphology is identical.
"""

from __future__ import annotations

import numpy as np

from ..config import RobotCfg
from .base import Simulator
from .state import RobotState, quat_to_projected_gravity, world_velocities_to_base


class MjlabSimulator(Simulator):
    name = "mjlab"

    def __init__(self) -> None:
        self.sim = None  # mjlab.sim.Simulation
        self._mj_model = None
        self.robot_cfg: RobotCfg | None = None
        self._qpos_adr: np.ndarray | None = None
        self._qvel_adr: np.ndarray | None = None
        self._ctrl_ids: np.ndarray | None = None
        self._dt = 0.005
        self._device = "cuda:0"
        self._tau = None

    @staticmethod
    def is_available() -> bool:
        import importlib.util

        if importlib.util.find_spec("mjlab") is None:
            return False
        try:
            import torch

            return bool(torch.cuda.is_available())
        except Exception:
            return False

    def load(self, robot_cfg: RobotCfg, *, render: bool = False, seed: int = 0) -> None:
        import mujoco
        from mjlab.sim.sim import MujocoCfg, Simulation, SimulationCfg

        self.robot_cfg = robot_cfg
        self._mj_model = mujoco.MjModel.from_xml_path(robot_cfg.resolve(robot_cfg.mjcf_path))
        self._dt = float(self._mj_model.opt.timestep)

        cfg = SimulationCfg(mujoco=MujocoCfg(timestep=self._dt))
        self.sim = Simulation(num_envs=1, cfg=cfg, model=self._mj_model, device=self._device)

        # Map canonical joint names -> qpos/qvel addresses and actuator ids, using
        # the host-side MjModel (enforces canonical joint order on every read).
        qpos_adr, qvel_adr, ctrl_ids = [], [], []
        for jname in robot_cfg.joint_names:
            jid = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise ValueError(f"joint '{jname}' not found in MJCF")
            qpos_adr.append(self._mj_model.jnt_qposadr[jid])
            qvel_adr.append(self._mj_model.jnt_dofadr[jid])
            aid = mujoco.mj_name2id(self._mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, jname)
            if aid < 0:
                raise ValueError(f"actuator '{jname}' not found in MJCF")
            ctrl_ids.append(aid)
        self._qpos_adr = np.array(qpos_adr, dtype=int)
        self._qvel_adr = np.array(qvel_adr, dtype=int)
        self._ctrl_ids = np.array(ctrl_ids, dtype=int)
        self._tau = np.zeros(robot_cfg.n_dof, dtype=np.float32)

    @property
    def dt(self) -> float:
        return self._dt

    def total_mass(self) -> float:
        return float(np.sum(self._mj_model.body_mass))

    def reset(self) -> RobotState:
        import torch

        self.sim.reset()
        qpos = self.sim.data.qpos
        qvel = self.sim.data.qvel
        # Free-joint base: [x y z, qw qx qy qz]; env index 0.
        base = torch.tensor(
            [0.0, 0.0, self.robot_cfg.base_height_init, 1.0, 0.0, 0.0, 0.0],
            device=self._device,
        )
        qpos[0, 0:7] = base
        jp = torch.tensor(self.robot_cfg.default_joint_pos, device=self._device)
        qpos[0, torch.as_tensor(self._qpos_adr, device=self._device)] = jp
        qvel[0, :] = 0.0
        self.sim.forward()
        return self.get_state()

    def apply_torques(self, tau: np.ndarray) -> None:
        self._tau = np.asarray(tau, dtype=np.float32).ravel()

    def step(self) -> None:
        import torch

        ctrl = self.sim.data.ctrl
        ctrl[0, torch.as_tensor(self._ctrl_ids, device=self._device)] = torch.tensor(
            self._tau, device=self._device
        )
        self.sim.step()

    def get_state(self) -> RobotState:
        qpos = _row0(self.sim.data.qpos)
        qvel = _row0(self.sim.data.qvel)
        base_pos = qpos[0:3].astype(np.float32)
        quat = qpos[3:7].astype(np.float32)  # (w, x, y, z)

        base_lin_vel, base_ang_vel = world_velocities_to_base(quat, qvel[0:3], qvel[3:6])
        joint_pos = qpos[self._qpos_adr].astype(np.float32)
        joint_vel = qvel[self._qvel_adr].astype(np.float32)
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


def _row0(arr) -> np.ndarray:
    """Read env-0 row of an mjlab data array (torch/warp-backed) as numpy."""
    row = arr[0]
    return row.detach().cpu().numpy() if hasattr(row, "detach") else np.asarray(row)
