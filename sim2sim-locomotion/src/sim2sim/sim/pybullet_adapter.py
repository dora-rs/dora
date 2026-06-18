"""PyBullet backend.

CPU, pip-installable (``pip install pybullet``). Loads the same quad12 robot
from URDF so the policy faces an identical morphology to MuJoCo. We disable
PyBullet's default velocity motors and drive pure torque control, matching the
shared PD law applied in the runner.

PyBullet quaternions are ``(x, y, z, w)``; we convert to the framework's
``(w, x, y, z)`` convention on the way out.
"""

from __future__ import annotations

import numpy as np

from ..config import RobotCfg
from .base import Simulator
from .state import RobotState, quat_to_projected_gravity, world_velocities_to_base


class PybulletSimulator(Simulator):
    name = "pybullet"

    def __init__(self) -> None:
        self._bc = None
        self.robot_id: int | None = None
        self._joint_ids: list[int] = []
        self._dt = 0.005
        self.robot_cfg: RobotCfg | None = None
        self._render = False

    @staticmethod
    def is_available() -> bool:
        import importlib.util

        return importlib.util.find_spec("pybullet") is not None

    def load(self, robot_cfg: RobotCfg, *, render: bool = False, seed: int = 0) -> None:
        import pybullet as p
        from pybullet_utils import bullet_client

        self.robot_cfg = robot_cfg
        self._render = render
        mode = p.GUI if render else p.DIRECT
        self._bc = bullet_client.BulletClient(connection_mode=mode)
        bc = self._bc
        bc.setGravity(0, 0, -9.81)
        bc.setTimeStep(self._dt)
        _load_ground(bc)

        self.robot_id = bc.loadURDF(
            robot_cfg.resolve(robot_cfg.urdf_path),
            basePosition=[0, 0, robot_cfg.base_height_init],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=False,
            flags=p.URDF_USE_INERTIA_FROM_FILE,
        )

        # Map canonical joint names -> pybullet joint indices (enforce order).
        name_to_idx = {}
        for j in range(bc.getNumJoints(self.robot_id)):
            info = bc.getJointInfo(self.robot_id, j)
            name_to_idx[info[1].decode()] = j
        self._joint_ids = []
        for jname in robot_cfg.joint_names:
            if jname not in name_to_idx:
                raise ValueError(f"joint '{jname}' not found in URDF")
            self._joint_ids.append(name_to_idx[jname])

        # Disable the default position/velocity motors so torque control is pure.
        for jid in self._joint_ids:
            bc.setJointMotorControl2(self.robot_id, jid, p.VELOCITY_CONTROL, force=0.0)

    @property
    def dt(self) -> float:
        return self._dt

    def total_mass(self) -> float:
        bc = self._bc
        mass = bc.getDynamicsInfo(self.robot_id, -1)[0]  # base link
        for j in range(bc.getNumJoints(self.robot_id)):
            mass += bc.getDynamicsInfo(self.robot_id, j)[0]
        return float(mass)

    def reset(self) -> RobotState:
        bc = self._bc
        bc.resetBasePositionAndOrientation(
            self.robot_id, [0, 0, self.robot_cfg.base_height_init], [0, 0, 0, 1]
        )
        bc.resetBaseVelocity(self.robot_id, [0, 0, 0], [0, 0, 0])
        for jid, q in zip(self._joint_ids, self.robot_cfg.default_joint_pos, strict=True):
            bc.resetJointState(self.robot_id, jid, float(q), 0.0)
        return self.get_state()

    def apply_torques(self, tau: np.ndarray) -> None:
        import pybullet as p

        tau = np.asarray(tau, dtype=np.float64).ravel()
        self._bc.setJointMotorControlArray(
            self.robot_id, self._joint_ids, p.TORQUE_CONTROL, forces=tau.tolist()
        )

    def step(self) -> None:
        self._bc.stepSimulation()

    def get_state(self) -> RobotState:
        bc = self._bc
        pos, orn_xyzw = bc.getBasePositionAndOrientation(self.robot_id)
        lin_world, ang_world = bc.getBaseVelocity(self.robot_id)
        quat = np.array(
            [orn_xyzw[3], orn_xyzw[0], orn_xyzw[1], orn_xyzw[2]], dtype=np.float32
        )  # -> (w, x, y, z)

        base_lin_vel, base_ang_vel = world_velocities_to_base(quat, lin_world, ang_world)

        states = bc.getJointStates(self.robot_id, self._joint_ids)
        joint_pos = np.array([s[0] for s in states], dtype=np.float32)
        joint_vel = np.array([s[1] for s in states], dtype=np.float32)
        return RobotState(
            base_pos=np.asarray(pos, dtype=np.float32),
            base_quat=quat,
            base_lin_vel=base_lin_vel,
            base_ang_vel=base_ang_vel,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            projected_gravity=quat_to_projected_gravity(quat),
            sim_time=0.0,  # pybullet has no intrinsic clock; runner tracks time
        )

    def close(self) -> None:
        if self._bc is not None:
            try:
                self._bc.disconnect()
            except Exception:
                pass
            self._bc = None


def _load_ground(bc) -> None:
    """Add a static ground plane, preferring bundled plane.urdf, else a GEOM_PLANE."""
    import pybullet as p

    try:
        import pybullet_data

        bc.setAdditionalSearchPath(pybullet_data.getDataPath())
        bc.loadURDF("plane.urdf")
    except Exception:
        col = bc.createCollisionShape(p.GEOM_PLANE)
        bc.createMultiBody(0, col)
