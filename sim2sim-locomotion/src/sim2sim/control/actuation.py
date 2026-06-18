"""Shared action -> torque control law.

Every simulator is driven at the torque level through the *same* PD controller
so the action interpretation never differs between backends. Policies output
normalized joint position targets; we map them to absolute targets and then to
torques:

    target = default_joint_pos + action_scale * action
    tau    = Kp * (target - q) - Kd * qd      (optionally clamped)

Keeping this out of the adapters is essential: if MuJoCo used position servos
and PyBullet used torque control, the "same" policy would behave differently for
reasons that have nothing to do with the physics under test.
"""

from __future__ import annotations

import numpy as np

from ..config import RobotCfg


class PDController:
    def __init__(self, robot_cfg: RobotCfg):
        self.default = robot_cfg.default_joint_pos.astype(np.float32)
        self.kp = robot_cfg.kp.astype(np.float32)
        self.kd = robot_cfg.kd.astype(np.float32)
        self.action_scale = float(robot_cfg.action_scale)
        self.torque_limit = (
            robot_cfg.torque_limit.astype(np.float32)
            if robot_cfg.torque_limit is not None
            else None
        )

    def action_to_target(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action, dtype=np.float32).ravel()
        return self.default + self.action_scale * action

    def compute_torque(
        self, action: np.ndarray, joint_pos: np.ndarray, joint_vel: np.ndarray
    ) -> np.ndarray:
        target = self.action_to_target(action)
        tau = self.kp * (target - joint_pos) - self.kd * joint_vel
        if self.torque_limit is not None:
            tau = np.clip(tau, -self.torque_limit, self.torque_limit)
        return tau.astype(np.float32)
