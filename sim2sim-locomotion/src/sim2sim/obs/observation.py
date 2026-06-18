"""Observation assembly — the heart of sim-to-sim parity.

A locomotion policy is trained against a specific observation layout: an ordered
concatenation of terms (base velocities, projected gravity, the velocity
command, joint deviations, joint velocities, the previous action, ...), each
multiplied by a term-specific scale. If any simulator builds this vector even
slightly differently, the "same" policy is no longer the same experiment.

:class:`ObservationBuilder` assembles that vector from a neutral
:class:`~sim2sim.sim.state.RobotState`, driven entirely by ``PolicyCfg``. It is
the only place observations are constructed, so all backends are guaranteed
identical inputs.
"""

from __future__ import annotations

import numpy as np

from ..config import PolicyCfg, RobotCfg
from ..sim.state import RobotState

# Registry of observation terms. Each maps to a function (state, ctx) -> 1D array
# *before* scaling. Add a term here and reference it by name in the policy YAML.
_TERMS = {
    "base_lin_vel": lambda s, c: s.base_lin_vel,
    "base_ang_vel": lambda s, c: s.base_ang_vel,
    "projected_gravity": lambda s, c: s.projected_gravity,
    "velocity_command": lambda s, c: c["command"],
    "joint_pos_rel": lambda s, c: s.joint_pos - c["default_joint_pos"],
    "joint_pos": lambda s, c: s.joint_pos,
    "joint_vel": lambda s, c: s.joint_vel,
    "last_action": lambda s, c: c["last_action"],
}


def available_terms() -> tuple[str, ...]:
    return tuple(_TERMS)


class ObservationBuilder:
    """Builds policy inputs deterministically from a RobotState + command.

    ``last_action`` is stateful (the previous policy output), so the builder is
    reset at the start of every episode via :meth:`reset`.
    """

    def __init__(self, policy_cfg: PolicyCfg, robot_cfg: RobotCfg):
        unknown = [t.name for t in policy_cfg.obs_terms if t.name not in _TERMS]
        if unknown:
            raise ValueError(f"unknown observation term(s) {unknown}; available: {list(_TERMS)}")
        self.terms = policy_cfg.obs_terms
        self.default_joint_pos = robot_cfg.default_joint_pos.astype(np.float32)
        self.n_dof = robot_cfg.n_dof
        self._last_action = np.zeros(self.n_dof, dtype=np.float32)

    def reset(self) -> None:
        self._last_action = np.zeros(self.n_dof, dtype=np.float32)

    def set_last_action(self, action: np.ndarray) -> None:
        self._last_action = np.asarray(action, dtype=np.float32).ravel()

    @property
    def dim(self) -> int:
        """Observation length implied by the configured terms (n_dof-aware)."""
        sizes = {
            "base_lin_vel": 3,
            "base_ang_vel": 3,
            "projected_gravity": 3,
            "velocity_command": 3,
            "joint_pos_rel": self.n_dof,
            "joint_pos": self.n_dof,
            "joint_vel": self.n_dof,
            "last_action": self.n_dof,
        }
        return sum(sizes[t.name] for t in self.terms)

    def build(self, state: RobotState, command: np.ndarray) -> np.ndarray:
        ctx = {
            "command": np.asarray(command, dtype=np.float32).ravel(),
            "default_joint_pos": self.default_joint_pos,
            "last_action": self._last_action,
        }
        parts = [
            np.asarray(_TERMS[t.name](state, ctx), dtype=np.float32).ravel() * t.scale
            for t in self.terms
        ]
        return np.concatenate(parts).astype(np.float32)
