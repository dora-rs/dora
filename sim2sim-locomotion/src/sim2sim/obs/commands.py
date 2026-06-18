"""Velocity command generation for locomotion eval.

A command is ``[lin_vel_x, lin_vel_y, ang_vel_z]`` in the base frame — the
target the policy is asked to track. For reproducible cross-sim comparison the
command is sampled once per episode from the configured ranges using a seeded
RNG, then held constant, so every simulator chases the identical target.
"""

from __future__ import annotations

import numpy as np

from ..config import PolicyCfg


class CommandGenerator:
    def __init__(self, policy_cfg: PolicyCfg):
        r = policy_cfg.command_ranges
        self._ranges = (
            r.get("lin_vel_x", (0.0, 0.0)),
            r.get("lin_vel_y", (0.0, 0.0)),
            r.get("ang_vel_z", (0.0, 0.0)),
        )
        self._command = np.zeros(3, dtype=np.float32)

    def sample(self, rng: np.random.Generator) -> np.ndarray:
        self._command = np.array(
            [rng.uniform(lo, hi) for (lo, hi) in self._ranges], dtype=np.float32
        )
        return self._command

    @property
    def command(self) -> np.ndarray:
        return self._command
