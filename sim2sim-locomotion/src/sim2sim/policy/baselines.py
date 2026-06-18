"""Scripted baseline policies for sanity checks and tests.

These let the whole harness be exercised end-to-end without a trained network,
which is what the smoke tests and CI rely on.
"""

from __future__ import annotations

import numpy as np


class ZeroPolicy:
    """Always commands the default pose (action = 0)."""

    def __init__(self, action_dim: int):
        self.action_dim = int(action_dim)

    def reset(self) -> None:
        pass

    def act(self, obs: np.ndarray) -> np.ndarray:
        return np.zeros(self.action_dim, dtype=np.float32)


class RandomPolicy:
    """Small zero-mean Gaussian actions; seeded for reproducibility."""

    def __init__(self, action_dim: int, scale: float = 0.1, seed: int = 0):
        self.action_dim = int(action_dim)
        self.scale = float(scale)
        self._seed = int(seed)
        self._rng = np.random.default_rng(seed)

    def reset(self) -> None:
        # Re-seed each episode so runs are reproducible across simulators.
        self._rng = np.random.default_rng(self._seed)

    def act(self, obs: np.ndarray) -> np.ndarray:
        return (self._rng.standard_normal(self.action_dim) * self.scale).astype(np.float32)
