"""The simulator adapter contract.

Adapters are deliberately *thin*: load a model, accept joint torques, advance
one physics step, and report a neutral :class:`RobotState`. The episode loop,
control law, and observation assembly all live above this interface (in the
runner / control / obs modules), which is what guarantees the policy sees an
identical pipeline no matter which backend is underneath.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from ..config import RobotCfg
from .state import RobotState


class Simulator(ABC):
    name: str = "abstract"

    @abstractmethod
    def load(self, robot_cfg: RobotCfg, *, render: bool = False, seed: int = 0) -> None:
        """Construct the physics world and load the robot. Called once."""

    @abstractmethod
    def reset(self) -> RobotState:
        """Reset the robot to its initial standing pose; return the first state."""

    @abstractmethod
    def apply_torques(self, tau: np.ndarray) -> None:
        """Set the joint torques to apply on the next :meth:`step`."""

    @abstractmethod
    def step(self) -> None:
        """Advance the simulation by exactly :attr:`dt` seconds."""

    @abstractmethod
    def get_state(self) -> RobotState:
        """Return the current robot state in canonical joint order."""

    @property
    @abstractmethod
    def dt(self) -> float:
        """Physics timestep in seconds."""

    def total_mass(self) -> float:
        """Total robot mass (kg), used for cost-of-transport. Override per backend."""
        return 1.0

    def close(self) -> None:  # noqa: B027 - intentional optional cleanup hook
        pass

    @staticmethod
    def is_available() -> bool:
        """Whether this backend can run here (deps importable, hardware present).

        Must NOT initialize a GPU or load heavy modules with side effects — it is
        called by the registry just to decide which simulators to run.
        """
        return False
