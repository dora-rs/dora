"""Per-episode locomotion metrics.

Metrics are accumulated step-by-step over an episode and reduced to scalars.
They are intentionally simulator-agnostic (computed from RobotState + command +
torque) so the same numbers are comparable across backends.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..sim.state import RobotState


@dataclass
class EpisodeMetrics:
    """Scalar summary of one episode."""

    steps: int
    survived: bool
    survival_time: float
    distance: float  # planar distance travelled by the base
    lin_vel_tracking_err: float  # mean |cmd_xy - actual_xy|
    ang_vel_tracking_err: float  # mean |cmd_yaw - actual_yaw|
    mean_torque: float  # mean |tau| over all joints/steps
    cost_of_transport: float  # mean mechanical power / (m g v)
    action_rate: float  # mean |a_t - a_{t-1}| (smoothness)

    def as_dict(self) -> dict[str, float]:
        return {
            "steps": float(self.steps),
            "survived": float(self.survived),
            "survival_time": self.survival_time,
            "distance": self.distance,
            "lin_vel_tracking_err": self.lin_vel_tracking_err,
            "ang_vel_tracking_err": self.ang_vel_tracking_err,
            "mean_torque": self.mean_torque,
            "cost_of_transport": self.cost_of_transport,
            "action_rate": self.action_rate,
        }


@dataclass
class _Accumulator:
    """Running accumulators for a single episode."""

    total_mass: float
    control_dt: float
    fall_height: float
    fall_tilt: float

    start_xy: np.ndarray | None = None
    last_xy: np.ndarray | None = None
    last_action: np.ndarray | None = None
    steps: int = 0
    lin_err_sum: float = 0.0
    ang_err_sum: float = 0.0
    torque_abs_sum: float = 0.0
    power_sum: float = 0.0
    action_rate_sum: float = 0.0
    speed_sum: float = 0.0
    fell: bool = False
    _grav_init: bool = field(default=False, repr=False)

    def update(
        self,
        state: RobotState,
        command: np.ndarray,
        action: np.ndarray,
        torque: np.ndarray,
    ) -> None:
        xy = state.base_pos[:2].astype(np.float64)
        if self.start_xy is None:
            self.start_xy = xy.copy()
        self.last_xy = xy

        # Velocity tracking error against the (vx, vy, yaw) command.
        lin_actual = state.base_lin_vel[:2]
        yaw_actual = state.base_ang_vel[2]
        self.lin_err_sum += float(np.linalg.norm(command[:2] - lin_actual))
        self.ang_err_sum += float(abs(command[2] - yaw_actual))
        self.speed_sum += float(np.linalg.norm(lin_actual))

        # Effort & mechanical cost of transport (|tau . qd|).
        self.torque_abs_sum += float(np.mean(np.abs(torque)))
        self.power_sum += float(np.sum(np.abs(torque * state.joint_vel)))

        # Action smoothness.
        if self.last_action is not None:
            self.action_rate_sum += float(np.mean(np.abs(action - self.last_action)))
        self.last_action = np.asarray(action, dtype=np.float64).copy()

        # Fall detection: base too low or tipped past threshold.
        if state.base_pos[2] < self.fall_height or state.projected_gravity[2] > -self.fall_tilt:
            self.fell = True

        self.steps += 1

    def finalize(self) -> EpisodeMetrics:
        steps = max(self.steps, 1)
        dist = (
            float(np.linalg.norm(self.last_xy - self.start_xy))
            if self.last_xy is not None and self.start_xy is not None
            else 0.0
        )
        mean_speed = self.speed_sum / steps
        g = 9.81
        mean_power = self.power_sum / steps
        denom = self.total_mass * g * max(mean_speed, 1e-3)
        cot = mean_power / denom
        return EpisodeMetrics(
            steps=self.steps,
            survived=not self.fell,
            survival_time=self.steps * self.control_dt,
            distance=dist,
            lin_vel_tracking_err=self.lin_err_sum / steps,
            ang_vel_tracking_err=self.ang_err_sum / steps,
            mean_torque=self.torque_abs_sum / steps,
            cost_of_transport=cot,
            action_rate=self.action_rate_sum / max(steps - 1, 1),
        )


def aggregate(episodes: list[EpisodeMetrics]) -> dict[str, tuple[float, float]]:
    """Reduce a list of episodes to mean/std per metric."""
    if not episodes:
        return {}
    keys = episodes[0].as_dict().keys()
    out: dict[str, tuple[float, float]] = {}
    for k in keys:
        vals = np.array([e.as_dict()[k] for e in episodes], dtype=np.float64)
        out[k] = (float(vals.mean()), float(vals.std()))
    return out
