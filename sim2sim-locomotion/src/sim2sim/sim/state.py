"""Neutral, simulator-agnostic robot state.

This is the *decoupling layer* of the whole framework: every simulator adapter
converts its native state into a :class:`RobotState`, and every downstream
consumer (observation builder, metrics) reads only from :class:`RobotState`.
Because no consumer ever touches a simulator API, the policy sees an identical
pipeline regardless of which backend produced the numbers.

All quantities use SI units. Quaternions are ``(w, x, y, z)``. Base-frame
velocities are expressed in the robot base frame (not the world frame), which is
what locomotion policies are almost always trained on.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class RobotState:
    """A snapshot of the robot, normalized across simulators.

    The single most common sim-to-sim bug is a joint-ordering mismatch: each
    simulator enumerates joints in its own order. Adapters MUST return
    ``joint_pos`` / ``joint_vel`` in the canonical order defined by
    ``RobotCfg.joint_names`` so that everything downstream lines up.
    """

    base_pos: np.ndarray  # (3,) world-frame position of the base
    base_quat: np.ndarray  # (4,) orientation as (w, x, y, z)
    base_lin_vel: np.ndarray  # (3,) linear velocity in the BASE frame
    base_ang_vel: np.ndarray  # (3,) angular velocity in the BASE frame
    joint_pos: np.ndarray  # (n_dof,) in canonical joint order
    joint_vel: np.ndarray  # (n_dof,) in canonical joint order
    projected_gravity: np.ndarray  # (3,) gravity unit vector in the BASE frame
    sim_time: float  # seconds since reset
    contacts: np.ndarray | None = field(default=None)  # (n_feet,) bool, optional

    def __post_init__(self) -> None:
        # Coerce to float32 contiguous arrays so observation assembly is cheap
        # and bit-stable across backends (numpy promotion rules otherwise differ
        # depending on whether a sim handed us float32 or float64).
        self.base_pos = _vec(self.base_pos, 3)
        self.base_quat = _vec(self.base_quat, 4)
        self.base_lin_vel = _vec(self.base_lin_vel, 3)
        self.base_ang_vel = _vec(self.base_ang_vel, 3)
        self.projected_gravity = _vec(self.projected_gravity, 3)
        self.joint_pos = np.ascontiguousarray(self.joint_pos, dtype=np.float32).ravel()
        self.joint_vel = np.ascontiguousarray(self.joint_vel, dtype=np.float32).ravel()
        if self.joint_pos.shape != self.joint_vel.shape:
            raise ValueError(
                f"joint_pos {self.joint_pos.shape} and joint_vel "
                f"{self.joint_vel.shape} must have the same length"
            )
        if self.contacts is not None:
            self.contacts = np.ascontiguousarray(self.contacts, dtype=bool).ravel()

    @property
    def n_dof(self) -> int:
        return int(self.joint_pos.shape[0])


def _vec(x: np.ndarray, n: int) -> np.ndarray:
    arr = np.ascontiguousarray(x, dtype=np.float32).ravel()
    if arr.shape[0] != n:
        raise ValueError(f"expected a length-{n} vector, got shape {arr.shape}")
    return arr


def quat_to_rotation_matrix(quat_wxyz: np.ndarray) -> np.ndarray:
    """(w, x, y, z) -> 3x3 base->world rotation matrix.

    Shared by every adapter so velocity-frame conversions are identical.
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )


def world_velocities_to_base(
    quat_wxyz: np.ndarray, lin_world: np.ndarray, ang_world: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate world-frame linear/angular velocities into the base frame.

    Locomotion policies are trained on base-frame velocities; adapters call this
    to convert their native world-frame readings identically.
    """
    rot = quat_to_rotation_matrix(quat_wxyz)
    lin = (rot.T @ np.asarray(lin_world, dtype=np.float32)).astype(np.float32)
    ang = (rot.T @ np.asarray(ang_world, dtype=np.float32)).astype(np.float32)
    return lin, ang


def quat_to_projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
    """Rotate the world gravity unit vector (0, 0, -1) into the base frame.

    Shared helper so every adapter computes ``projected_gravity`` identically
    instead of each reimplementing the quaternion math (another classic source
    of cross-sim drift). ``quat`` is ``(w, x, y, z)``.
    """
    w, x, y, z = (float(v) for v in quat_wxyz)
    # g_base = R(q)^T @ g_world with g_world = (0, 0, -1), i.e. minus the third
    # row of the base->world rotation matrix. Verified: identity quat -> (0,0,-1);
    # a +90 deg pitch about y -> (1, 0, 0).
    gx = 2.0 * (w * y - x * z)
    gy = -2.0 * (y * z + w * x)
    gz = 2.0 * (x * x + y * y) - 1.0
    return np.array([gx, gy, gz], dtype=np.float32)
