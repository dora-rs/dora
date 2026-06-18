"""Config loading and the shared quaternion->projected-gravity helper."""

from __future__ import annotations

import math

import numpy as np

from sim2sim.sim.state import RobotState, quat_to_projected_gravity


def test_robot_cfg_loads(robot_cfg):
    assert robot_cfg.n_dof == 12
    assert len(robot_cfg.joint_names) == 12
    assert robot_cfg.kp.shape == (12,)  # scalar broadcast
    assert robot_cfg.default_joint_pos.shape == (12,)
    assert robot_cfg.resolve(robot_cfg.mjcf_path).endswith("quad12.xml")


def test_policy_cfg_loads(policy_cfg):
    names = [t.name for t in policy_cfg.obs_terms]
    assert names[0] == "base_lin_vel"
    assert "last_action" in names
    assert policy_cfg.command_ranges["lin_vel_x"] == (-1.0, 1.0)


def test_projected_gravity_identity():
    g = quat_to_projected_gravity(np.array([1.0, 0.0, 0.0, 0.0]))
    np.testing.assert_allclose(g, [0, 0, -1], atol=1e-6)


def test_projected_gravity_pitch_90():
    # +90 deg about y: gravity points along +x in the base frame.
    h = math.sqrt(0.5)
    g = quat_to_projected_gravity(np.array([h, 0.0, h, 0.0]))
    np.testing.assert_allclose(g, [1, 0, 0], atol=1e-6)


def test_projected_gravity_roll_90():
    # +90 deg about x: gravity points along -y in the base frame.
    h = math.sqrt(0.5)
    g = quat_to_projected_gravity(np.array([h, h, 0.0, 0.0]))
    np.testing.assert_allclose(g, [0, -1, 0], atol=1e-6)


def test_robot_state_enforces_shapes():
    s = RobotState(
        base_pos=[0, 0, 0.4],
        base_quat=[1, 0, 0, 0],
        base_lin_vel=[0, 0, 0],
        base_ang_vel=[0, 0, 0],
        joint_pos=np.zeros(12),
        joint_vel=np.zeros(12),
        projected_gravity=[0, 0, -1],
        sim_time=0.0,
    )
    assert s.n_dof == 12
    assert s.joint_pos.dtype == np.float32
