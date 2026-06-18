"""The action->torque PD law must be deterministic and identical everywhere."""

from __future__ import annotations

import numpy as np

from sim2sim.control.actuation import PDController


def test_pd_formula(robot_cfg):
    pd = PDController(robot_cfg)
    n = robot_cfg.n_dof
    action = np.zeros(n)
    q = robot_cfg.default_joint_pos.copy()
    qd = np.zeros(n)
    # At the default pose with zero action and zero velocity, torque is zero.
    tau = pd.compute_torque(action, q, qd)
    np.testing.assert_allclose(tau, np.zeros(n), atol=1e-6)


def test_pd_tracks_target(robot_cfg):
    pd = PDController(robot_cfg)
    n = robot_cfg.n_dof
    action = np.ones(n)
    q = robot_cfg.default_joint_pos.copy()
    qd = np.zeros(n)
    target = pd.action_to_target(action)
    np.testing.assert_allclose(target, robot_cfg.default_joint_pos + robot_cfg.action_scale)
    # tau = Kp*(target - q) since qd = 0
    expected = robot_cfg.kp * (target - q)
    expected = np.clip(expected, -robot_cfg.torque_limit, robot_cfg.torque_limit)
    np.testing.assert_allclose(pd.compute_torque(action, q, qd), expected, rtol=1e-5)


def test_torque_limit_clamp(robot_cfg):
    pd = PDController(robot_cfg)
    n = robot_cfg.n_dof
    # Huge positional error -> torque saturates at the limit.
    tau = pd.compute_torque(np.full(n, 100.0), np.zeros(n), np.zeros(n))
    assert np.all(np.abs(tau) <= robot_cfg.torque_limit + 1e-4)
    assert np.all(np.isclose(np.abs(tau), robot_cfg.torque_limit))
