"""The observation contract is the crux of sim-to-sim: identical state in,
identical observation out, in a known order with known scales."""

from __future__ import annotations

import numpy as np

from sim2sim.obs.observation import ObservationBuilder, available_terms
from sim2sim.sim.state import RobotState


def _make_state(n_dof: int) -> RobotState:
    return RobotState(
        base_pos=np.array([1.0, 2.0, 0.4]),
        base_quat=np.array([1.0, 0.0, 0.0, 0.0]),
        base_lin_vel=np.array([0.5, 0.0, 0.0]),
        base_ang_vel=np.array([0.0, 0.0, 0.3]),
        joint_pos=np.linspace(0.0, 1.0, n_dof),
        joint_vel=np.full(n_dof, 0.2),
        projected_gravity=np.array([0.0, 0.0, -1.0]),
        sim_time=0.1,
    )


def test_dim_matches_concatenation(policy_cfg, robot_cfg):
    ob = ObservationBuilder(policy_cfg, robot_cfg)
    state = _make_state(robot_cfg.n_dof)
    obs = ob.build(state, np.array([0.5, 0.0, 0.3]))
    assert obs.shape == (ob.dim,)
    assert obs.dtype == np.float32
    # 3+3+3+3 + 12+12+12 for the quad12_flat layout
    assert ob.dim == 4 * 3 + 3 * robot_cfg.n_dof


def test_deterministic_and_ordered(policy_cfg, robot_cfg):
    ob = ObservationBuilder(policy_cfg, robot_cfg)
    state = _make_state(robot_cfg.n_dof)
    cmd = np.array([0.5, 0.0, 0.3])
    o1 = ob.build(state, cmd)
    ob.reset()
    o2 = ob.build(state, cmd)
    np.testing.assert_array_equal(o1, o2)

    # Hand-check the leading terms against the configured scales (2.0, 0.25).
    np.testing.assert_allclose(o1[0:3], state.base_lin_vel * 2.0, rtol=1e-6)
    np.testing.assert_allclose(o1[3:6], state.base_ang_vel * 0.25, rtol=1e-6)
    np.testing.assert_allclose(o1[6:9], state.projected_gravity, rtol=1e-6)
    np.testing.assert_allclose(o1[9:12], cmd, rtol=1e-6)


def test_joint_pos_rel_uses_default(policy_cfg, robot_cfg):
    ob = ObservationBuilder(policy_cfg, robot_cfg)
    state = _make_state(robot_cfg.n_dof)
    o = ob.build(state, np.zeros(3))
    n = robot_cfg.n_dof
    start = 12  # after the four 3-vectors
    expected = state.joint_pos - robot_cfg.default_joint_pos
    np.testing.assert_allclose(o[start : start + n], expected, rtol=1e-5)


def test_last_action_is_tracked(policy_cfg, robot_cfg):
    ob = ObservationBuilder(policy_cfg, robot_cfg)
    state = _make_state(robot_cfg.n_dof)
    n = robot_cfg.n_dof
    # last_action is the final term; starts at zero after reset.
    o0 = ob.build(state, np.zeros(3))
    np.testing.assert_array_equal(o0[-n:], np.zeros(n))
    act = np.arange(n, dtype=np.float32)
    ob.set_last_action(act)
    o1 = ob.build(state, np.zeros(3))
    np.testing.assert_array_equal(o1[-n:], act)


def test_unknown_term_rejected(robot_cfg):
    from sim2sim.config import ObsTerm, PolicyCfg

    bad = PolicyCfg(onnx_path=None, obs_terms=[ObsTerm("not_a_term")], command_ranges={})
    try:
        ObservationBuilder(bad, robot_cfg)
        raise AssertionError("expected ValueError for unknown term")
    except ValueError as e:
        assert "not_a_term" in str(e)


def test_available_terms_nonempty():
    assert "base_lin_vel" in available_terms()
    assert "last_action" in available_terms()
