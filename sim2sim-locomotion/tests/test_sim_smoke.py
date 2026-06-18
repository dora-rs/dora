"""End-to-end CPU smoke tests: a few hundred steps must run without NaNs and
produce finite metrics. Skipped automatically if a backend isn't installed."""

from __future__ import annotations

import numpy as np
import pytest
from conftest import REPO_ROOT

from sim2sim.config import EvalCfg
from sim2sim.eval.runner import run_simulator
from sim2sim.policy.baselines import RandomPolicy
from sim2sim.sim import registry

EVAL_CFG = REPO_ROOT / "configs" / "eval.yaml"


def _short_cfg() -> EvalCfg:
    cfg = EvalCfg.from_yaml(EVAL_CFG)
    cfg.episodes = 2
    cfg.max_steps = 50
    cfg.seeds = [0, 1]
    return cfg


@pytest.mark.parametrize("sim_name", ["mujoco", "pybullet"])
def test_backend_smoke(sim_name):
    if not registry.is_available(sim_name):
        pytest.skip(f"{sim_name} not installed")
    cfg = _short_cfg()
    robot_cfg = cfg.load_robot()
    policy_cfg = cfg.load_policy()
    sim = registry.make(sim_name)

    result = run_simulator(
        sim_name, sim, lambda d: RandomPolicy(robot_cfg.n_dof, 0.1, 0), robot_cfg, policy_cfg, cfg
    )
    assert len(result.episodes) == 2
    summary = result.summary
    for key, (mean, std) in summary.items():
        assert np.isfinite(mean), f"{sim_name}/{key} mean is not finite"
        assert np.isfinite(std), f"{sim_name}/{key} std is not finite"


def test_observation_state_no_nan_mujoco():
    if not registry.is_available("mujoco"):
        pytest.skip("mujoco not installed")
    cfg = _short_cfg()
    robot_cfg = cfg.load_robot()
    sim = registry.make("mujoco")
    sim.load(robot_cfg)
    state = sim.reset()
    assert not np.isnan(state.joint_pos).any()
    assert not np.isnan(state.projected_gravity).any()
    # Upright at reset: projected gravity ~ (0, 0, -1).
    np.testing.assert_allclose(state.projected_gravity, [0, 0, -1], atol=1e-5)
    sim.close()
