"""The LeRobot Humanoid legs robot must load + roll out on the CPU backends and
share the canonical joint contract across them."""

from __future__ import annotations

import numpy as np
import pytest
from conftest import REPO_ROOT

from sim2sim.config import EvalCfg, PolicyCfg, RobotCfg
from sim2sim.eval.runner import run_simulator
from sim2sim.obs.observation import ObservationBuilder
from sim2sim.policy.baselines import ZeroPolicy
from sim2sim.sim import registry

ROBOT = REPO_ROOT / "configs" / "robot" / "lerobot_legs.yaml"
POLICY = REPO_ROOT / "configs" / "policy" / "lerobot_legs_flat.yaml"
EVAL = REPO_ROOT / "configs" / "eval_lerobot.yaml"


def test_config_loads():
    robot = RobotCfg.from_yaml(ROBOT)
    assert robot.n_dof == 12
    assert robot.joint_names[0] == "L_hip_yaw"
    assert robot.resolve(robot.mjcf_path).endswith("lerobot_legs.xml")
    assert robot.resolve(robot.urdf_path).endswith("lerobot_legs.urdf")
    pol = PolicyCfg.from_yaml(POLICY)
    assert ObservationBuilder(pol, robot).dim == 48


def test_eval_config_lists_mjlab():
    cfg = EvalCfg.from_yaml(EVAL)
    assert "mjlab" in cfg.sims  # the LeRobot legged training simulator


@pytest.mark.parametrize("sim_name", ["mujoco", "pybullet"])
def test_lerobot_legs_rolls_out(sim_name):
    if not registry.is_available(sim_name):
        pytest.skip(f"{sim_name} not installed")
    robot = RobotCfg.from_yaml(ROBOT)
    policy_cfg = PolicyCfg.from_yaml(POLICY)
    cfg = EvalCfg.from_yaml(EVAL)
    cfg.episodes, cfg.max_steps, cfg.seeds = 1, 40, [0]

    result = run_simulator(
        sim_name, registry.make(sim_name), lambda d: ZeroPolicy(robot.n_dof), robot, policy_cfg, cfg
    )
    assert len(result.episodes) == 1
    for _key, (mean, std) in result.summary.items():
        assert np.isfinite(mean) and np.isfinite(std)
