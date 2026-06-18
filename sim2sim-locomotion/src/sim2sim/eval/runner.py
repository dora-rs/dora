"""Episode loop that ties policy <-> control <-> simulator together.

This is the one place the control frequency, observation assembly, and PD law
are orchestrated, so the experiment is identical on every backend. Adapters only
supply physics; the runner supplies the protocol.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..config import EvalCfg, PolicyCfg, RobotCfg
from ..control.actuation import PDController
from ..obs.commands import CommandGenerator
from ..obs.observation import ObservationBuilder
from ..policy.base import Policy
from ..sim.base import Simulator
from .metrics import EpisodeMetrics, _Accumulator, aggregate


@dataclass
class SimResult:
    sim: str
    episodes: list[EpisodeMetrics]

    @property
    def summary(self) -> dict[str, tuple[float, float]]:
        return aggregate(self.episodes)


def run_episode(
    sim: Simulator,
    policy: Policy,
    robot_cfg: RobotCfg,
    obs_builder: ObservationBuilder,
    cmd_gen: CommandGenerator,
    eval_cfg: EvalCfg,
    seed: int,
) -> EpisodeMetrics:
    """Run a single seeded episode and return its metrics."""
    rng = np.random.default_rng(seed)
    command = cmd_gen.sample(rng)

    policy.reset()
    obs_builder.reset()
    state = sim.reset()

    pd = PDController(robot_cfg)
    acc = _Accumulator(
        total_mass=sim.total_mass(),
        control_dt=eval_cfg.control_dt,
        fall_height=eval_cfg.fall_height,
        fall_tilt=eval_cfg.fall_tilt,
    )
    decimation = max(1, round(eval_cfg.control_dt / sim.dt))

    for _ in range(eval_cfg.max_steps):
        obs = obs_builder.build(state, command)
        action = policy.act(obs)
        obs_builder.set_last_action(action)

        # The policy chooses an action at the control rate; the PD torque is
        # recomputed at the (faster) physics rate from fresh joint feedback,
        # matching how legged_gym / Isaac Lab drive their decimation loop.
        torque = np.zeros(robot_cfg.n_dof, dtype=np.float32)
        for _ in range(decimation):
            torque = pd.compute_torque(action, state.joint_pos, state.joint_vel)
            sim.apply_torques(torque)
            sim.step()
            state = sim.get_state()

        acc.update(state, command, action, torque)
        if acc.fell:
            break

    return acc.finalize()


def run_simulator(
    sim_name: str,
    sim: Simulator,
    policy_factory,
    robot_cfg: RobotCfg,
    policy_cfg: PolicyCfg,
    eval_cfg: EvalCfg,
) -> SimResult:
    """Run all episodes for one simulator. ``policy_factory(obs_dim) -> Policy``."""
    sim.load(robot_cfg, render=eval_cfg.render, seed=0)
    obs_builder = ObservationBuilder(policy_cfg, robot_cfg)
    cmd_gen = CommandGenerator(policy_cfg)
    policy = policy_factory(obs_builder.dim)

    episodes: list[EpisodeMetrics] = []
    for seed in eval_cfg.seeds[: eval_cfg.episodes]:
        episodes.append(run_episode(sim, policy, robot_cfg, obs_builder, cmd_gen, eval_cfg, seed))
    sim.close()
    return SimResult(sim=sim_name, episodes=episodes)
