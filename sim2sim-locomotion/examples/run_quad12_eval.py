"""Scripted equivalent of `sim2sim eval` — run the quad12 policy on every
available simulator and print + save the comparison report.

    python examples/run_quad12_eval.py

Use a trained policy by setting `onnx_path` in configs/policy/quad12_flat.yaml;
otherwise this falls back to a seeded random baseline so it always runs.
"""

from __future__ import annotations

from pathlib import Path

from sim2sim.config import EvalCfg
from sim2sim.eval.report import to_markdown, write_report
from sim2sim.eval.runner import run_simulator
from sim2sim.policy.baselines import RandomPolicy
from sim2sim.sim import registry

CONFIG = Path(__file__).resolve().parents[1] / "configs" / "eval.yaml"


def main() -> None:
    cfg = EvalCfg.from_yaml(CONFIG)
    robot_cfg = cfg.load_robot()
    policy_cfg = cfg.load_policy()

    # Prefer the configured ONNX policy; otherwise a random baseline.
    if policy_cfg.onnx_path:
        from sim2sim.policy.onnx_policy import OnnxPolicy

        def factory(obs_dim):
            return OnnxPolicy(
                policy_cfg.resolve(policy_cfg.onnx_path),
                expected_obs_dim=obs_dim,
                clip_actions=policy_cfg.clip_actions,
            )
    else:

        def factory(obs_dim):
            return RandomPolicy(robot_cfg.n_dof, scale=0.1, seed=0)

    results = []
    for name in cfg.sims:
        if not registry.is_available(name):
            print(f"[skip] {name}: unavailable on this host")
            continue
        print(f"[run ] {name} ...")
        results.append(
            run_simulator(name, registry.make(name), factory, robot_cfg, policy_cfg, cfg)
        )

    if results:
        print("\n" + to_markdown(results) + "\n")
        written = write_report(results, "report")
        print("wrote:", written)


if __name__ == "__main__":
    main()
