"""Command-line interface: ``sim2sim eval`` and ``sim2sim list-sims``."""

from __future__ import annotations

import argparse
import sys

from .config import EvalCfg, PolicyCfg, RobotCfg
from .eval.report import to_markdown, write_report
from .eval.runner import run_simulator
from .sim import registry


def _make_policy_factory(policy_cfg: PolicyCfg, robot_cfg: RobotCfg, kind: str):
    """Return a callable ``(obs_dim) -> Policy`` for the requested policy kind.

    'auto' uses the ONNX policy when ``onnx_path`` is set, else a RandomPolicy so
    the harness is runnable without a trained network.
    """
    n_dof = robot_cfg.n_dof

    def factory(obs_dim: int):
        from .policy.baselines import RandomPolicy, ZeroPolicy

        resolved = kind
        if resolved == "auto":
            resolved = "onnx" if policy_cfg.onnx_path else "random"

        if resolved == "onnx":
            if not policy_cfg.onnx_path:
                raise SystemExit("policy kind 'onnx' requires onnx_path in the policy config")
            from .policy.onnx_policy import OnnxPolicy

            return OnnxPolicy(
                policy_cfg.resolve(policy_cfg.onnx_path),
                expected_obs_dim=obs_dim,
                clip_actions=policy_cfg.clip_actions,
            )
        if resolved == "zero":
            return ZeroPolicy(n_dof)
        if resolved == "random":
            return RandomPolicy(n_dof, scale=0.1, seed=0)
        raise SystemExit(f"unknown policy kind '{resolved}'")

    return factory


def cmd_eval(args: argparse.Namespace) -> int:
    eval_cfg = EvalCfg.from_yaml(args.config)
    if args.sims:
        eval_cfg.sims = [s.strip() for s in args.sims.split(",") if s.strip()]
    robot_cfg = eval_cfg.load_robot()
    policy_cfg = eval_cfg.load_policy()
    factory = _make_policy_factory(policy_cfg, robot_cfg, args.policy)

    avail = registry.availability(eval_cfg.sims)
    runnable = [s for s in eval_cfg.sims if avail.get(s)]
    skipped = [s for s in eval_cfg.sims if not avail.get(s)]
    for s in skipped:
        print(f"[skip] simulator '{s}' is not available here (deps/GPU missing)", file=sys.stderr)
    if not runnable:
        print("No requested simulators are available.", file=sys.stderr)
        return 2

    results = []
    for s in runnable:
        print(f"[run ] {s}: {eval_cfg.episodes} episode(s) ...", file=sys.stderr)
        sim = registry.make(s)
        results.append(run_simulator(s, sim, factory, robot_cfg, policy_cfg, eval_cfg))

    print("\n" + to_markdown(results) + "\n")
    written = write_report(results, args.out, make_plots=not args.no_plots)
    for kind, path in written.items():
        print(f"[wrote] {kind}: {path}", file=sys.stderr)
    return 0


def cmd_list_sims(args: argparse.Namespace) -> int:
    names = registry.registered_names()
    avail = registry.availability(list(names))
    print("Registered simulators:")
    for n in names:
        status = "available" if avail[n] else "unavailable (deps/GPU missing)"
        print(f"  {n:10s} {status}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="sim2sim", description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)

    pe = sub.add_parser("eval", help="evaluate a policy across simulators")
    pe.add_argument("--config", required=True, help="path to eval YAML")
    pe.add_argument("--out", default="report", help="output directory for the report")
    pe.add_argument("--sims", default=None, help="comma-separated override of sims list")
    pe.add_argument(
        "--policy",
        default="auto",
        choices=["auto", "onnx", "random", "zero"],
        help="policy kind (auto: onnx if configured, else random)",
    )
    pe.add_argument("--no-plots", action="store_true", help="skip matplotlib plots")
    pe.set_defaults(func=cmd_eval)

    pl = sub.add_parser("list-sims", help="show registered simulators and availability")
    pl.set_defaults(func=cmd_list_sims)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
