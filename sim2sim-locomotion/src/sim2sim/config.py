"""Configuration dataclasses and a tiny YAML loader.

Deliberately dependency-light (no hydra/pydantic): three frozen-ish dataclasses
plus ``from_yaml`` constructors. Configs are the single source of truth for the
robot model, the policy's observation contract, and the evaluation run, so the
*same* numbers drive every simulator.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml


def _pkg_assets_dir() -> Path:
    """Directory of bundled assets (src/sim2sim/assets)."""
    return Path(__file__).resolve().parent / "assets"


def _resolve_path(p: str, config_dir: str) -> str:
    """Resolve an asset reference.

    Supports a ``pkg://`` prefix that points into the installed package's
    ``assets/`` directory (so the bundled robot works regardless of cwd), an
    absolute path, or a path relative to the config file's directory.
    """
    if p.startswith("pkg://"):
        return str(_pkg_assets_dir() / p[len("pkg://") :])
    path = Path(p)
    return str(path if path.is_absolute() else Path(config_dir) / path)


def _load_yaml(path: str | os.PathLike) -> dict[str, Any]:
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a top-level mapping")
    return data


@dataclass
class RobotCfg:
    """Physical description of the robot, shared by every simulator adapter.

    ``joint_names`` defines the *canonical joint order*; adapters must report
    state and accept torques in this order. ``default_joint_pos`` is the nominal
    standing pose (the PD target offset and the observation reference).
    """

    name: str
    mjcf_path: str  # MuJoCo / Genesis
    urdf_path: str  # PyBullet (and Isaac via URDF->USD conversion)
    usd_path: str | None  # optional pre-converted USD for Isaac Lab
    joint_names: list[str]
    default_joint_pos: np.ndarray  # (n_dof,)
    kp: np.ndarray  # (n_dof,) proportional gains
    kd: np.ndarray  # (n_dof,) derivative gains
    action_scale: float
    torque_limit: np.ndarray | None  # (n_dof,) or None
    foot_names: list[str] = field(default_factory=list)
    base_height_init: float = 0.4
    config_dir: str = "."  # directory the config was loaded from (for path resolution)

    @property
    def n_dof(self) -> int:
        return len(self.joint_names)

    def resolve(self, p: str) -> str:
        """Resolve an asset path (supports ``pkg://``, absolute, or relative)."""
        return _resolve_path(p, self.config_dir)

    @classmethod
    def from_yaml(cls, path: str | os.PathLike) -> RobotCfg:
        d = _load_yaml(path)
        n = len(d["joint_names"])
        return cls(
            name=d["name"],
            mjcf_path=d["mjcf_path"],
            urdf_path=d["urdf_path"],
            usd_path=d.get("usd_path"),
            joint_names=list(d["joint_names"]),
            default_joint_pos=_as_vec(d["default_joint_pos"], n, "default_joint_pos"),
            kp=_as_vec(d["kp"], n, "kp"),
            kd=_as_vec(d["kd"], n, "kd"),
            action_scale=float(d["action_scale"]),
            torque_limit=(
                _as_vec(d["torque_limit"], n, "torque_limit")
                if d.get("torque_limit") is not None
                else None
            ),
            foot_names=list(d.get("foot_names", [])),
            base_height_init=float(d.get("base_height_init", 0.4)),
            config_dir=str(Path(path).resolve().parent),
        )


@dataclass
class ObsTerm:
    """One entry in the observation vector: a named term and its scale."""

    name: str
    scale: float = 1.0


@dataclass
class PolicyCfg:
    """The policy's observation contract + ONNX location.

    ``obs_terms`` defines the exact concatenation order and per-term scaling the
    policy was trained with. Reproducing it precisely is what makes a single
    ``.onnx`` behave identically across simulators.
    """

    onnx_path: str | None
    obs_terms: list[ObsTerm]
    command_ranges: dict[str, tuple[float, float]]  # lin_vel_x, lin_vel_y, ang_vel_z
    clip_actions: float | None = None
    config_dir: str = "."

    @classmethod
    def from_yaml(cls, path: str | os.PathLike) -> PolicyCfg:
        d = _load_yaml(path)
        terms = [ObsTerm(name=t["name"], scale=float(t.get("scale", 1.0))) for t in d["obs_terms"]]
        ranges = {k: (float(v[0]), float(v[1])) for k, v in d.get("command_ranges", {}).items()}
        return cls(
            onnx_path=d.get("onnx_path"),
            obs_terms=terms,
            command_ranges=ranges,
            clip_actions=(float(d["clip_actions"]) if d.get("clip_actions") is not None else None),
            config_dir=str(Path(path).resolve().parent),
        )

    def resolve(self, p: str) -> str:
        return _resolve_path(p, self.config_dir)


@dataclass
class EvalCfg:
    """Top-level evaluation run description."""

    robot: str  # path to robot yaml (relative to this file)
    policy: str  # path to policy yaml
    sims: list[str]  # e.g. ["mujoco", "pybullet"]
    episodes: int = 5
    max_steps: int = 1000
    control_dt: float = 0.02  # 50 Hz policy rate
    seeds: list[int] = field(default_factory=lambda: [0, 1, 2, 3, 4])
    fall_height: float = 0.18  # base below this (m) counts as a fall
    fall_tilt: float = 0.7  # projected_gravity_z above this magnitude => tilted over
    render: bool = False
    config_dir: str = "."

    @classmethod
    def from_yaml(cls, path: str | os.PathLike) -> EvalCfg:
        d = _load_yaml(path)
        cfg = cls(
            robot=d["robot"],
            policy=d["policy"],
            sims=list(d["sims"]),
            episodes=int(d.get("episodes", 5)),
            max_steps=int(d.get("max_steps", 1000)),
            control_dt=float(d.get("control_dt", 0.02)),
            seeds=list(d.get("seeds", [0, 1, 2, 3, 4])),
            fall_height=float(d.get("fall_height", 0.18)),
            fall_tilt=float(d.get("fall_tilt", 0.7)),
            render=bool(d.get("render", False)),
            config_dir=str(Path(path).resolve().parent),
        )
        return cfg

    def resolve(self, p: str) -> str:
        return _resolve_path(p, self.config_dir)

    def load_robot(self) -> RobotCfg:
        return RobotCfg.from_yaml(self.resolve(self.robot))

    def load_policy(self) -> PolicyCfg:
        return PolicyCfg.from_yaml(self.resolve(self.policy))


def _as_vec(x: Any, n: int, name: str) -> np.ndarray:
    arr = np.asarray(x, dtype=np.float32).ravel()
    if arr.shape[0] == 1 and n > 1:  # allow scalar broadcast
        arr = np.full(n, float(arr[0]), dtype=np.float32)
    if arr.shape[0] != n:
        raise ValueError(f"{name}: expected length {n}, got {arr.shape[0]}")
    return arr
