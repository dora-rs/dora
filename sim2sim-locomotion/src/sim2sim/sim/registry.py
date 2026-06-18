"""Simulator registry with availability probing.

Maps backend names to adapter classes *lazily* — importing this module must not
import mujoco/pybullet/genesis/isaac. The adapter class is only imported when
asked for, and each adapter's ``is_available()`` probes its dependency without
side effects so the CLI can report which requested sims can actually run.
"""

from __future__ import annotations

from importlib import import_module

from .base import Simulator

# name -> (module, class). Import is deferred until make()/availability().
_REGISTRY: dict[str, tuple[str, str]] = {
    "mujoco": ("sim2sim.sim.mujoco_adapter", "MujocoSimulator"),
    "pybullet": ("sim2sim.sim.pybullet_adapter", "PybulletSimulator"),
    "mjlab": ("sim2sim.sim.mjlab_adapter", "MjlabSimulator"),
    "genesis": ("sim2sim.sim.genesis_adapter", "GenesisSimulator"),
    "isaaclab": ("sim2sim.sim.isaaclab_adapter", "IsaacLabSimulator"),
}


def registered_names() -> tuple[str, ...]:
    return tuple(_REGISTRY)


def get_class(name: str) -> type[Simulator]:
    if name not in _REGISTRY:
        raise KeyError(f"unknown simulator '{name}'; known: {list(_REGISTRY)}")
    module_name, cls_name = _REGISTRY[name]
    module = import_module(module_name)
    return getattr(module, cls_name)


def is_available(name: str) -> bool:
    """True if the backend's deps import and any required hardware is present."""
    try:
        return get_class(name).is_available()
    except Exception:
        return False


def make(name: str) -> Simulator:
    return get_class(name)()


def availability(names: list[str]) -> dict[str, bool]:
    return {n: is_available(n) for n in names}
