"""Registry: lazy mapping + graceful handling of unavailable backends."""

from __future__ import annotations

import pytest

from sim2sim.sim import registry


def test_all_backends_registered():
    names = registry.registered_names()
    assert set(names) == {"mujoco", "pybullet", "genesis", "isaaclab"}


def test_unknown_backend_raises():
    with pytest.raises(KeyError):
        registry.get_class("does_not_exist")


def test_availability_is_bool_map():
    avail = registry.availability(["mujoco", "pybullet", "genesis", "isaaclab"])
    assert set(avail) == {"mujoco", "pybullet", "genesis", "isaaclab"}
    assert all(isinstance(v, bool) for v in avail.values())


def test_gpu_backends_unavailable_without_cuda():
    # On a CPU host these must report unavailable (and never crash on import).
    assert registry.is_available("genesis") is False
    assert registry.is_available("isaaclab") is False


def test_get_class_imports_lazily():
    # Importing the registry must not import a backend; fetching the class does.
    cls = registry.get_class("mujoco")
    assert cls.name == "mujoco"
