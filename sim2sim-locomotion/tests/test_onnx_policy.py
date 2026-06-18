"""ONNX policy load + inference + obs-dim contract."""

from __future__ import annotations

import numpy as np
import pytest
from conftest import make_linear_onnx

from sim2sim.policy.onnx_policy import OnnxPolicy


def test_load_and_infer(tmp_path):
    path = str(tmp_path / "policy.onnx")
    make_linear_onnx(path, obs_dim=48, action_dim=12)
    pol = OnnxPolicy(path, expected_obs_dim=48)
    action = pol.act(np.zeros(48, dtype=np.float32))
    assert action.shape == (12,)
    assert action.dtype == np.float32


def test_obs_dim_mismatch_raises(tmp_path):
    path = str(tmp_path / "policy.onnx")
    make_linear_onnx(path, obs_dim=48, action_dim=12)
    with pytest.raises(ValueError, match="obs dim"):
        OnnxPolicy(path, expected_obs_dim=36)


def test_clip_actions(tmp_path):
    path = str(tmp_path / "policy.onnx")
    # Large weights so outputs exceed the clip bound.
    make_linear_onnx(path, obs_dim=8, action_dim=4, seed=1)
    pol = OnnxPolicy(path, clip_actions=0.001)
    action = pol.act(np.full(8, 10.0, dtype=np.float32))
    assert np.all(np.abs(action) <= 0.001 + 1e-9)


def test_satisfies_policy_protocol(tmp_path):
    from sim2sim.policy.base import Policy

    path = str(tmp_path / "policy.onnx")
    make_linear_onnx(path, obs_dim=8, action_dim=4)
    pol = OnnxPolicy(path)
    assert isinstance(pol, Policy)
