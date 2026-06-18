"""Shared test fixtures."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOT_CFG = REPO_ROOT / "configs" / "robot" / "quad12.yaml"
POLICY_CFG = REPO_ROOT / "configs" / "policy" / "quad12_flat.yaml"


@pytest.fixture
def robot_cfg():
    from sim2sim.config import RobotCfg

    return RobotCfg.from_yaml(ROBOT_CFG)


@pytest.fixture
def policy_cfg():
    from sim2sim.config import PolicyCfg

    return PolicyCfg.from_yaml(POLICY_CFG)


def make_linear_onnx(path: str, obs_dim: int, action_dim: int, seed: int = 0) -> str:
    """Write a tiny single-input/single-output linear ONNX policy (obs->action).

    Used to test loading + the obs-dim contract without a heavy ML dependency at
    runtime (onnx is a dev-only dependency).
    """
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    rng = np.random.default_rng(seed)
    w = (rng.standard_normal((obs_dim, action_dim)) * 0.01).astype(np.float32)
    b = np.zeros(action_dim, dtype=np.float32)

    inp = helper.make_tensor_value_info("obs", TensorProto.FLOAT, [1, obs_dim])
    out = helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, action_dim])
    node = helper.make_node("Gemm", ["obs", "W", "B"], ["action"])
    graph = helper.make_graph(
        [node],
        "linear_policy",
        [inp],
        [out],
        initializer=[
            numpy_helper.from_array(w, name="W"),
            numpy_helper.from_array(b, name="B"),
        ],
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    model.ir_version = 9
    onnx.checker.check_model(model)
    onnx.save(model, path)
    return path
