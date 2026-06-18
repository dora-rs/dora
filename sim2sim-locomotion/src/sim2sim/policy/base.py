"""Policy interface.

A policy maps an observation vector to an action vector. The protocol is
intentionally minimal so that an ONNX policy, a scripted baseline, or a future
TorchScript/JAX policy are all drop-in interchangeable in the runner.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class Policy(Protocol):
    def reset(self) -> None:
        """Clear any per-episode internal state (e.g. recurrent hidden state)."""
        ...

    def act(self, obs: np.ndarray) -> np.ndarray:
        """Return an action for the given observation vector."""
        ...
