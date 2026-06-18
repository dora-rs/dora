"""ONNX policy — the reference, sim-agnostic policy implementation.

The same ``.onnx`` file is fed to every simulator. We read the graph's input and
output names directly (rather than hard-coding them) so policies exported from
different training stacks (Isaac Lab, legged_gym, rsl_rl, ...) load without
fuss. Inference runs on CPU by default.
"""

from __future__ import annotations

import numpy as np


class OnnxPolicy:
    def __init__(
        self,
        onnx_path: str,
        expected_obs_dim: int | None = None,
        clip_actions: float | None = None,
        providers: list[str] | None = None,
    ):
        import onnxruntime as ort  # lazy: keep onnxruntime out of import-time cost

        self.session = ort.InferenceSession(
            onnx_path, providers=providers or ["CPUExecutionProvider"]
        )
        inputs = self.session.get_inputs()
        if len(inputs) != 1:
            raise ValueError(
                f"expected a single-input policy graph, got {len(inputs)} inputs: "
                f"{[i.name for i in inputs]}"
            )
        self._input_name = inputs[0].name
        self._output_name = self.session.get_outputs()[0].name
        self._obs_dim = _static_last_dim(inputs[0].shape)
        self.clip_actions = clip_actions

        if expected_obs_dim is not None and self._obs_dim is not None:
            if self._obs_dim != expected_obs_dim:
                raise ValueError(
                    f"policy expects obs dim {self._obs_dim} but the configured "
                    f"observation builds dim {expected_obs_dim}; check obs_terms ordering"
                )

    def reset(self) -> None:
        pass

    def act(self, obs: np.ndarray) -> np.ndarray:
        x = np.asarray(obs, dtype=np.float32).reshape(1, -1)
        out = self.session.run([self._output_name], {self._input_name: x})[0]
        action = np.asarray(out, dtype=np.float32).ravel()
        if self.clip_actions is not None:
            action = np.clip(action, -self.clip_actions, self.clip_actions)
        return action


def _static_last_dim(shape: list) -> int | None:
    """Return the trailing dimension if it is a concrete int, else None.

    ONNX shapes may carry symbolic batch dims (strings / None); only the feature
    dimension is useful for validation.
    """
    if not shape:
        return None
    last = shape[-1]
    return int(last) if isinstance(last, int) else None
