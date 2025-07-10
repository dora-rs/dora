# dora-isaacsim

## Getting started

- Install it with uv:

```bash
uv venv -p 3.11 --seed
uv pip install -e .
```

## Contribution Guide

- Format with [ruff](https://docs.astral.sh/ruff/):

```bash
uv pip install ruff
uv run ruff check . --fix
```

- Lint with ruff:

```bash
uv run ruff check .
```

- Test with [pytest](https://github.com/pytest-dev/pytest)

```bash
uv pip install pytest
uv run pytest . # Test
```

## YAML Specification

```yaml
- id: dora-isaacsim
    build: pip install -e ../../node-hub/dora-isaacsim
    path: ../../node-hub/dora-isaacsim/dora_isaacsim/main.py
    env:
        # substitute to your own "<path of isaacsim>/python.sh"
        ISAAC_PYTHON_PATH: "/home/lv/isaacsim/python.sh"
        CONFIG_NAME: "stack_cube_act"
    inputs:
        request_camera: policy-act/request_camera
        request_joint_pos: policy-act/request_joint_pos
        action: policy-act/action
    outputs:
        - camera
        - joint_pos
```

## Examples

## License

dora-isaacsim's code are released under the MIT License
