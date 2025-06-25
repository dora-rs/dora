# policy-act

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
- id: policy-act
    build: pip install -e ../../node-hub/policy-act
    path: ../../node-hub/policy-act/policy_act/main.py
    args: 
        --task_name stack_cube
        --ckpt_dir ../../node-hub/policy-act/policy_act/assets/ckpt/
        --policy_class ACT
        --kl_weight 10
        --chunk_size 20
        --hidden_dim 512
        --batch_size 8
        --dim_feedforward 3200
        --num_epochs 2000
        --lr 1e-5
        --seed 0
        --temporal_agg
        # --eval
    inputs: 
        camera: dora-isaacsim/camera
        joint_pos: dora-isaacsim/joint_pos
    outputs:
        - request_camera
        - request_joint_pos
        - action
    env:
        SCENARIO: sim
```

## Examples

## License

policy-act's code are released under the MIT License
