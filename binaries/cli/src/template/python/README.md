## Getting started

- Install it with:

```bash
uv venv -p 3.11 --seed
dora build dataflow.yml --uv
```

- Run it with:

```bash
dora run dataflow.yml --uv
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

## Examples

## License

Node Name's code are released under the MIT License
