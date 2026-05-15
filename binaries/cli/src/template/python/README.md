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
uv pip install -e talker-1 -e talker-2 -e listener-1
uv run pytest . # Test
```

When testing this template from a Dora source checkout, install the checkout's
Python API into the same root environment before running pytest, for example:

```bash
uv pip install -e ../apis/python/node -e talker-1 -e talker-2 -e listener-1
```

## YAML Specification

## Examples

## License

Node Name's code are released under the MIT License
