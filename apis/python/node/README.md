This crate corresponds to the Node API for Dora.

## Building

To build the Python module for development:

```bash
uv venv --seed -p 3.11
uv pip install -e .
```

## Type hinting

Type hinting requires to run a second step

```bash
python generate_stubs.py dora dora/__init__.pyi
maturin develop
```
