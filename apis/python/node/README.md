This crate corresponds to the Node API for Dora.

## Building

To build the Python module for development:

```bash
python -m venv .env
source .env/bin/activate
pip install maturin
maturin develop
```

## Type hinting

Type hinting requires to run a second step

```bash
python generate_stubs.py dora dora/__init__.pyi
maturin develop
```
