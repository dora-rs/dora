# Quick example on how to use a camera

Make sure to have, dora and pip installed.

```bash
uv venv -p 3.11 --seed
uv pip install -e ../../apis/python/node --reinstall
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```
