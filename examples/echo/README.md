# Dora echo example

Make sure to have, dora, uv and cargo installed.

```bash
uv venv -p 3.11 --seed
uv pip install -e ../../apis/python/node --reinstall
dora build dataflow.yml --uv
dora run dataflow.yml --uv
```
