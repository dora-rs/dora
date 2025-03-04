# Dora Speech to Text example

Make sure to have, dora, pip and cargo installed.

```bash
uv venv --seed -p 3.11
uv pip install -e ../../apis/python/node --reinstall
dora build kokoro-dev.yml
dora run kokoro-dev.yml
```
