# Quick example on using a LLM with dora-rs

Make sure to have, dora, pip and cargo installed.

```bash
# Make sure to not be in a virtual environment
uv venv --seed -p 3.11
uv pip install -e ../../apis/python/node
dora build qwen-dev.yml --uv
dora run qwen-dev.yml --uv
```
