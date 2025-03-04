# Quick example on using a VLM with dora-rs

Make sure to have, dora, uv and cargo installed.

```bash
cd examples/vlm
uv venv -p 3.11 --seed
uv pip install -e ../../apis/python/node
dora build qwen2-5-vl-vision-only-dev.yml --uv
dora run qwen2-5-vl-vision-only-dev.yml --uv
```

- Without cloning the repository:

```bash
uv venv -p 3.11 --seed
dora build https://raw.githubusercontent.com/dora-rs/dora/main/examples/vlm/qwenvl.yml --uv
dora run https://raw.githubusercontent.com/dora-rs/dora/main/examples/vlm/qwenvl.yml --uv
```
