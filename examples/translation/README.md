# Dora argo example

Make sure to have, dora, pip and cargo installed.

```bash
git clone https://github.com/dora-rs/dora.git
cd dora/examples/translation

uv venv --seed -p 3.11
dora build phi4-dev.yml --uv
dora run phi4-dev.yml --uv

# Start talking or play a recording in English, Chinese, German, French, Italian, Japanese, Spanish, Portuguese
```
