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

For the remote instance, with an example instance at ip: 3.82.54.170

```bash
cd examples/translation
uv venv --seed -p 3.11
uv pip install -e ../../node-hub/dora-microphone
uv pip install -e ../../node-hub/dora-rerun

dora daemon --coordinator-addr 3.82.54.170 --machine-id macbook &
dora start phi4-remote.yml --coordinator-addr 3.82.54.170 --uv

# When you see `all nodes are ready`
# Start talking and you should see text appearing
```

To setup the remote instance:

```bash
git clone https://github.com/dora-rs/dora.git
cd examples/translation

uv venv --seed -p 3.11
dora build phi4-remote.yml --uv

dora coordinator & # In a separate window is better
dora daemon --machine-id gpu
```
