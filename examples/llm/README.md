# Quick example on using a LLM with dora-rs

Make sure to have, dora, pip and cargo installed.

## System dependencies:

- MacOS

```bash
brew install portaudio
brew install espeak-ng
```

- Linux

```bash
sudo apt-get install portaudio19-dev
sudo apt-get install espeak
```

## Installation

```bash
# Make sure to have the latest the development dora version
cargo build -p dora-cli --release
## Alternatively
# uv pip install -e ../../binaries/cli --reinstall

uv venv --seed -p 3.11
# Make sure to have the latest the development dora node api version
uv pip install -e ../../apis/python/node --reinstall
dora build qwen-dev.yml --uv
dora run qwen-dev.yml --uv
```

## With prepackaged version

```bash
pip install dora-rs-cli==3.10
uv venv --seed -p 3.11
# Make sure to have the latest the development dora node api version
dora build qwen-dev.yml --uv
dora run qwen-dev.yml --uv
```

## To run

```bash
dora run qwen-dev.yml --uv
```
