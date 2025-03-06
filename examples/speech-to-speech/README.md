# Dora Speech to Text example

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

## To Run:

```bash
uv venv --seed -p 3.11
uv pip install -e ../../apis/python/node --reinstall
dora build kokoro-dev.yml
dora run kokoro-dev.yml
```
