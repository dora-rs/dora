# slim Dora Docker Environment

This Dockerfile provides a slim environment for running Dora applications with Python and uv package manager.

## What's Included

- Python 3.12
- Rust (required for Dora)
- uv package manager
- Latest Dora release

## Building the Image

```bash
docker build . -t dora-slim
```

## Running the Container

```bash
docker run -it --rm --device=/dev/video0 dora-slim
```

## Running not in interactive

```bash
docker run --rm dora-slim dora --help
```

## Running with privilege as well as USB connection

```bash
docker run --rm  --device=/dev/video0 dora-slim dora --help
```

## Usage

Once inside the container, you can:

```bash
## Create a virtual environment
uv venv --seed -p 3.11

## Install nodes dependencies of a remote graph
dora build https://raw.githubusercontent.com/dora-rs/dora/refs/heads/main/examples/object-detection/yolo.yml --uv

## Run yolo graph
dora run yolo.yml --uv
```

This container is designed to provide a consistent environment for Dora development without requiring complex setup on the host machine.
