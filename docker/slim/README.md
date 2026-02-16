# slim Adora Docker Environment

This Dockerfile provides a slim environment for running Adora applications with Python and uv package manager.

## What's Included

- Python 3.12
- Rust (required for Adora)
- uv package manager
- Latest Adora release

## Building the Image

```bash
docker build . -t adora-slim
```

## Running the Container

```bash
docker run -it --rm --device=/dev/video0 adora-slim
```

## Running not in interactive

```bash
docker run --rm adora-slim adora --help
```

## Running with privilege as well as USB connection

```bash
docker run --rm  --device=/dev/video0 adora-slim adora --help
```

## Usage

Once inside the container, you can:

```bash
## Create a virtual environment
uv venv --seed -p 3.11

## Install nodes dependencies of a remote graph
adora build https://raw.githubusercontent.com/adora-rs/adora/refs/heads/main/examples/object-detection/yolo.yml --uv

## Run yolo graph
adora run yolo.yml --uv
```

This container is designed to provide a consistent environment for Adora development without requiring complex setup on the host machine.
