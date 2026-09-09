# slim Dora Docker Environment

This Dockerfile provides a slim environment for running Dora applications with Python and uv package manager.

## What's Included

- Python 3.12
- Rust (required for Dora)
- uv package manager
- Latest Dora release

## Supported Platforms

`linux/amd64` and `linux/arm64`.

32-bit arm is not supported and cannot be: there is no `arm/v6` variant of the `python:3.12-slim` base image, and on `arm/v7` the install stops at `pyarrow`, which has never published an armv7l wheel and whose sdist needs Arrow C++. (The `dora-rs` wheels themselves *are* built for armv7l — it is the Arrow dependency that is missing, not dora.) `.github/workflows/docker-image.yml` builds exactly the platforms listed here; keep the two in step.

## Pulling the Image

```bash
docker pull ghcr.io/dora-rs/dora-slim:latest
```

Published on every merge to `main` that touches `docker/`, once the smoke below passes.

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

## Verifying the image

`smoke.sh` is the deployment smoke: it runs *inside* a built image and asserts that the `dora` it ships can actually host a dataflow. It reports the `dora-rs-cli` / `dora-rs` pair the image resolved at build time, then runs a two-node dataflow and checks that Arrow payloads arrive intact at the sink.

From the repository root:

```bash
make qa-docker-slim
```

Or by hand, from this directory:

```bash
docker build . -t dora-slim
docker run --rm -v "$PWD/smoke.sh:/smoke.sh:ro" dora-slim bash /smoke.sh
```

CI runs the same script against every build (`.github/workflows/docker-image.yml`), and the image is only published once it passes. The Dockerfile installs `dora-rs-cli` from PyPI rather than from the workspace, so the smoke covers the *published* CLI plus the image -- see `docs/qa-runbook.md` §3.14 for what a failure does and does not implicate.
