# Minimal Dora Docker Environment

This Dockerfile provides a minimal environment for running Dora applications with Python and uv package manager.

## What's Included

- Python 3.12
- Rust (required for Dora)
- uv package manager
- Latest Dora release

## Building the Image

```bash
docker build . -t dora-minimal
```

## Running the Container

```bash
docker run -it --rm dora-minimal
```

## Running not in interactive

```bash
docker run --rm dora-minimal dora --help
```

## Running with privilege as well as USB connection

```bash
docker run --rm  --device=/dev/ttyUSB0 dora-minimal dora --help
```

## Usage

Once inside the container, you can:

- Run Dora commands: `dora --help`
- Use uv for package management: `uv install numpy`
- Develop and test your Dora applications

This container is designed to provide a consistent environment for Dora development without requiring complex setup on the host machine.
