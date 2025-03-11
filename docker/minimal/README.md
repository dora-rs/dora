# Minimal Dora Docker Environment

This Dockerfile provides a minimal environment for running Dora applications with Python and uv package manager.

## What's Included

- Python 3.12
- Rust (required for Dora)
- uv package manager
- Latest Dora release

## Building the Image

docker build -t dora-minimal 

## Running the Container

docker run -it --rm -v $(pwd):/app dora-minimal

## Usage

Once inside the container, you can:

- Run Dora commands: `dora --help`
- Use uv for package management: `uv install numpy`
- Develop and test your Dora applications

This container is designed to provide a consistent environment for Dora development without requiring complex setup on the host machine.
