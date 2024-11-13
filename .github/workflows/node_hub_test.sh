#!/bin/bash
set -euo

# List of ignored modules 
ignored_folders=("dora-internvl" "dora-parler" "dora-keyboard" "dora-microphone" "terminal-input")

# Get current working directory
dir=$(pwd)

# Get the base name of the directory (without the path)
base_dir=$(basename "$dir")

# Check if the directory name is in the ignored list
if [[ " ${ignored_folders[@]} " =~ " ${base_dir} " ]]; then
    echo "Skipping $base_dir as we cannot test it on the CI..."
else
    if [ -f "$dir/Cargo.toml" ]; then
        echo "Running build and tests for Rust project in $dir..."
        cargo build
        cargo test
    else
        if [ -f "$dir/pyproject.toml" ]; then
        echo "Running linting and tests for Python project in $dir..."
        pip install .
        poetry run black --check .
        poetry run pylint --disable=C,R  --ignored-modules=cv2 **/*.py
        poetry run pytest
        fi
    fi   
fi