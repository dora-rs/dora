#!/bin/bash
set -euo

# List of ignored modules 
# TODO: Fix HF github action issue.
ignored_folders=("dora-distil-whisper" "dora-internvl" "dora-parler" "dora-qwenvl" "dora-keyboard" "dora-microphone" "terminal-input")

for dir in node-hub/*/ ; do
    # Get the base name of the directory (without the path)
    base_dir=$(basename "$dir")

    # Check if the directory name is in the ignored list
    if [[ " ${ignored_folders[@]} " =~ " ${base_dir} " ]]; then
        echo "Skipping $base_dir as there is a hf model fetching issue..."
        continue
    fi

    if [ -d "$dir" ]; then
        if [ -f "$dir/pyproject.toml" ]; then
        echo "Running linting and tests for Python project in $dir..."
        (cd "$dir" && pip install .)
        (cd "$dir" && poetry run black --check .)
        (cd "$dir" && poetry run pylint --disable=C,R  --ignored-modules=cv2 **/*.py)
        (cd "$dir" && poetry run pytest)
        fi
                
        if [ -f "$dir/Cargo.toml" ]; then
        echo "Running build and tests for Rust project in $dir..."
        (cd "$dir" && cargo build)
        (cd "$dir" && cargo test)
        fi
    fi
done
