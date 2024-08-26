#!/bin/bash
set -euo

for dir in node-hub/*/ ; do
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
