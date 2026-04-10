#!/usr/bin/env bash
# Clean generated output and build directories from examples/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

count=0
for dir in $(find "$ROOT_DIR/examples" -type d \( -name "out" -o -name "build" -o -name "__pycache__" \)); do
    echo "rm -rf $dir"
    rm -rf "$dir"
    count=$((count + 1))
done

echo "Cleaned $count directories"
