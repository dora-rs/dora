#!/bin/bash
# Stage C node library files from TARGET_DIR to PREFIX directory
# Usage: stage-c-node.sh <TARGET_DIR> <PREFIX>
# TARGET_DIR: cargo build output directory (e.g., target/release or target/x86_64-unknown-linux-gnu/release)
# PREFIX:     staging output directory (e.g., dora-c-libraries-x86_64-unknown-linux-gnu)

set -e

TARGET_DIR="${1:-target/release}"
PREFIX="${2:-dora-c-libraries}"

echo "Staging C node library files..."
echo "  TARGET_DIR: $TARGET_DIR"
echo "  PREFIX:     $PREFIX"

# Create PREFIX directory structure
mkdir -p "$PREFIX/lib/cmake"
mkdir -p "$PREFIX/include"

# Copy cmake config files from TARGET_DIR
cp -r "$TARGET_DIR/lib/cmake/dora-node-api-c" "$PREFIX/lib/cmake/"

# Copy header from TARGET_DIR
cp "$TARGET_DIR/include/node_api.h" "$PREFIX/include/"

# Copy library from TARGET_DIR
LIB_NAME="libdora_node_api_c.a"
if [ -f "$TARGET_DIR/$LIB_NAME" ]; then
    cp "$TARGET_DIR/$LIB_NAME" "$PREFIX/lib/"
else
    echo "WARNING: Library file not found at $TARGET_DIR/$LIB_NAME"
fi

echo "Staged files:"
ls -la "$PREFIX/lib/"
ls -la "$PREFIX/include/"
ls -la "$PREFIX/lib/cmake/dora-node-api-c/"