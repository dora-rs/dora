#!/bin/bash
# Stage C node library files from OUT_DIR to PREFIX directory
# Usage: stage-c-node.sh <BUILD_DIR> <PREFIX>
# BUILD_DIR: cargo build output directory (e.g., target/release or target/x86_64-unknown-linux-gnu/release)
# PREFIX: staging output directory (e.g., dora-c-libraries-x86_64-unknown-linux-gnu)

set -e

BUILD_DIR="${1:-target/release}"
PREFIX="${2:-dora-c-libraries}"

echo "Staging C node library files..."
echo "  BUILD_DIR: $BUILD_DIR"
echo "  PREFIX:    $PREFIX"

# Find OUT_DIR for dora-node-api-c
OUT_DIR=$(find "$BUILD_DIR/build" -type d -name 'out' 2>/dev/null | grep dora-node-api-c | head -1)

if [ -z "$OUT_DIR" ]; then
    echo "ERROR: Could not find OUT_DIR for dora-node-api-c in $BUILD_DIR/build"
    exit 1
fi

echo "  OUT_DIR:   $OUT_DIR"

# Create PREFIX directory structure
mkdir -p "$PREFIX/lib/cmake"
mkdir -p "$PREFIX/include"

# Copy cmake config files from OUT_DIR
cp -r "$OUT_DIR/lib/cmake/dora-node-api-c" "$PREFIX/lib/cmake/"

# Copy header from OUT_DIR
cp "$OUT_DIR/include/node_api.h" "$PREFIX/include/"

# Copy library from BUILD_DIR
if [ -f "$BUILD_DIR/libdora_node_api_c.a" ]; then
    cp "$BUILD_DIR/libdora_node_api_c.a" "$PREFIX/lib/"
else
    echo "WARNING: Library file not found at $BUILD_DIR/libdora_node_api_c.a"
fi

echo "Staged files:"
ls -la "$PREFIX/lib/"
ls -la "$PREFIX/include/"
ls -la "$PREFIX/lib/cmake/dora-node-api-c/"