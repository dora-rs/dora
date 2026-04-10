#!/usr/bin/env bash
# Run Rust and Python benchmarks side-by-side and compare results.
#
# Prerequisites:
#   - dora CLI installed
#   - Python dora package: pip install dora-rs
#   - numpy and pyarrow: pip install numpy pyarrow
#
# Usage:
#   cd examples/benchmark
#   ./compare.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

RESULTS_DIR="results"
mkdir -p "$RESULTS_DIR"

echo "=== Building Rust benchmark nodes ==="
cargo build --release -p benchmark-example-node -p benchmark-example-sink

echo ""
echo "=== Running Rust benchmark ==="
BENCH_CSV="$RESULTS_DIR/rust.csv" dora run dataflow.yml | tee "$RESULTS_DIR/rust.txt"

echo ""
echo "=== Running Python benchmark ==="
BENCH_CSV="$RESULTS_DIR/python.csv" dora run python-dataflow.yml --uv | tee "$RESULTS_DIR/python.txt"

echo ""
echo "=== Results ==="
echo "Rust results:   $RESULTS_DIR/rust.txt"
echo "Python results: $RESULTS_DIR/python.txt"
echo "CSV files:      $RESULTS_DIR/rust.csv, $RESULTS_DIR/python.csv"
echo ""
echo "Compare latency side-by-side:"
echo "  paste $RESULTS_DIR/rust.txt $RESULTS_DIR/python.txt | head -20"
