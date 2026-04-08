#!/usr/bin/env bash
# scripts/qa/coverage.sh — code coverage via cargo-llvm-cov
#
# Runs the workspace under llvm-cov, emits an lcov.info report,
# prints a summary table. CI uploads lcov.info to Codecov.
#
# Install: cargo install cargo-llvm-cov && rustup component add llvm-tools-preview
#
# To gate diff coverage on PRs, install diff-cover:
#   pip install diff-cover
#   diff-cover lcov.info --compare-branch=origin/main --fail-under=80

set -euo pipefail

cd "$(dirname "$0")/../.."

if ! command -v cargo-llvm-cov >/dev/null; then
  echo "cargo-llvm-cov not installed. Run:"
  echo "  cargo install cargo-llvm-cov"
  echo "  rustup component add llvm-tools-preview"
  exit 2
fi

EXCLUDES=(
  --exclude adora-node-api-python
  --exclude adora-operator-api-python
  --exclude adora-ros2-bridge-python
  --exclude adora-cli-api-python
  --exclude adora-examples
  # C++ bindings: their build.rs fails under llvm-cov instrumentation
  # (safer_ffi header generation). Bindings are not logic, so low
  # coverage value anyway.
  --exclude adora-node-api-cxx
  --exclude adora-operator-api-cxx
)

# Single invocation: runs tests, writes lcov.info, prints summary table.
# --summary-only limits text output but is mutually exclusive with --lcov,
# so we call cargo-llvm-cov twice; the second call reuses cached test data.
cargo llvm-cov --workspace "${EXCLUDES[@]}" --summary-only
cargo llvm-cov --workspace "${EXCLUDES[@]}" --lcov --output-path lcov.info

echo
echo "Coverage written to lcov.info"
echo "For an HTML report: cargo llvm-cov --workspace --html --open"
