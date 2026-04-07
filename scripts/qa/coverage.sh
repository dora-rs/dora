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

cargo llvm-cov --workspace \
  --exclude adora-node-api-python \
  --exclude adora-operator-api-python \
  --exclude adora-ros2-bridge-python \
  --exclude adora-cli-api-python \
  --exclude adora-examples \
  --lcov --output-path lcov.info \
  --summary-only

echo
echo "Coverage report written to lcov.info"
