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
  --exclude dora-node-api-python
  --exclude dora-operator-api-python
  --exclude dora-ros2-bridge-python
  --exclude dora-cli-api-python
  --exclude dora-examples
  # C++ bindings: their build.rs fails under llvm-cov instrumentation
  # (safer_ffi header generation). Bindings are not logic, so low
  # coverage value anyway.
  --exclude dora-node-api-cxx
  --exclude dora-operator-api-cxx
)

# Single invocation: runs tests, writes lcov.info, prints summary table.
# --summary-only limits text output but is mutually exclusive with --lcov,
# so we call cargo-llvm-cov twice; the second call reuses cached test data.
cargo llvm-cov --workspace "${EXCLUDES[@]}" --summary-only
cargo llvm-cov --workspace "${EXCLUDES[@]}" --lcov --output-path lcov.info

echo
echo "Coverage written to lcov.info"
echo "For an HTML report: cargo llvm-cov --workspace --html --open"

# --- Optional: diff coverage gate ---
#
# If diff-cover is installed and we're running on a branch with a
# non-empty diff vs origin/main, also print a diff coverage report
# and fail if diff coverage on touched lines is below the threshold.
#
# Non-blocking by default (DIFF_COVERAGE_FAIL_UNDER=0 disables the gate).
# Pass DIFF_COVERAGE_FAIL_UNDER=80 (or similar) to enforce.
DIFF_COVERAGE_FAIL_UNDER="${DIFF_COVERAGE_FAIL_UNDER:-0}"
DIFF_COVERAGE_BASE="${DIFF_COVERAGE_BASE:-origin/main}"

if command -v diff-cover >/dev/null 2>&1; then
  # Fetch base if it's a remote ref
  if [[ "$DIFF_COVERAGE_BASE" == origin/* ]]; then
    git fetch --quiet origin "${DIFF_COVERAGE_BASE#origin/}" 2>/dev/null || true
  fi
  # Only run if there's actually a diff (avoid noise on main)
  if ! git diff --quiet "$DIFF_COVERAGE_BASE"...HEAD 2>/dev/null; then
    echo
    echo "--- diff coverage vs $DIFF_COVERAGE_BASE ---"
    if [[ "$DIFF_COVERAGE_FAIL_UNDER" -gt 0 ]]; then
      diff-cover lcov.info \
        --compare-branch="$DIFF_COVERAGE_BASE" \
        --fail-under="$DIFF_COVERAGE_FAIL_UNDER"
    else
      # Report only, don't fail
      diff-cover lcov.info --compare-branch="$DIFF_COVERAGE_BASE" || true
    fi
  fi
else
  echo
  echo "(diff-cover not installed; skipping diff coverage gate)"
  echo "Install: pip install diff-cover"
fi
