#!/usr/bin/env bash
# scripts/qa/mutants.sh — mutation testing via cargo-mutants
#
# Runs mutation testing on critical crates. By default scopes to files
# changed vs origin/main (--in-diff). Pass --full for the full repo.
#
# Install: cargo install cargo-mutants

set -euo pipefail

cd "$(dirname "$0")/../.."

if ! command -v cargo-mutants >/dev/null; then
  echo "cargo-mutants not installed. Run: cargo install cargo-mutants"
  exit 2
fi

CRITICAL_CRATES=(
  --package dora-core
  --package dora-daemon
  --package dora-coordinator
  --package dora-message
  --package dora-coordinator-store
  --package shared-memory-server
)

if [[ "${1:-}" == "--full" ]]; then
  echo "Running full-repo mutation test (this takes 1-4 hours)..."
  cargo mutants \
    "${CRITICAL_CRATES[@]}" \
    --jobs 4 \
    --timeout 120
else
  echo "Running diff mutation test vs origin/main..."
  DIFF_FILE="$(mktemp)"
  trap 'rm -f "$DIFF_FILE"' EXIT
  git diff -U0 origin/main...HEAD >"$DIFF_FILE" 2>/dev/null || git diff -U0 origin/main >"$DIFF_FILE"
  cargo mutants \
    --in-diff "$DIFF_FILE" \
    "${CRITICAL_CRATES[@]}" \
    --jobs 4 \
    --timeout 120
fi
