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
  --package adora-core
  --package adora-daemon
  --package adora-coordinator
  --package adora-message
  --package adora-coordinator-store
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
  cargo mutants \
    --in-diff origin/main \
    "${CRITICAL_CRATES[@]}" \
    --jobs 4 \
    --timeout 120
fi
