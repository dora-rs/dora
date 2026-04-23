#!/usr/bin/env bash
# scripts/qa/mutants.sh — mutation testing via cargo-mutants
#
# Runs mutation testing on critical crates. By default scopes to files
# changed vs origin/main (--in-diff). Pass --full for the full repo.
#
# Install: cargo install cargo-mutants
#
# Timeout: 45s per mutant. 9x longer than a normal test, but short
# enough that hang-inducing mutations (e.g. broken channels) fail
# fast instead of burning 2 minutes each. Was 120s; lowered after an
# observed --full run saw 52 120-second timeouts in the first 1008
# mutants (= ~1h 44min pure timeout waste). If a real passing test
# legitimately takes >45s, bump this back up.

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

TIMEOUT=45

if [[ "${1:-}" == "--full" ]]; then
  echo "Running full-repo mutation test."
  echo "On the dora workspace this has been measured at 10-18 hours."
  echo "Use only when deliberately auditing test quality; not every nightly."
  cargo mutants \
    "${CRITICAL_CRATES[@]}" \
    --jobs 4 \
    --timeout "$TIMEOUT"
else
  echo "Running diff mutation test vs origin/main..."
  DIFF_FILE="$(mktemp)"
  trap 'rm -f "$DIFF_FILE"' EXIT
  git diff -U0 origin/main...HEAD >"$DIFF_FILE" 2>/dev/null || git diff -U0 origin/main >"$DIFF_FILE"
  cargo mutants \
    --in-diff "$DIFF_FILE" \
    "${CRITICAL_CRATES[@]}" \
    --jobs 4 \
    --timeout "$TIMEOUT"
fi
