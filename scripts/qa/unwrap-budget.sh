#!/usr/bin/env bash
# scripts/qa/unwrap-budget.sh — ratchet check on .unwrap()/.expect() count
#
# Counts unwrap/expect in non-test, non-build, non-example Rust source.
# Fails if count exceeds the baseline in .unwrap-budget.
# Reductions are reported but not auto-applied; commit a smaller budget
# in the same PR if you reduce.
#
# KNOWN LIMITATION: this script excludes `tests/` directories and
# `examples/`, but it does NOT exclude `#[cfg(test)] mod tests {}` blocks
# inline in source files. Adding test-only unwraps to a source file will
# inflate the count and require a budget bump even though no production
# code changed. Fix queued: parse with syn/ast-grep to count only
# non-test scopes. See follow-up tracking.

set -euo pipefail

cd "$(dirname "$0")/../.."

BUDGET_FILE=".unwrap-budget"

if [[ ! -f "$BUDGET_FILE" ]]; then
  echo "$BUDGET_FILE not found. Establishing initial baseline."
  CURRENT=$(rg --type rust \
    -g '!**/tests/**' \
    -g '!**/build.rs' \
    -g '!**/examples/**' \
    '\.unwrap\(\)|\.expect\(' \
    libraries/ binaries/ apis/ 2>/dev/null | wc -l | tr -d ' ')
  echo "$CURRENT" > "$BUDGET_FILE"
  echo "Initial baseline: $CURRENT"
  exit 0
fi

CURRENT=$(rg --type rust \
  -g '!**/tests/**' \
  -g '!**/build.rs' \
  -g '!**/examples/**' \
  '\.unwrap\(\)|\.expect\(' \
  libraries/ binaries/ apis/ 2>/dev/null | wc -l | tr -d ' ')

BUDGET=$(cat "$BUDGET_FILE" | tr -d ' \n')

echo "unwrap/expect count: $CURRENT (budget: $BUDGET)"

if [[ "$CURRENT" -gt "$BUDGET" ]]; then
  echo
  echo "Unwrap budget exceeded: $CURRENT > $BUDGET"
  echo
  echo "New unwraps in this change. Either:"
  echo "  1. Replace with proper error handling, or"
  echo "  2. Update $BUDGET_FILE with justification in the commit message."
  exit 1
fi

if [[ "$CURRENT" -lt "$BUDGET" ]]; then
  echo
  echo "Note: count reduced. Commit a smaller budget:"
  echo "  echo $CURRENT > $BUDGET_FILE"
  # Non-blocking
fi
