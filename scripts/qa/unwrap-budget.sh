#!/usr/bin/env bash
# scripts/qa/unwrap-budget.sh — ratchet check on .unwrap()/.expect() count
#
# Counts unwrap/expect in non-test, non-build, non-example Rust source.
# Fails if count exceeds the baseline in .unwrap-budget.
# Reductions are reported but not auto-applied; commit a smaller budget
# in the same PR if you reduce.

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
