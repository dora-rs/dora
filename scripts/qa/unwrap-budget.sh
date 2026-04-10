#!/usr/bin/env bash
# scripts/qa/unwrap-budget.sh — ratchet check on .unwrap()/.expect() count
#
# Counts unwrap/expect in non-test, non-build, non-example Rust source.
# Fails if count exceeds the baseline in .unwrap-budget.
# Reductions are reported but not auto-applied; commit a smaller budget
# in the same PR if you reduce.
#
# Exclusion rules (applied in order):
#   1. Paths: `tests/`, `benches/`, `examples/` directories; `build.rs`
#   2. Files named `tests.rs` (submodule test files declared via
#      `#[cfg(test)] mod tests;` from a sibling source file — the file
#      content does not carry the cfg(test) attribute but the whole
#      file is test-only)
#   3. Content after the first `#[cfg(test)]` line in any remaining
#      source file (inline `mod tests {}` blocks at the bottom of a
#      source file — dora's convention)
#
# Rule 3 uses simple line-based truncation, which works because dora
# consistently places `#[cfg(test)]` only at the top of a test module
# and only at the end of source files. Verified 2026-04-08 by grep —
# if this convention changes, this script must be updated.

set -euo pipefail

cd "$(dirname "$0")/../.."

BUDGET_FILE=".unwrap-budget"

# Count unwrap/expect in non-test source code. Three steps:
# 1. Enumerate candidate .rs files (excluding tests/, examples/, build.rs)
# 2. Drop files named tests.rs (submodule test files)
# 3. For each remaining file, strip content from the first #[cfg(test)]
#    line to EOF, then count .unwrap() / .expect( occurrences.
count_unwraps() {
  local total=0
  local file
  while IFS= read -r file; do
    # Rule 2: drop test submodule files
    case "$file" in
      */tests.rs) continue ;;
    esac
    # Rule 3: truncate at first #[cfg(test)] line, count unwraps in head
    local n
    n=$(awk '
      /^[[:space:]]*#\[cfg\(test\)\]/ { exit }
      {
        n = gsub(/\.unwrap\(\)|\.expect\(/, "&")
        if (n > 0) count += n
      }
      END { print count + 0 }
    ' "$file")
    total=$((total + n))
  done < <(
    rg --files --type rust \
      -g '!**/tests/**' \
      -g '!**/benches/**' \
      -g '!**/examples/**' \
      -g '!**/build.rs' \
      libraries/ binaries/ apis/ 2>/dev/null
  )
  echo "$total"
}

if [[ ! -f "$BUDGET_FILE" ]]; then
  echo "$BUDGET_FILE not found. Establishing initial baseline."
  CURRENT=$(count_unwraps)
  echo "$CURRENT" > "$BUDGET_FILE"
  echo "Initial baseline: $CURRENT"
  exit 0
fi

CURRENT=$(count_unwraps)

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
