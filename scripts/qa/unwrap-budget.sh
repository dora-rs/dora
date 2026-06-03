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
#   3. Each `#[cfg(test)]`-attributed item (the brace-balanced `mod`/`fn`/
#      `impl` block that follows the attribute) — inline test modules and
#      test helpers anywhere in a source file, not just at EOF.
#
# Rule 3 brace-balances each `#[cfg(test)]` block instead of truncating to
# EOF, so production code that follows an interspersed test block (e.g. a
# `#[cfg(test)]` helper fn before more production fns, as in
# binaries/cli/src/output.rs) is still counted. All `#[cfg(test)]` items in
# the counted source are brace blocks (mod/fn/impl); brace-less attributed
# items (`#[cfg(test)] use ...;`) do not occur and are not handled.

set -euo pipefail

cd "$(dirname "$0")/../.."

BUDGET_FILE=".unwrap-budget"

if ! command -v rg >/dev/null 2>&1; then
  echo "scripts/qa/unwrap-budget.sh requires ripgrep ('rg') on PATH" >&2
  echo "Install ripgrep and re-run the QA checks." >&2
  exit 2
fi

# Count unwrap/expect in non-test source code. Three steps:
# 1. Enumerate candidate .rs files (excluding tests/, examples/, build.rs)
# 2. Drop files named tests.rs (submodule test files)
# 3. For each remaining file, skip each #[cfg(test)] brace-balanced block
#    (mod/fn/impl) and count .unwrap() / .expect( in the rest.
count_unwraps() {
  local total=0
  local file
  while IFS= read -r file; do
    # Rule 2: drop test submodule files
    case "$file" in
      */tests.rs) continue ;;
    esac
    # Rule 3: skip each #[cfg(test)] block, count unwrap/expect in the rest.
    # The block ends at a `}` DEDENTED to the attribute's indentation. We use
    # dedent (not raw brace counting) because counting `{`/`}` would miscount
    # braces inside string literals and `format!("{}")` macros that pepper test
    # code; the block-closing brace is the only `}` back at the attribute indent.
    local n
    n=$(awk '
      # Inside a test block: ends when a `}` dedents to the attribute indent.
      in_test {
        if ($0 ~ ("^" ind "[}]")) in_test = 0
        next
      }
      # Saw #[cfg(test)]; skip the items signature lines until its opening `{`
      # (block, e.g. mod/fn/impl) or a `;` terminator (one-line decl).
      awaiting {
        if ($0 ~ /[{]/) { awaiting = 0; in_test = 1 }
        else if ($0 ~ /;[[:space:]]*$/) awaiting = 0
        next
      }
      /^[[:space:]]*#\[cfg\(test\)\]/ {
        ind = $0; sub(/[^ \t].*/, "", ind)   # leading whitespace = indent
        if ($0 ~ /[{]/) in_test = 1; else awaiting = 1
        next
      }
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
