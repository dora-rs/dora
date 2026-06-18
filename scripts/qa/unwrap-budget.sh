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
    # A MULTI-LINE block opener ends with `{` (rustfmt puts the opening brace
    # last); its matching close is the only `}` DEDENTED back to the attribute
    # indent. We key off dedent + the trailing-`{`, not raw brace counting,
    # because counting `{`/`}` miscounts braces inside string literals and
    # `format!("{}")` macros that pepper test code. A SELF-CONTAINED one-line
    # item (`#[cfg(test)] fn helper() {}` or a `mod tests;` decl) is fully on
    # one line and is skipped without entering block mode, so production code
    # after it is still counted.
    local n
    n=$(awk '
      # Track raw-string spans (`r#"..."#`). The dedent that closes a test block
      # is found by a `}` at the attribute indent — but a test fixture string
      # (e.g. an embedded `fn main() { ... }`) can put a `}` at column 0 too,
      # which would wrongly end the block and resume counting the rest of the
      # test module as production. So remember whether THIS line is inside a raw
      # string before doing the block-end check. (Single-hash `r#"` form — the
      # form the fixtures use.)
      {
        line_was_raw = in_raw
        if (in_raw) { if ($0 ~ /"#/) in_raw = 0 }
        else if ($0 ~ /r#"/ && $0 !~ /r#".*"#/) in_raw = 1
      }
      # Inside a multi-line test block: ends at a `}` dedented to its indent —
      # but only on a real code line, not raw-string fixture content.
      in_test {
        if (!line_was_raw && $0 ~ ("^" ind "[}]")) in_test = 0
        next
      }
      # Saw #[cfg(test)] with the item on later line(s): skip signature lines
      # until the opener ends with `{` (enter block), or the item resolves on a
      # single line (one-line `{...}` body, or a `;`-terminated decl).
      awaiting {
        if ($0 ~ /\{[[:space:]]*$/) { awaiting = 0; in_test = 1 }
        else if ($0 ~ /[{};]/) awaiting = 0
        next
      }
      /^[[:space:]]*#\[cfg\(test\)\]/ {
        ind = $0; sub(/[^ \t].*/, "", ind)   # leading whitespace = indent
        if ($0 ~ /\{[[:space:]]*$/) in_test = 1      # multi-line block opener
        else if ($0 !~ /[{]/) awaiting = 1           # brace is on a later line
        # else: self-contained one-line `{...}` item -> skip just this line
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
