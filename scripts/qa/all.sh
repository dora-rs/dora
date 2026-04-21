#!/usr/bin/env bash
# scripts/qa/all.sh — master QA runner
#
# Runs the QA gates from docs/plan-agentic-qa-strategy.md.
# Designed for local-first execution: same script runs locally and in CI.
#
# Modes (increasing thoroughness):
#   --fast          ~1 min     pre-commit sanity (fmt, clippy, audit, unwrap, typos)
#   --full          ~5-10 min  pre-push (fast + tests + coverage + optional adversarial)
#   --deep          ~15 min    full Tier 1 PR gate (full + mutants on diff + semver)
#   --tier1                    back-compat alias for --deep
#   --nightly       ~4 hours   Tier 2 locally (deep + proptest@1000 + miri + full mutants)
#   --release-gate             Tier 3 automatable (deep + semver; audit+dogfood are human gates)
#
# Without flags, runs --fast.

set -euo pipefail

cd "$(dirname "$0")/../.."

MODE="${1:---fast}"
FAILED=()

run() {
  local name="$1"; shift
  echo
  echo "=== $name ==="
  if "$@"; then
    echo "  PASS: $name"
  else
    echo "  FAIL: $name"
    FAILED+=("$name")
  fi
}

# Optional run: skip (without failing) if the required tool is missing.
# Used for Tier 2 items like miri that need nightly Rust.
run_optional() {
  local name="$1" check="$2"; shift 2
  if ! eval "$check" > /dev/null 2>&1; then
    echo
    echo "=== $name (SKIP: prerequisite missing) ==="
    return
  fi
  run "$name" "$@"
}

# Normalize --tier1 to --deep so the rest of the script only branches on one.
if [[ "$MODE" == "--tier1" ]]; then
  echo "(note: --tier1 is a deprecated alias for --deep)"
  MODE="--deep"
fi

# ----- Always run (fast) -----
run "fmt"           cargo fmt --all -- --check
run "clippy"        cargo clippy --all \
                      --exclude dora-node-api-python \
                      --exclude dora-operator-api-python \
                      --exclude dora-ros2-bridge-python \
                      -- -D warnings
run "audit"         scripts/qa/audit.sh
run "unwrap-budget" scripts/qa/unwrap-budget.sh
run "typos"         scripts/qa/typos.sh

case "$MODE" in
  --fast)
    : # done
    ;;
  --full|--deep|--nightly|--release-gate)
    run "test" cargo test --all \
      --exclude dora-node-api-python \
      --exclude dora-operator-api-python \
      --exclude dora-ros2-bridge-python \
      --exclude dora-cli-api-python \
      --exclude dora-examples
    run "coverage" scripts/qa/coverage.sh
    # Adversarial LLM review skips cleanly on machines without codex/claude.
    run "adversarial" scripts/qa/adversarial.sh --optional
    ;;
  *)
    echo "Unknown mode: $MODE"
    echo "Usage: $0 [--fast|--full|--deep|--tier1|--nightly|--release-gate]"
    exit 2
    ;;
esac

case "$MODE" in
  --deep|--release-gate)
    run "mutants (diff-scoped)" scripts/qa/mutants.sh
    run "semver"                scripts/qa/semver.sh
    ;;
  --nightly)
    # Tier 2 skips the diff-scoped mutants run -- `mutants --full` below
    # subsumes it.
    run "semver" scripts/qa/semver.sh
    ;;
esac

case "$MODE" in
  --nightly)
    # Tier 2 locally. See docs/plan-agentic-qa-strategy.md §6.
    #
    # proptest: the existing property tests run under `cargo test` with a
    # reduced case count in Tier 1. Here we bump to 1000 cases per property.
    run "proptest-1000" env PROPTEST_CASES=1000 cargo test --all \
      --exclude dora-node-api-python \
      --exclude dora-operator-api-python \
      --exclude dora-ros2-bridge-python \
      --exclude dora-cli-api-python \
      --exclude dora-examples \
      -- proptest

    # miri requires nightly Rust + miri component. Skip (with note) if missing.
    run_optional "miri" "cargo +nightly miri --version" \
      cargo +nightly miri test -p dora-core

    # Full mutation testing (not diff-scoped). The default `qa-mutants` is
    # diff-scoped for the Tier 1 PR gate; --full runs every function.
    run "mutants (full-repo)" scripts/qa/mutants.sh --full
    ;;
esac

echo
echo "============================================================"
if [[ ${#FAILED[@]} -eq 0 ]]; then
  echo "All checks passed."
  exit 0
else
  echo "FAILED:"
  printf '  - %s\n' "${FAILED[@]}"
  exit 1
fi
