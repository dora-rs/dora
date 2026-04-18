#!/usr/bin/env bash
# scripts/qa/all.sh — master QA runner
#
# Runs the QA gates from docs/plan-agentic-qa-strategy.md.
# Designed for local-first execution: same script runs locally and in CI.
#
# Modes:
#   --fast    pre-commit budget (~1 minute) — fmt, clippy, audit, unwrap, typos
#   --full    pre-push budget (~5-10 minutes) — fast + tests + coverage
#   --tier1   full Tier 1 gate (~15 minutes) — full + mutants + semver
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

if [[ "$MODE" == "--fast" ]]; then
  : # done
elif [[ "$MODE" == "--full" || "$MODE" == "--tier1" ]]; then
  run "test" cargo test --all \
    --exclude dora-node-api-python \
    --exclude dora-operator-api-python \
    --exclude dora-ros2-bridge-python \
    --exclude dora-cli-api-python \
    --exclude dora-examples
  run "coverage" scripts/qa/coverage.sh
  # Adversarial LLM review skips cleanly on machines without codex/claude.
  run "adversarial" scripts/qa/adversarial.sh --optional

  if [[ "$MODE" == "--tier1" ]]; then
    run "mutants" scripts/qa/mutants.sh
    run "semver"  scripts/qa/semver.sh
  fi
else
  echo "Unknown mode: $MODE"
  echo "Usage: $0 [--fast|--full|--tier1]"
  exit 2
fi

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
