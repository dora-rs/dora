#!/usr/bin/env bash
# scripts/qa/semver.sh — SemVer breakage check via cargo-semver-checks
#
# Compares public API surface of publishable crates against the most
# recent published version on crates.io. Soft warning during 0.x;
# becomes a hard gate post-1.0.
#
# Install: cargo install cargo-semver-checks

set -euo pipefail

cd "$(dirname "$0")/../.."

if ! command -v cargo-semver-checks >/dev/null; then
  echo "cargo-semver-checks not installed. Run: cargo install cargo-semver-checks"
  exit 2
fi

PUBLIC_CRATES=(
  adora-node-api
  adora-operator-api
  adora-core
  adora-message
  adora-arrow-convert
)

# adora-* crates are not yet published on crates.io, so use the last git
# tag as the baseline. Override by exporting SEMVER_BASELINE=<ref> before
# running this script.
BASELINE="${SEMVER_BASELINE:-$(git describe --tags --abbrev=0 2>/dev/null || echo main)}"
echo "Baseline: $BASELINE"
echo

FAILED=0
for crate in "${PUBLIC_CRATES[@]}"; do
  echo "--- $crate ---"
  if ! cargo semver-checks check-release -p "$crate" --baseline-rev "$BASELINE"; then
    FAILED=1
  fi
done

if [[ "$FAILED" == "1" ]]; then
  echo
  echo "SemVer check found breaking changes."
  echo "During 0.x this is a warning. After 1.0 this will be a hard fail."
  # Soft warning during 0.x — exit 0 even on breaks
  exit 0
fi
