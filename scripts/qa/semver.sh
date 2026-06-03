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
  dora-node-api
  dora-operator-api
  dora-core
  dora-message
  dora-arrow-convert
)

# dora-* crates are not yet published on crates.io, so use the last git
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
  # Soft warning during 0.x; hard gate once the workspace hits 1.0 (API
  # stability is the 1.0 promise). Read the workspace version from the root
  # Cargo.toml ([workspace.package] version).
  WORKSPACE_VERSION=$(awk -F'"' '/^version = /{print $2; exit}' Cargo.toml)
  echo "SemVer check found breaking changes (workspace version: ${WORKSPACE_VERSION:-unknown})."
  case "$WORKSPACE_VERSION" in
    0.*|"")
      echo "During 0.x this is a soft warning (exit 0)."
      exit 0
      ;;
    *)
      echo "Post-1.0: breaking changes are a hard failure. Bump the major"
      echo "version or revert the breaking change."
      exit 1
      ;;
  esac
fi
