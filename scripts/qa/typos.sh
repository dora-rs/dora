#!/usr/bin/env bash
# scripts/qa/typos.sh — spelling/prose gate
#
# Wraps `crate-ci/typos` so `make qa-fast` (and every tier above it)
# runs the same prose check remote CI runs. Added per #1657 so
# docs-heavy changes don't need a separate manual step after qa-fast.
#
# Install:
#   cargo install typos-cli

set -euo pipefail

cd "$(dirname "$0")/../.."

if ! command -v typos >/dev/null; then
  echo "typos not installed. Run: cargo install typos-cli"
  exit 2
fi

typos
