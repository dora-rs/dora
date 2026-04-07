#!/usr/bin/env bash
# scripts/qa/audit.sh — supply chain gate
#
# Runs cargo-audit (CVE check against RustSec advisory database)
# and cargo-deny (license, source, and dependency policy).
#
# Both tools must be installed:
#   cargo install cargo-audit cargo-deny

set -euo pipefail

cd "$(dirname "$0")/../.."

if ! command -v cargo-audit >/dev/null; then
  echo "cargo-audit not installed. Run: cargo install cargo-audit"
  exit 2
fi

if ! command -v cargo-deny >/dev/null; then
  echo "cargo-deny not installed. Run: cargo install cargo-deny"
  exit 2
fi

echo "--- cargo-audit (CVE scan) ---"
# cargo-audit reports unmaintained crates as warnings; cargo-deny enforces
# the ignore policy. Real vulnerabilities still cause cargo-audit to exit 1.
cargo audit

echo
echo "--- cargo-deny (policy enforcement) ---"
cargo deny check
