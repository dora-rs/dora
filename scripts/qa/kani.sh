#!/usr/bin/env bash
# scripts/qa/kani.sh — formal verification via the Kani model checker
#
# Runs the `#[kani::proof]` harnesses embedded in the workspace
# (gated behind `cfg(kani)`, so they never affect normal builds).
# Unlike tests, a passing proof covers *every* input in the harness's
# state space, not just sampled values.
#
# Current proof crates (see docs/formal-verification.md for the
# inventory of properties and how to add new ones):
#
#   dora-message   libraries/message/src/auth.rs
#                  constant_time_eq == slice equality (bounded length)
#   dora-core      libraries/core/src/metadata.rs
#                  shared-memory buffer containment: soundness,
#                  completeness, panic/overflow freedom (exhaustive)
#
# Install: cargo install kani-verifier && cargo kani setup
#          (the setup step downloads the CBMC backend, ~couple hundred MB)
#
# Budget: ~5-10 min on a cold target dir (Kani recompiles the crate
# graph with its own rustc), well under a minute warm. Not part of the
# per-commit qa-fast loop; run it when touching a file that contains
# proofs, and in pre-release gating.

set -euo pipefail

cd "$(dirname "$0")/../.."

if ! cargo kani --version >/dev/null 2>&1; then
  echo "Kani not installed. Run: make qa-kani-install"
  echo "  (cargo install kani-verifier && cargo kani setup)"
  exit 2
fi

# `cargo kani --version` succeeds even when `cargo kani setup` (which
# downloads the CBMC backend into ~/.kani) was never run or was
# interrupted; catch that state with a friendly hint instead of a raw
# mid-run error.
if ! ls "${KANI_HOME:-$HOME/.kani}"/kani-*/ >/dev/null 2>&1; then
  echo "Kani is installed but not set up. Run: cargo kani setup"
  exit 2
fi

# Discover proof crates instead of hard-coding them, so adding a
# `#[kani::proof]` harness anywhere in the workspace is enough — a stale
# list here would silently skip new proofs and report a false green.
mapfile -t PROOF_CRATES < <(
  grep -rl --include='*.rs' '#\[kani::proof\]' apis binaries libraries \
    | while read -r file; do
        dir="$(dirname "$file")"
        while [[ ! -f "$dir/Cargo.toml" ]]; do dir="$(dirname "$dir")"; done
        sed -n 's/^name *= *"\(.*\)"/\1/p' "$dir/Cargo.toml" | head -1
      done | sort -u
)

if [[ ${#PROOF_CRATES[@]} -eq 0 ]]; then
  echo "No #[kani::proof] harnesses found in the workspace." >&2
  exit 1
fi

for crate in "${PROOF_CRATES[@]}"; do
  echo "==> cargo kani -p $crate"
  cargo kani -p "$crate"
done

echo "All Kani proof harnesses verified (crates: ${PROOF_CRATES[*]})."
