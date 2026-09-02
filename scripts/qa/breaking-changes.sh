#!/usr/bin/env bash
# scripts/qa/breaking-changes.sh -- the dora 1.x compatibility gate.
#
# dora 1.0 freezes a set of surfaces for the life of the 1.x series
# (docs/api-rust.md, "Stability scope at 1.0"). This checks all of them
# against the last released tag:
#
#   no compilation (seconds, runs on every PR)
#     - the C node API header
#     - the C++ node API (cxx bridge)
#     - the dataflow YAML schema
#     - the postcard wire format of dora-message
#     - the `dora` command surface (via its checked-in snapshot)
#     - the Python support floor (requires-python / abi3)
#     - a major version bump, which would silence the check below
#
#   compilation required (--full, minutes)
#     - the Rust API of the crates the guarantee covers (cargo-semver-checks)
#     - freshness of the two generated inputs above: the CLI snapshot and the
#       JSON schema. Without this a PR could break a surface *and* leave its
#       snapshot stale, so the cheap diff would see nothing.
#
# Usage:
#   scripts/qa/breaking-changes.sh                  # everything
#   scripts/qa/breaking-changes.sh --fast           # no-compile checks only
#   scripts/qa/breaking-changes.sh --baseline v1.0.0
#
# Env: BREAKING_BASELINE=<ref>, ALLOW_MAJOR_BUMP=1.

set -euo pipefail

cd "$(dirname "$0")/../.."

MODE=full
# A plain string, not an array: an empty array under `set -u` aborts on bash
# 3.2, which is what macOS ships -- `make qa-fast` would fail there on a clean
# tree.
BASELINE_OVERRIDE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --fast) MODE=fast ;;
    --full) MODE=full ;;
    --baseline)
      [[ $# -ge 2 ]] || { echo "--baseline needs a ref" >&2; exit 2; }
      BASELINE_OVERRIDE="$2"; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

# shellcheck source=baseline.sh
source "$(dirname "$0")/baseline.sh"
BASELINE=$(resolve_breaking_baseline "$BASELINE_OVERRIDE") || exit 2
FAILED=0

# Best-effort: the fallback covers surfaces the release predates, so a shallow
# checkout that cannot reach the PR base just leaves those checks skipped.
if [[ -n "${BREAKING_FALLBACK_BASELINE:-}" ]] &&
   ! git rev-parse -q --verify "${BREAKING_FALLBACK_BASELINE}^{commit}" >/dev/null 2>&1; then
  git fetch --quiet --depth=1 origin "$BREAKING_FALLBACK_BASELINE" 2>/dev/null || true
fi

# The differ is text-driven, so an upstream reformat could silently turn an
# extractor into a no-op and every surface would report "ok" forever. Its own
# tests feed known breaks through it; they take milliseconds, so they run
# first, every time, rather than being a separate thing to remember.
echo "=== differ self-test ==="
if ! python3 -m unittest discover -s scripts/qa/tests -q; then
  echo "the breaking-change differ is broken -- its findings cannot be trusted" >&2
  exit 2
fi
echo

echo "=== frozen surfaces (no build) ==="
python3 scripts/qa/breaking_changes.py --baseline "$BASELINE" || FAILED=1
echo

if [[ "$MODE" == fast ]]; then
  echo "Skipped (--fast): Rust API check, snapshot freshness."
  exit "$FAILED"
fi

# Generated inputs must match what generates them, or the diff above is
# comparing a stale file and passes for the wrong reason.
echo "=== generated inputs are current ==="
if [[ -n "${CARGO:-}" ]] || command -v cargo >/dev/null; then
  UPDATE_CLI_SURFACE=1 cargo test -p dora-cli --test cli_surface >/dev/null
  cargo run --quiet -p dora-core --bin generate_schema >/dev/null
  SCHEMAS="dora-schema.json libraries/core/dora-schema.json libraries/core/dora-node-schema.json binaries/cli/cli-surface.txt"
  if ! git diff --exit-code --stat -- $SCHEMAS; then
    echo
    echo "The generated surface files are stale. Regenerating them changed the"
    echo "tree, which means the check above compared an out-of-date snapshot."
    echo "Commit the regenerated files, then re-read the report."
    FAILED=1
  else
    echo "  CLI snapshot and JSON schemas match their generators."
  fi
else
  echo "  skipped: cargo not found"
fi
echo

echo "=== Rust API (cargo-semver-checks) ==="
# Delegated to semver.sh, which owns the crate list and the "which crates does
# the guarantee cover" reasoning. The crates it does *not* list are not gaps:
# dora-node-api-c and -cxx are checked above through their header and bridge,
# dora-node-api-python through the frozen module test in
# apis/python/node/tests/test_public_surface.py, and dora-cli through its
# command snapshot -- all surfaces rustdoc cannot see.
if ! BREAKING_BASELINE="$BASELINE" SEMVER_BASELINE="$BASELINE" scripts/qa/semver.sh; then
  FAILED=1
fi

echo
if [[ "$FAILED" == 1 ]]; then
  echo "Breaking changes found against $BASELINE."
  echo "dora 1.x freezes these surfaces -- see docs/api-rust.md."
  exit 1
fi
echo "No breaking changes against $BASELINE."
