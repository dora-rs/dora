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

# Only the crates covered by dora's 1.0 stability guarantee — see the
# "Stability scope at 1.0" section of docs/api-rust.md.
#
# Deliberately absent:
#   dora-core         internal; published only because dora-node-api and
#                     dora-cli depend on it, and cargo requires a published
#                     crate's dependencies to be published
#   dora-operator-api shipped outside the guarantee (experimental), so a
#                     breaking change here is expected, not a regression
#   dora-cli          its Rust lib target is internal; the surface 1.0 covers
#                     is the `dora` command and the dataflow YAML schema,
#                     neither of which cargo-semver-checks can see
PUBLIC_CRATES=(
  dora-node-api
  dora-message
  dora-arrow-convert
)

# Compare against the last released tag -- what deployed users are running.
# Resolved by the shared helper so both halves of the gate check the same
# release, and so this works in a shallow CI checkout: `git describe` fails
# there, which would have silently made the baseline `main`. Override by hand
# with SEMVER_BASELINE=<ref>.
# shellcheck source=baseline.sh
source "$(dirname "$0")/baseline.sh"
BASELINE=$(resolve_breaking_baseline) || exit 2
echo "Baseline: $BASELINE"
echo

FAILED=0
for crate in "${PUBLIC_CRATES[@]}"; do
  echo "--- $crate ---"
  # `--release-type minor` is what makes this mean anything. Left to derive
  # the release type from the version numbers, cargo-semver-checks sees that
  # the workspace version has already moved (an rc to its release, or 1.0 to
  # 1.1), concludes breakage is permitted, and skips every lint -- printing
  # "no semver update required" having run 0 of 254 checks. Forcing `minor`
  # runs the major-breaking lints on every run, which is the post-1.0 rule:
  # no breaking change without a deliberate 2.0.
  if ! cargo semver-checks check-release -p "$crate" \
      --baseline-rev "$BASELINE" --release-type minor; then
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
