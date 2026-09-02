#!/usr/bin/env bash
# scripts/qa/baseline.sh -- resolve the release the compatibility gate compares
# against. Sourced by breaking-changes.sh and semver.sh, so the two halves of
# the gate can never disagree about which release they are checking.
#
#   source scripts/qa/baseline.sh
#   BASELINE=$(resolve_breaking_baseline)      # honours $BREAKING_BASELINE
#
# Also fetches the tag when it is missing. CI checkouts are shallow and carry
# no tags at all -- and `git describe` fails there, so a caller that resolved
# its own baseline would silently end up comparing against `main`, or against
# nothing.

resolve_breaking_baseline() {
  local ref="${1:-${SEMVER_BASELINE:-${BREAKING_BASELINE:-}}}"
  if [ -z "$ref" ]; then
    ref=$(python3 scripts/qa/breaking_changes.py --print-baseline) || return 1
  fi

  if ! git rev-parse -q --verify "${ref}^{commit}" >/dev/null 2>&1; then
    echo "fetching baseline $ref (not in this checkout)" >&2
    if ! git fetch --quiet --depth=1 origin "refs/tags/${ref}:refs/tags/${ref}" 2>/dev/null; then
      # Not a tag, or no such tag: try it as a plain commit-ish before giving up.
      git fetch --quiet --depth=1 origin "$ref" 2>/dev/null || {
        echo "error: could not fetch baseline $ref" >&2
        return 1
      }
    fi
  fi

  echo "$ref"
}
