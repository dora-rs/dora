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

# Make `ref` readable in this checkout, fetching it at depth 1 if it is not.
ensure_ref() {
  local ref="$1"
  git rev-parse -q --verify "${ref}^{commit}" >/dev/null 2>&1 && return 0

  echo "fetching $ref (not in this checkout)" >&2
  git fetch --quiet --depth=1 origin "refs/tags/${ref}:refs/tags/${ref}" 2>/dev/null && return 0
  # Not a tag, or no such tag: try it as a plain commit-ish before giving up.
  git fetch --quiet --depth=1 origin "$ref" 2>/dev/null && return 0
  return 1
}

resolve_breaking_baseline() {
  local ref="${1:-${SEMVER_BASELINE:-${BREAKING_BASELINE:-}}}"
  if [ -z "$ref" ]; then
    ref=$(python3 scripts/qa/breaking_changes.py --print-baseline) || return 1
  fi

  ensure_ref "$ref" || {
    echo "error: could not fetch baseline $ref" >&2
    return 1
  }

  echo "$ref"
}
