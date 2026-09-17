#!/usr/bin/env bash
# scripts/qa/lockfile.sh — Cargo.lock freshness gate (#3512)
#
# Asserts that the committed Cargo.lock already satisfies every workspace
# manifest, i.e. that `--locked` builds work.
#
# Why this needs its own gate. Ordinary cargo commands -- build, check,
# test, clippy -- rewrite a stale lock in place and then succeed, so the
# repair never reaches a commit and nothing ever goes red. Only `--locked`
# refuses, and until this gate every `--locked` build lived in the nightly
# (the `msrv` job, and the cluster jobs' `cargo install --path
# binaries/cli --locked`). Those report 3-4 hours later, file a
# nightly-regression issue, and print a message that names neither the
# crate nor the change that stranded it -- so this script diffs the lock
# against a fresh resolve and names the packages itself.
#
# Why it is wired into a required status rather than only qa-fast: a lock
# entry can be valid on a PR branch and invalid once merged. #3512 was
# that shape -- #3360 added the `hang-before-init-node` fixture, which
# inherits `version.workspace = true`, as a brand-new `[[package]]` block
# pinned at 1.0.0, landing on a main already bumped to 1.0.1. New text
# conflicts with nothing, so the merge was clean and the result was a lock
# no branch had ever validated. `Check` is in `.trunk/trunk.yaml`
# `required_statuses` and ci.yml triggers on `push: trunk-merge/**`, so
# this runs on the batch branch -- the merged tree -- not just the PR
# head. Keep it that way: a `paths:` filter, an `if:` guard on the job, or
# dropping `Check` from `required_statuses` would each reopen this.
#
# Cost is a resolve with no compilation, ~0.4 s warm, and it needs no
# network (--locked already suppresses the index update).

set -euo pipefail

cd "$(dirname "$0")/../.."

if [ ! -f Cargo.lock ]; then
  echo "error: Cargo.lock is missing from the repository root" >&2
  exit 1
fi

# `cargo metadata` resolves the full dependency graph without building
# anything. With --locked it refuses to write the lock and fails if the
# committed one is not already a solution.
if metadata_err="$(cargo metadata --locked --format-version 1 2>&1 >/dev/null)"; then
  echo "lockfile: OK (Cargo.lock satisfies every workspace manifest)"
  exit 0
fi

echo "$metadata_err" | sed 's/^/  /' >&2

# cargo exits non-zero for plenty of reasons that have nothing to do with
# the lock -- a malformed manifest, an index fetch that timed out, no
# cargo on PATH. Reporting those as "your lock is stale" sends the reader
# after a fix that cannot work, so only the --locked refusal gets the
# lockfile diagnosis. Exit 2 for the rest, matching how every sibling gate
# separates "this gate cannot run here" from "this gate found a
# violation" (typos.sh, audit.sh, unwrap-budget.sh, package-includes.sh).
if ! grep -q -- '--locked was passed' <<<"$metadata_err"; then
  echo "error: could not resolve the dependency graph (not a lockfile problem)" >&2
  exit 2
fi

echo "error: Cargo.lock is out of date with the workspace manifests" >&2

# Name the packages. Cargo will not say which entry is stale, and that is
# the single most useful thing to know -- resolve into a scratch copy and
# diff the `name`/`version` pairs, so the reader gets
# `hang-before-init-node 1.0.0 -> 1.0.1` instead of a paragraph of prose.
# Best-effort: any failure here just costs the detail, not the gate.
lock_versions() {
  awk '/^name = /   { n = $3 }
       /^version = /{ if (n != "") { print n, $3; n = "" } }' "$1" |
    tr -d '"' | sort
}

scratch="$(mktemp -d)"
trap 'rm -rf "$scratch"' EXIT
if cp Cargo.lock "$scratch/before.lock" &&
  cargo metadata --format-version 1 >/dev/null 2>&1; then
  lock_versions "$scratch/before.lock" >"$scratch/before.versions"
  lock_versions Cargo.lock >"$scratch/after.versions"
  # Put the committed lock back before reporting: this is a read-only
  # gate, and a dev whose `qa-fast` silently fixed the tree would commit
  # the repair by accident without ever reading why it was needed.
  cp "$scratch/before.lock" Cargo.lock

  # Two one-sided diffs rather than a paired old -> new, because a name
  # can legitimately appear at several versions at once (quick-xml is in
  # here four times); pairing those would invent transitions.
  committed="$(comm -23 "$scratch/before.versions" "$scratch/after.versions" || true)"
  resolved="$(comm -13 "$scratch/before.versions" "$scratch/after.versions" || true)"
  if [ -n "$committed$resolved" ]; then
    echo >&2
    echo "Entries a fresh resolve would change:" >&2
    [ -n "$committed" ] && echo "$committed" | sed 's/^/  - committed: /' >&2
    [ -n "$resolved" ] && echo "$resolved" | sed 's/^/  + resolved:  /' >&2
  fi
fi

cat >&2 <<'EOF'

A `--locked` build cannot resolve this tree, so the nightly will fail even
though every local build passes -- plain cargo commands update the lock in
place instead of complaining.

Fix: run `cargo metadata --format-version 1 >/dev/null` (or any cargo
command) to refresh Cargo.lock, then commit the result alongside the
manifest change that caused it. A workspace version bump touches the lock
entry of every crate using `version.workspace = true`, including test-only
fixtures under tests/ and examples/ that you never build directly.
EOF
exit 1
