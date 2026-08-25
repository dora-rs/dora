#!/usr/bin/env bash
# scripts/qa/ci-nightly-reporting.sh — structural check on nightly failure reporting
#
# The nightly workflow reports itself through two jobs that consume every
# other job via `needs`:
#
#   file-issue-on-failure   opens/updates the `nightly-regression` issue
#   close-issue-on-success  closes it again once the nightly is green
#
# Both read `needs.<job>.result`, and that indirection has silently lost
# regressions three separate times (#2742, #2987). This script asserts the
# three structural invariants those failures violated. It parses
# .github/workflows/nightly.yml only — it does not run any nightly job.
#
#   1. No job in `file-issue-on-failure`'s `needs` carries JOB-LEVEL
#      `continue-on-error: true`.
#
#      `needs.<job>.result` reads `success` for a continue-on-error job even
#      when it failed, so such a job is listed as reported but cannot ever
#      be reported. The two `ros2-zenoh-*` jobs failed every night for two
#      weeks this way (#2790 added them 2026-07-21, #2742 found them
#      2026-08-03) across thirteen nightly issues, none of which named them.
#
#      Deliberate exceptions live in `.nightly-advisory-jobs` (see #2987) —
#      a job may be advisory while it earns trust, but the exemption is
#      explicit, dated, and greppable rather than invisible.
#
#      STEP-level `continue-on-error` is unaffected and not checked: it does
#      not touch `needs.<job>.result`. `smoke-suite` uses it legitimately to
#      capture an exit code into a step output and re-raise it later, so a
#      naive `grep continue-on-error` would flag a correct job. The two are
#      told apart by indentation (4 spaces = job key, 8 = step key).
#
#   2. Every job in the workflow appears in `file-issue-on-failure`'s
#      `needs` (the two reporter jobs themselves excepted).
#
#      A job missing from that list is unmonitored in the same way, just via
#      omission instead of a flag. `hub-smoke` sat unreported this way from
#      the day it was added.
#
#   3. `close-issue-on-success`'s `needs` covers everything
#      `file-issue-on-failure`'s does.
#
#      The closer refuses to close while any job it watches is red. A job the
#      reporter watches but the closer does not produces a FLAPPING issue:
#      the nightly files it, and the next nightly closes it again while the
#      job is still broken. Failing that way is arguably worse than silence,
#      because the issue tracker actively asserts the problem is fixed.
#
# Exit codes: 0 all invariants hold, 1 a violation, 2 the workflow could not
# be parsed (missing file / renamed reporter job) — never a silent pass.

set -euo pipefail

cd "$(dirname "$0")/../.."

WORKFLOW=".github/workflows/nightly.yml"
EXCEPTIONS=".nightly-advisory-jobs"
REPORTER="file-issue-on-failure"
CLOSER="close-issue-on-success"

if [[ ! -f "$WORKFLOW" ]]; then
  echo "ci-nightly-reporting: $WORKFLOW not found" >&2
  exit 2
fi

# Flatten the workflow into three record types:
#   JOB  <job>         a top-level key under `jobs:`
#   COE  <job>         that job carries job-level `continue-on-error: true`
#   NEED <job> <dep>   <job> lists <dep> in its `needs:`
#
# Indentation is the whole grammar here: 2 spaces = job name, 4 = job key,
# 6 = `needs:` list item, 8 = step key. That is load-bearing (see invariant 1
# above), so the patterns anchor on exact leading whitespace rather than
# using a looser match.
parse_workflow() {
  awk '
    # --- track entry into / exit from the top-level `jobs:` mapping ---
    /^jobs:[[:space:]]*$/ { injobs = 1; next }
    injobs && /^[^[:space:]#]/ { injobs = 0; job = ""; inneeds = 0 }
    !injobs { next }

    # --- job header: `  <name>:` ---
    /^  [A-Za-z0-9_-]+:[[:space:]]*$/ {
      job = $0
      sub(/^  /, "", job)
      sub(/:[[:space:]]*$/, "", job)
      inneeds = 0
      print "JOB " job
      next
    }

    job == "" { next }

    # --- `needs:` block-list items: `      - <dep>` ---
    inneeds && /^      -[[:space:]]/ {
      dep = $0
      sub(/^      -[[:space:]]+/, "", dep)
      sub(/[[:space:]]+$/, "", dep)
      sub(/[[:space:]]*#.*$/, "", dep)
      if (dep != "") print "NEED " job " " dep
      next
    }
    # Any other job-level key ends the needs block. No `next` — the line
    # still has to be offered to the rules below.
    inneeds && /^    [A-Za-z]/ { inneeds = 0 }

    # --- `needs:` in its three YAML spellings ---
    /^    needs:[[:space:]]*$/ { inneeds = 1; next }
    /^    needs:[[:space:]]*\[/ {
      list = $0
      sub(/^    needs:[[:space:]]*\[/, "", list)
      sub(/\].*$/, "", list)
      n = split(list, parts, ",")
      for (i = 1; i <= n; i++) {
        gsub(/[[:space:]"'"'"']/, "", parts[i])
        if (parts[i] != "") print "NEED " job " " parts[i]
      }
      next
    }
    /^    needs:[[:space:]]*[A-Za-z0-9_-]+/ {
      dep = $0
      sub(/^    needs:[[:space:]]*/, "", dep)
      sub(/[[:space:]]*#.*$/, "", dep)
      sub(/[[:space:]]+$/, "", dep)
      if (dep != "") print "NEED " job " " dep
      next
    }

    # --- job-level `continue-on-error: true` (exactly 4 spaces) ---
    /^    continue-on-error:[[:space:]]*true[[:space:]]*$/ { print "COE " job; next }
  ' "$WORKFLOW"
}

RECORDS="$(parse_workflow)"

jobs_list()  { printf '%s\n' "$RECORDS" | awk '$1=="JOB"{print $2}' | sort -u; }
coe_list()   { printf '%s\n' "$RECORDS" | awk '$1=="COE"{print $2}' | sort -u; }
needs_of()   { printf '%s\n' "$RECORDS" | awk -v j="$1" '$1=="NEED" && $2==j {print $3}' | sort -u; }

ALL_JOBS="$(jobs_list)"
COE_JOBS="$(coe_list)"
REPORTER_NEEDS="$(needs_of "$REPORTER")"
CLOSER_NEEDS="$(needs_of "$CLOSER")"

# A rename or refactor that makes the reporter unparseable must not read as
# "all invariants hold" — that is precisely the silent-pass this script exists
# to prevent.
if ! printf '%s\n' "$ALL_JOBS" | grep -qx "$REPORTER"; then
  echo "ci-nightly-reporting: no '$REPORTER' job in $WORKFLOW" >&2
  echo "If the reporting job was renamed, update REPORTER in this script." >&2
  exit 2
fi
if ! printf '%s\n' "$ALL_JOBS" | grep -qx "$CLOSER"; then
  echo "ci-nightly-reporting: no '$CLOSER' job in $WORKFLOW" >&2
  echo "If the closing job was renamed, update CLOSER in this script." >&2
  exit 2
fi
if [[ -z "$REPORTER_NEEDS" ]]; then
  echo "ci-nightly-reporting: '$REPORTER' has an empty 'needs:' list" >&2
  echo "That reports nothing at all. Check the parser against $WORKFLOW." >&2
  exit 2
fi

# Advisory allowlist: blank lines and `#` comments ignored, one job per line.
ADVISORY=""
if [[ -f "$EXCEPTIONS" ]]; then
  ADVISORY="$(sed -e 's/[[:space:]]*#.*$//' -e '/^[[:space:]]*$/d' \
                  -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' "$EXCEPTIONS" | sort -u)"
fi

fail=0
note() { echo "  - $1"; }

# --- Invariant 1: nothing in the reporter's needs is silently advisory ------
advisory_violations=""
while IFS= read -r j; do
  [[ -z "$j" ]] && continue
  printf '%s\n' "$REPORTER_NEEDS" | grep -qx "$j" || continue
  printf '%s\n' "$ADVISORY" | grep -qx "$j" && continue
  advisory_violations+="$j"$'\n'
done <<< "$COE_JOBS"

if [[ -n "${advisory_violations//[$'\n']/}" ]]; then
  fail=1
  echo "FAIL: job-level 'continue-on-error: true' on jobs listed in '$REPORTER' needs:"
  while IFS= read -r j; do [[ -n "$j" ]] && note "$j"; done <<< "$advisory_violations"
  echo
  echo "  'needs.<job>.result' reads 'success' for a continue-on-error job, so"
  echo "  these are listed as reported but can never actually report. Either"
  echo "  drop the flag, or add the job to $EXCEPTIONS with a reason and issue"
  echo "  reference if it is deliberately advisory for now."
  echo
fi

# --- Invariant 2: every job is watched by the reporter ---------------------
unwatched=""
while IFS= read -r j; do
  [[ -z "$j" ]] && continue
  [[ "$j" == "$REPORTER" || "$j" == "$CLOSER" ]] && continue
  printf '%s\n' "$REPORTER_NEEDS" | grep -qx "$j" && continue
  unwatched+="$j"$'\n'
done <<< "$ALL_JOBS"

if [[ -n "${unwatched//[$'\n']/}" ]]; then
  fail=1
  echo "FAIL: nightly jobs missing from '$REPORTER' needs:"
  while IFS= read -r j; do [[ -n "$j" ]] && note "$j"; done <<< "$unwatched"
  echo
  echo "  A job absent from that list can fail every night without filing"
  echo "  anything. Add it to the '$REPORTER' needs list."
  echo
fi

# --- Invariant 3: the closer watches everything the reporter does ----------
unclosed=""
while IFS= read -r j; do
  [[ -z "$j" ]] && continue
  printf '%s\n' "$CLOSER_NEEDS" | grep -qx "$j" && continue
  unclosed+="$j"$'\n'
done <<< "$REPORTER_NEEDS"

if [[ -n "${unclosed//[$'\n']/}" ]]; then
  fail=1
  echo "FAIL: jobs watched by '$REPORTER' but not by '$CLOSER':"
  while IFS= read -r j; do [[ -n "$j" ]] && note "$j"; done <<< "$unclosed"
  echo
  echo "  The nightly issue would be filed when these fail and then closed"
  echo "  again by the next nightly while they are still failing. Add them to"
  echo "  the '$CLOSER' needs list."
  echo
fi

if [[ "$fail" -ne 0 ]]; then
  echo "ci-nightly-reporting: FAILED (see $WORKFLOW)" >&2
  exit 1
fi

job_count="$(printf '%s\n' "$ALL_JOBS" | grep -c . || true)"
advisory_count="$(printf '%s\n' "$ADVISORY" | grep -c . || true)"
echo "ci-nightly-reporting: OK — ${job_count} jobs, all reported and closable" \
     "(${advisory_count} documented advisory)"
