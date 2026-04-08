#!/usr/bin/env bash
# scripts/qa/adversarial.sh — adversarial LLM review (local-first)
#
# Runs a different-model code review on the current diff as a fourth-eye
# check on AI-authored code. Uses whichever CLI is available:
#
#   1. codex (OpenAI Codex CLI)     — preferred, different provider
#   2. claude (Claude Code CLI)      — fallback
#
# Output:
#   - Printed to stdout
#   - Saved to /tmp/adversarial-review-<short-sha>.md
#   - Posted as PR comment when $GITHUB_ACTIONS=true (requires gh + PR ref)
#
# Usage:
#   scripts/qa/adversarial.sh                      # diff vs origin/main
#   scripts/qa/adversarial.sh --base HEAD~3        # diff vs HEAD~3
#   scripts/qa/adversarial.sh --diff mypatch.diff  # diff from a file
#   scripts/qa/adversarial.sh --backend claude     # force claude CLI
#
# Environment overrides:
#   ADVERSARIAL_BACKEND=codex|claude  — skip auto-detect
#   ADVERSARIAL_BASE=<ref>            — base for diff (default: origin/main)
#   ADVERSARIAL_MAX_DIFF_KB=200       — skip review if diff is larger

set -euo pipefail

cd "$(dirname "$0")/../.."

# ----- Parse args -----
BASE="${ADVERSARIAL_BASE:-origin/main}"
DIFF_FILE=""
BACKEND="${ADVERSARIAL_BACKEND:-}"
MAX_DIFF_KB="${ADVERSARIAL_MAX_DIFF_KB:-200}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --base) BASE="$2"; shift 2 ;;
    --diff) DIFF_FILE="$2"; shift 2 ;;
    --backend) BACKEND="$2"; shift 2 ;;
    --help|-h)
      sed -n '2,25p' "$0"
      exit 0
      ;;
    *) echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
done

# ----- Pick backend -----
if [[ -z "$BACKEND" ]]; then
  if command -v codex >/dev/null 2>&1; then
    BACKEND=codex
  elif command -v claude >/dev/null 2>&1; then
    BACKEND=claude
  else
    echo "error: neither 'codex' nor 'claude' CLI is installed." >&2
    exit 2
  fi
fi

# ----- Compute diff -----
if [[ -n "$DIFF_FILE" ]]; then
  if [[ ! -f "$DIFF_FILE" ]]; then
    echo "error: diff file not found: $DIFF_FILE" >&2
    exit 2
  fi
  DIFF=$(cat "$DIFF_FILE")
else
  if [[ "$BASE" == origin/* ]]; then
    git fetch --quiet origin "${BASE#origin/}" 2>/dev/null || true
  fi
  DIFF=$(git diff "${BASE}...HEAD" 2>/dev/null || git diff "$BASE" 2>/dev/null || true)
fi

if [[ -z "$DIFF" ]]; then
  echo "No diff against $BASE — nothing to review."
  exit 0
fi

DIFF_KB=$(( $(printf '%s' "$DIFF" | wc -c) / 1024 ))
if [[ "$DIFF_KB" -gt "$MAX_DIFF_KB" ]]; then
  echo "warning: diff is ${DIFF_KB}KB (> ${MAX_DIFF_KB}KB limit)."
  echo "Skipping adversarial review. Split the PR or raise ADVERSARIAL_MAX_DIFF_KB."
  exit 0
fi

echo "Adversarial review via $BACKEND (diff: ${DIFF_KB}KB vs $BASE)" >&2
echo >&2

# ----- Build prompt -----
PROMPT_FILE="$(dirname "$0")/adversarial-prompt.md"
if [[ ! -f "$PROMPT_FILE" ]]; then
  echo "error: prompt template not found: $PROMPT_FILE" >&2
  exit 2
fi
PROMPT_HEADER=$(cat "$PROMPT_FILE")
FULL_PROMPT="$PROMPT_HEADER
$DIFF"

# ----- Run the backend -----
SHORT_SHA=$(git rev-parse --short HEAD 2>/dev/null || echo "nocommit")
OUTPUT_FILE="/tmp/adversarial-review-${SHORT_SHA}.md"

case "$BACKEND" in
  codex)
    printf '%s' "$FULL_PROMPT" | codex exec - 2>/dev/null | tee "$OUTPUT_FILE"
    ;;
  claude)
    printf '%s' "$FULL_PROMPT" | claude -p 2>/dev/null | tee "$OUTPUT_FILE"
    ;;
  *)
    echo "error: unknown backend: $BACKEND" >&2
    exit 2
    ;;
esac

echo >&2
echo "Review written to $OUTPUT_FILE" >&2

# ----- Optional: post as PR comment in CI -----
if [[ "${GITHUB_ACTIONS:-}" == "true" && -n "${GITHUB_PR_NUMBER:-}" ]]; then
  if command -v gh >/dev/null 2>&1; then
    gh pr comment "$GITHUB_PR_NUMBER" --body-file "$OUTPUT_FILE" || \
      echo "warning: failed to post PR comment" >&2
  fi
fi
