#!/usr/bin/env bash
# scripts/qa/secret-files.sh — credential-file gate (#2194)
#
# Two checks over what git actually tracks, reported separately because
# they mean different things and need different fixes.
#
# Why this is a gate and not just a .gitignore rule: it already was a
# .gitignore rule, and that did not help. `.adora-token` — a real 64-hex
# coordinator bearer token — reached the public repo during the v1.0
# consolidation squash-merge (#2194) and was removed in #2225. .gitignore
# only suppresses files git is not already tracking, so anything arriving
# through an import, a squash of another repo's history, or a deliberate
# `git add -f` walks straight past it. What ships is the tree, so the tree
# is what this checks.
#
# Scope: filenames only. This is a fast, zero-false-positive backstop for
# the mistake that actually happened here, not a content scanner — a
# credential pasted into a .rs or .md file sails past it. The deeper
# control is GitHub's secret-scanning push protection, which is
# server-side, content-based, and blocks before the blob ever lands.
# Enable that too; it is a repo setting, not something expressible here.

set -euo pipefail

cd "$(dirname "$0")/../.."

status=0

# --- Check 1: tracked despite being ignored ---------------------------
# Treats .gitignore as the source of truth instead of restating it, so a
# token name added there is covered here for free. This is the exact
# shape of the #2194 failure: `.adora-token` was listed in .gitignore and
# tracked anyway. Not a credential finding on its own — .gitignore is
# mostly broad content rules (`*.png`, `examples/**/*.txt`) — so it gets
# its own wording rather than credential-rotation advice.
tracked_but_ignored="$(git ls-files -c -i --exclude-standard)"

if [ -n "$tracked_but_ignored" ]; then
  echo "error: git tracks files that .gitignore says to ignore:" >&2
  echo "$tracked_but_ignored" | sed 's/^/  /' >&2
  cat >&2 <<'EOF'

A tracked file matching an ignore rule was force-added, or arrived through
an import that bypassed the ignore rules. Either untrack it
(`git rm --cached <path>`) or, if it belongs in the repo, add a negation
(`!<path>`) to .gitignore so the rules say what is actually true.

If it turns out to hold a credential, see the rotation note below.
EOF
  status=1
fi

# --- Check 2: credential-shaped filenames -----------------------------
# Names .gitignore does not happen to carry. Anchored at the end so safe
# neighbours stay allowed: `.env.example`, `id_ed25519.pub`,
# `config.pem.template`.
PATTERN='(^|/)('
PATTERN+='\.env|\.env\.(local|dev|development|test|staging|prod|production)'
PATTERN+='|\.dora-token|\.adora-token'
PATTERN+='|\.netrc|\.npmrc|\.pypirc|credentials'
PATTERN+='|id_(rsa|dsa|ecdsa|ed25519)'
PATTERN+=')$|\.(pem|key|p12|pfx|jks|keystore)$'

credential_shaped="$(git ls-files | grep -iE "$PATTERN" || true)"

if [ -n "$credential_shaped" ]; then
  echo "error: git is tracking credential-shaped files:" >&2
  echo "$credential_shaped" | sed 's/^/  /' >&2
  cat >&2 <<'EOF'

If one of these is a real credential, removing it from the tree is not
enough — the value stays readable in history (`git show <commit>^:<path>`)
and in every clone and fork that already pulled. Rotate it first, then
remove the file.

If it is a fixture or a template, rename it so the name does not claim to
be a live credential (`.env.example`, `test-key.pem.template`, ...).
EOF
  status=1
fi

if [ "$status" -eq 0 ]; then
  echo "secret-files: OK (no credential-shaped or wrongly-tracked files)"
fi

exit "$status"
