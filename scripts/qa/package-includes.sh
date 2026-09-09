#!/usr/bin/env bash
# scripts/qa/package-includes.sh — build-time file inclusion gate (#3400)
#
# `cargo package` builds the `.crate` tarball from one crate directory:
# it takes the git-tracked files under that directory and nothing else.
# A crate whose source or build script pulls in a file that is *not* in
# that set therefore compiles fine in this workspace and fails for every
# user who installs it from crates.io — after the version is uploaded and
# can no longer be replaced.
#
# #3400 is this: `dora-operator-api-c/build.rs` did
#
#     include_str!("../node/cmake/dora-api-config.cmake.in")
#
# reaching into the sibling `dora-node-api-c` crate. The workspace build
# was green, the published crate had no such file, and `cargo install
# dora-cli` was broken for everyone on 1.0.0.
#
# Two ways to lose a file, both checked here for every publishable crate:
#
#   1. The path escapes the crate directory (`../` into a sibling crate
#      or the repo root). Cargo cannot package files it does not own.
#   2. The path stays inside the crate but is not tracked by git — a
#      generated or ignored file. Cargo packages tracked files only, so
#      these vanish the same way.
#
# Two argument forms are understood: a plain string literal, resolved
# against the including file, and `concat!(env!("CARGO_MANIFEST_DIR"),
# "...")`, resolved against the crate root — the form that hides a `../`
# escape behind a macro. An `OUT_DIR`-rooted path is build-script output,
# created on the user's machine, and needs no packaging. Anything else is
# reported as unparsed rather than passed over silently, so a new form
# cannot quietly opt out of the gate.
#
# Bare `include!` is skipped: in this workspace it is almost always cxx's
# `include!("foo.h")` inside an `extern "C++"` block, which names a C++
# header for the C++ compiler's include path, not a file cargo ships.
#
# Comments are stripped before scanning, so prose that mentions
# `include_str!` is not mistaken for a call.

set -euo pipefail

cd "$(dirname "$0")/../.."

if ! command -v python3 >/dev/null 2>&1; then
  echo "scripts/qa/package-includes.sh requires python3 on PATH" >&2
  exit 2
fi

META="$(mktemp)"
TRACKED="$(mktemp)"
trap 'rm -f "$META" "$TRACKED"' EXIT
cargo metadata --format-version 1 --no-deps >"$META"
# The tracked set is both the list of sources to scan and the answer to
# "would cargo ship this include target?".
git ls-files >"$TRACKED"

python3 - "$META" "$TRACKED" <<'PY'
import json
import os
import re
import sys

meta_path, tracked_path = sys.argv[1:3]

with open(meta_path, encoding="utf-8") as f:
    meta = json.load(f)
root = meta["workspace_root"]

with open(tracked_path, encoding="utf-8") as f:
    tracked = {line.rstrip("\n") for line in f if line.strip()}

# `--no-deps` limits `packages` to workspace members. `publish == []` is
# how cargo metadata reports `publish = false`.
packages = meta["packages"]
pkg_dirs = {}  # repo-relative crate dir -> (name, publishable)
for p in packages:
    d = os.path.relpath(os.path.dirname(p["manifest_path"]), root)
    d = "" if d == "." else d
    pkg_dirs[d] = (p["name"], p.get("publish") != [])


def owner(rel_path):
    """Innermost crate containing `rel_path` — crates nest (a macros crate
    inside its parent), and the inner manifest is the one that packages it."""
    containing = (d for d in pkg_dirs if not d or rel_path.startswith(d + "/"))
    return max(containing, key=len, default=None)


ANY_INCLUDE = re.compile(r"\binclude_(?:str|bytes)!")
# `include_str!("path")` — relative to the file doing the including.
LITERAL = re.compile(r'\binclude_(?:str|bytes)!\s*\(\s*"([^"\\]*)"\s*,?\s*\)')
# `include_str!(concat!(env!("CARGO_MANIFEST_DIR"), "/path"))` — relative
# to the crate root. `OUT_DIR` in the same position is build output.
ROOTED = re.compile(
    r'\binclude_(?:str|bytes)!\s*\(\s*concat!\s*\(\s*env!\s*\(\s*"(CARGO_MANIFEST_DIR|OUT_DIR)"\s*\)\s*,'
    r'\s*"([^"\\]*)"\s*,?\s*\)\s*,?\s*\)'
)
LINE_COMMENT = re.compile(r"//[^\n]*")
BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.DOTALL)


def strip_comments(src):
    """Blank out comments, keeping newlines so line numbers stay right."""
    def blank(m):
        return re.sub(r"[^\n]", " ", m.group(0))

    return LINE_COMMENT.sub(blank, BLOCK_COMMENT.sub(blank, src))

violations = []
for rel in sorted(tracked):
    if not rel.endswith(".rs"):
        continue
    d = owner(rel)
    if d is None:
        continue
    name, publishable = pkg_dirs[d]
    if not publishable:
        continue
    with open(os.path.join(root, rel), encoding="utf-8", errors="replace") as f:
        src = strip_comments(f.read())

    # Scanned over the whole file, not line by line: rustfmt wraps a long
    # `include_str!(` onto the next line, and a per-line scan would miss it.
    resolved = {}  # match offset -> (shown path, repo-relative target)
    for m in LITERAL.finditer(src):
        literal = m.group(1)
        resolved[m.start()] = (literal, os.path.join(os.path.dirname(rel), literal))
    for m in ROOTED.finditer(src):
        var, literal = m.group(1), m.group(2)
        shown = f'concat!(env!("{var}"), "{literal}")'
        if var == "OUT_DIR":
            # Build-script output: created on the user's machine, never packaged.
            resolved[m.start()] = (shown, None)
        else:
            resolved[m.start()] = (shown, os.path.join(d, literal.lstrip("/")))

    for m in ANY_INCLUDE.finditer(src):
        lineno = src.count("\n", 0, m.start()) + 1
        if m.start() not in resolved:
            violations.append(
                f"{rel}:{lineno}: this `include_` call uses an argument form the "
                f"gate cannot resolve, so it cannot be checked.\n"
                f"    Fix: use a plain string literal, or teach "
                f"scripts/qa/package-includes.sh the new form."
            )
            continue
        shown, target = resolved[m.start()]
        if target is None:
            continue
        target = os.path.normpath(target)
        # `owner`, not a prefix test: a nested crate carries its own
        # manifest, so cargo leaves its subtree out of the parent tarball.
        if owner(target) != d:
            violations.append(
                f"{rel}:{lineno}: `{shown}` resolves to `{target}`, outside "
                f"crate `{name}` ({d or '.'}) — `cargo package` cannot ship it.\n"
                f"    Fix: keep a crate-local copy of the file and include that."
            )
        elif target not in tracked:
            violations.append(
                f"{rel}:{lineno}: `{shown}` resolves to `{target}`, which git "
                f"does not track — `cargo package` ships tracked files only.\n"
                f"    Fix: commit the file, or generate it into OUT_DIR instead."
            )

if violations:
    print("Files included at build time that the published crate would not have:\n")
    for v in violations:
        print(f"  {v}")
    print(
        "\nEach of these compiles here and not from the published crate: in a"
        "\nbuild script or library that breaks `cargo install` (#3400); under"
        "\n`#[cfg(test)]` it breaks `cargo test` for anyone building from"
        "\ncrates.io. Nothing can be re-uploaded once a version is published."
    )
    sys.exit(1)

print("package includes OK: every include_str!/include_bytes! target ships with its crate")
PY
