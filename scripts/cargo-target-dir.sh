# shellcheck shell=bash
#
# Resolve the cargo target directory for the workspace.
#
# The QA and smoke scripts locate prebuilt binaries by path, so hardcoding
# `<repo>/target` breaks them whenever the target dir moves -- a
# CARGO_TARGET_DIR override, or a `build.target-dir` / `build.build-dir` in
# some .cargo/config.toml (e.g. one target dir shared across git worktrees, so
# that deps are not rebuilt per worktree).
#
# `cargo metadata` is authoritative: it already accounts for CARGO_TARGET_DIR
# and every config source cargo itself honors. We only fall back to
# `<root>/target` when cargo cannot answer at all (cargo or python3 missing, no
# manifest reachable), which reproduces the previous hardcoded behavior.
#
# Usage:
#   source "$SCRIPT_DIR/cargo-target-dir.sh"
#   TARGET_DIR="$(cargo_target_dir "$ROOT")"
cargo_target_dir() {
    local root="${1:-$PWD}"
    local dir

    # A failure anywhere in this chain -- no cargo, no python3, no reachable
    # manifest -- leaves $dir empty, and the fallback below takes over.
    dir="$(cd "$root" 2> /dev/null \
        && cargo metadata --format-version 1 --no-deps 2> /dev/null \
        | python3 -c 'import json,sys; print(json.load(sys.stdin)["target_directory"])' 2> /dev/null)" \
        || dir=""

    printf '%s\n' "${dir:-${CARGO_TARGET_DIR:-$root/target}}"
}
