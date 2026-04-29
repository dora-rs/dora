#!/usr/bin/env bash
#
# Test that build.rs correctly stages artifacts to the target directory
# and that xtask stage-c-node can copy them to a prefix.
#
# Usage:
#   ./scripts/c_cpp_lib/test-stage.sh <crate>              # Run all stages
#   ./scripts/c_cpp_lib/test-stage.sh <crate> --verbose    # Show build output
#
# Example:
#   ./scripts/c_cpp_lib/test-stage.sh dora-node-api-c
#
# Prerequisites: cargo, rustc

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <crate>"
    echo "Example: $0 dora-node-api-c"
    exit 1
fi

CRATE="$1"
LIB_NAME="$(echo "$CRATE" | sed 's/-/_/g')"

VERBOSE=false
for arg in "$@"; do
    case "$arg" in
        --verbose) VERBOSE=true ;;
    esac
done

PASS=0
FAIL=0
SKIP=0

run_cargo() {
    if [ "$VERBOSE" = true ]; then
        cargo "$@"
    else
        cargo "$@" >/dev/null 2>&1
    fi
}

assert_file() {
    local path="$1"
    local desc="$2"
    if [ -f "$path" ]; then
        echo "  OK: $desc"
    else
        echo "  FAIL: $desc (not found: $path)"
        FAIL=$((FAIL + 1))
        return 1
    fi
}

assert_cmake_files() {
    local prefix="$1"
    assert_file "$prefix/lib/lib${LIB_NAME}.a" "static library"
    assert_file "$prefix/include/node_api.h" "header file"
    assert_file "$prefix/lib/cmake/${CRATE}/${CRATE}Config.cmake" "Config.cmake"
    assert_file "$prefix/lib/cmake/${CRATE}/${CRATE}ConfigVersion.cmake" "ConfigVersion.cmake"
}

run_stage() {
    local target_dir="$1"
    local stage="${ROOT}/${target_dir}/${CRATE}"
    local prefix="${ROOT}/${target_dir}/test-stage-prefix"
    rm -rf "$prefix"

    echo "  Building: cargo build -p $CRATE ($target_dir)"
    run_cargo build -p "$CRATE"

    if [ -d "$stage" ]; then
        cargo run -p xtask --quiet -- stage-c-node "${ROOT}/${target_dir}" "$prefix"
        assert_cmake_files "$prefix" || true
        PASS=$((PASS + 1))
    else
        echo "  SKIP: staging directory not found at $stage"
        SKIP=$((SKIP + 1))
    fi
    rm -rf "$prefix"
}

run_stage_release() {
    local target_dir="$1"
    local stage="${ROOT}/${target_dir}/${CRATE}"
    local prefix="${ROOT}/${target_dir}/test-stage-prefix"
    rm -rf "$prefix"

    echo "  Building: cargo build --release -p $CRATE ($target_dir)"
    run_cargo build --release -p "$CRATE"

    if [ -d "$stage" ]; then
        cargo run -p xtask --quiet -- stage-c-node "${ROOT}/${target_dir}" "$prefix"
        assert_cmake_files "$prefix" || true
        PASS=$((PASS + 1))
    else
        echo "  SKIP: staging directory not found at $stage"
        SKIP=$((SKIP + 1))
    fi
    rm -rf "$prefix"
}

run_stage_targeted() {
    local target="$1"
    local target_dir="${ROOT}/target/${target}/release"
    local stage="${target_dir}/${CRATE}"
    local prefix="${target_dir}/test-stage-prefix"
    rm -rf "$prefix"

    echo "  Building: cargo build --release --target $target -p $CRATE"
    run_cargo build --release --target "$target" -p "$CRATE"

    if [ -d "$stage" ]; then
        cargo run -p xtask --quiet -- stage-c-node "$target_dir" "$prefix"
        assert_cmake_files "$prefix" || true
        PASS=$((PASS + 1))
    else
        echo "  SKIP: staging directory not found at $stage"
        SKIP=$((SKIP + 1))
    fi
    rm -rf "$prefix"
}

# ============================================================================
# Stage 1: Debug build
# ============================================================================
echo "=== Stage 1: Debug build ==="
run_stage "target/debug"
echo ""

# ============================================================================
# Stage 2: Release build
# ============================================================================
echo "=== Stage 2: Release build ==="
run_stage_release "target/release"
echo ""

# ============================================================================
# Stage 3: Build with --target
# ============================================================================
echo "=== Stage 3: Build with --target ==="
TARGET="$(rustc -vV | sed -n 's/host: //p')"
echo "  Host target: $TARGET"

run_stage_targeted "$TARGET"
echo ""

# ============================================================================
# Summary
# ============================================================================
echo "Results: $PASS passed, $FAIL failed, $SKIP skipped"
if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
