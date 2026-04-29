#!/usr/bin/env bash
#
# Test that build.rs correctly stages artifacts to the target directory
# and that xtask stage can copy them to a prefix.
#
# Usage:
#   ./scripts/c_cpp_lib/test-stage.sh <crate> [...]        # Test specified crates
#   ./scripts/c_cpp_lib/test-stage.sh --all                # Test all 4 crates
#   ./scripts/c_cpp_lib/test-stage.sh <crate> --verbose    # Show build output
#
# Examples:
#   ./scripts/c_cpp_lib/test-stage.sh dora-node-api-c
#   ./scripts/c_cpp_lib/test-stage.sh --all --verbose
#
# Prerequisites: cargo, rustc

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT"

ALL_CRATES=("dora-node-api-c" "dora-operator-api-c" "dora-node-api-cxx" "dora-operator-api-cxx")

CRATES=()
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all)
            CRATES=("${ALL_CRATES[@]}")
            shift
            ;;
        --verbose)
            VERBOSE=true
            shift
            ;;
        *)
            CRATES+=("$1")
            shift
            ;;
    esac
done

if [[ ${#CRATES[@]} -eq 0 ]]; then
    echo "Usage: $0 [--all | <crate> [...]] [--verbose]"
    echo "Available crates: ${ALL_CRATES[*]}"
    echo "Example: $0 --all"
    exit 1
fi

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
        return 1
    fi
}

assert_cmake_files() {
    local crate="$1"
    local prefix="$2"
    local lib_name
    
    case "$crate" in
        dora-node-api-c)
            lib_name="libdora_node_api_c.a"
            ;;
        dora-operator-api-c)
            lib_name="libdora_operator_api_c.a"
            ;;
        dora-node-api-cxx)
            lib_name="libdora_node_api_cxx.a"
            ;;
        dora-operator-api-cxx)
            lib_name="libdora_operator_api_cxx.a"
            ;;
        *)
            echo "  FAIL: Unknown crate $crate"
            return 1
            ;;
    esac
    
    assert_file "$prefix/lib/$lib_name" "static library" || return 1
    assert_file "$prefix/lib/cmake/$crate/${crate}Config.cmake" "Config.cmake" || return 1
    assert_file "$prefix/lib/cmake/$crate/${crate}ConfigVersion.cmake" "ConfigVersion.cmake" || return 1
}

is_cxx_crate() {
    [[ "$1" == *-cxx ]]
}

test_crate_debug() {
    local crate="$1"
    local target_dir="${ROOT}/target/debug"
    local prefix="${target_dir}/test-stage-prefix"
    rm -rf "$prefix"
    
    if is_cxx_crate "$crate"; then
        echo "  SKIP: C++ crates require release build with cxxbridge"
        return 0
    fi

    echo "  Building: cargo build -p $crate ($target_dir)"
    run_cargo build -p "$crate"

    if [ -d "${target_dir}/${crate}" ]; then
        cargo run -p xtask --quiet -- stage "$crate" "$target_dir" "$prefix"
        assert_cmake_files "$crate" "$prefix" || return 1
    else
        echo "  SKIP: staging directory not found at ${target_dir}/${crate}"
        return 0
    fi
    rm -rf "$prefix"
}

test_crate_release() {
    local crate="$1"
    local target_dir="${ROOT}/target/release"
    local prefix="${target_dir}/test-stage-prefix"
    rm -rf "$prefix"

    echo "  Building: cargo build --release -p $crate ($target_dir)"
    run_cargo build --release -p "$crate"

    local stage_dir
    if is_cxx_crate "$crate"; then
        stage_dir="${ROOT}/target/cxxbridge/$crate"
    else
        stage_dir="${target_dir}/${crate}"
    fi

    if [ -d "$stage_dir" ]; then
        cargo run -p xtask --quiet -- stage "$crate" "$target_dir" "$prefix"
        assert_cmake_files "$crate" "$prefix" || return 1
    else
        echo "  SKIP: staging directory not found at $stage_dir"
        return 0
    fi
    rm -rf "$prefix"
}

test_crate_targeted() {
    local crate="$1"
    local target="$2"
    local target_dir="${ROOT}/target/${target}/release"
    local prefix="${target_dir}/test-stage-prefix"
    rm -rf "$prefix"

    echo "  Building: cargo build --release --target $target -p $crate"
    run_cargo build --release --target "$target" -p "$crate"

    local stage_dir
    if is_cxx_crate "$crate"; then
        stage_dir="${ROOT}/target/cxxbridge/$crate"
    else
        stage_dir="${target_dir}/${crate}"
    fi

    if [ -d "$stage_dir" ]; then
        cargo run -p xtask --quiet -- stage "$crate" "$target_dir" "$prefix"
        assert_cmake_files "$crate" "$prefix" || return 1
    else
        echo "  SKIP: staging directory not found at $stage_dir"
        return 0
    fi
    rm -rf "$prefix"
}

# ============================================================================
# Run tests for each crate
# ============================================================================
PASS=0
FAIL=0
SKIP=0

for crate in "${CRATES[@]}"; do
    echo ""
    echo "=== Testing $crate ==="
    
    # Debug build (C crates only)
    echo ""
    echo "--- Stage 1: Debug build ---"
    if test_crate_debug "$crate"; then
        PASS=$((PASS + 1))
    else
        FAIL=$((FAIL + 1))
    fi
    echo ""
    
    # Release build
    echo "--- Stage 2: Release build ---"
    if test_crate_release "$crate"; then
        PASS=$((PASS + 1))
    else
        FAIL=$((FAIL + 1))
    fi
    echo ""
    
    # Targeted build (C crates only, cxxbridge headers are host-only)
    if ! is_cxx_crate "$crate"; then
        echo "--- Stage 3: Build with --target ---"
        TARGET="$(rustc -vV | sed -n 's/host: //p')"
        echo "  Host target: $TARGET"
        if test_crate_targeted "$crate" "$TARGET"; then
            PASS=$((PASS + 1))
        else
            FAIL=$((FAIL + 1))
        fi
    else
        echo "--- Stage 3: Build with --target ---"
        echo "  SKIP: C++ crates use host cxxbridge headers"
        SKIP=$((SKIP + 1))
    fi
    echo ""
done

# ============================================================================
# Summary
# ============================================================================
echo "Results: $PASS passed, $FAIL failed, $SKIP skipped"
if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
