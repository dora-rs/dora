#!/usr/bin/env bash
#
# Build-test C and C++ node APIs using dora new + CMake.
#
# Generates nodes with `dora new`, stages libraries to ./temp, and builds.
#
# Usage:
#   ./scripts/c_cpp_lib/build-test.sh [--verbose]
#
# Prerequisites: cargo, rustc, cmake, dora CLI

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$ROOT"

export PATH="$ROOT/target/release:$PATH"

VERBOSE=false
if [[ "${1:-}" == "--verbose" ]]; then
    VERBOSE=true
fi

TEMP_DIR="${ROOT}/temp"
run_cargo() {
    if [ "$VERBOSE" = true ]; then
        cargo "$@"
    else
        cargo "$@" >/dev/null 2>&1
    fi
}

echo ""
echo "=== Build Test: C/C++ Node APIs ==="
echo "  TEMP_DIR: ${TEMP_DIR}"
echo ""

mkdir -p "$TEMP_DIR"

# ============================================================================
# Stage libraries
# ============================================================================
echo "--- Stage 1: Stage libraries ---"
echo ""

echo "  [dora-node-api-c]"
cargo run -p xtask --quiet -- stage dora-node-api-c \
    target/release \
    "${TEMP_DIR}/c-prefix"

echo "  [dora-node-api-cxx]"
cargo run -p xtask --quiet -- stage dora-node-api-cxx \
    target/release \
    "${TEMP_DIR}/cpp-prefix"

echo ""

# ============================================================================
# Build C test node (using dora new)
# ============================================================================
echo "--- Stage 2: Build C test node ---"

C_NODE_DIR="${TEMP_DIR}/c-node-test"
rm -rf "$C_NODE_DIR"

mkdir -p "$C_NODE_DIR"
cd "$C_NODE_DIR"
dora new my-robot --kind node --lang c 2>/dev/null
cd "$ROOT"

C_NODE_ACTUAL="${C_NODE_DIR}/my-robot"
mkdir -p "${C_NODE_ACTUAL}/build"
cd "${C_NODE_ACTUAL}/build"
echo "  Building C test node..."
cmake .. -DCMAKE_PREFIX_PATH="${TEMP_DIR}/c-prefix"
cmake --build . --config Release
echo "  C node build OK"

# ============================================================================
# Build C++ test node (using dora new)
# ============================================================================
echo ""
echo "--- Stage 3: Build C++ test node ---"

CPP_NODE_DIR="${TEMP_DIR}/cpp-node-test"
rm -rf "$CPP_NODE_DIR"
mkdir -p "$CPP_NODE_DIR"
cd "$CPP_NODE_DIR"

dora new my-robot-cpp --kind node --lang cxx 2>/dev/null
cd "$ROOT"

CPP_NODE_ACTUAL="${CPP_NODE_DIR}/my-robot-cpp"
mkdir -p "${CPP_NODE_ACTUAL}/build"
cd "${CPP_NODE_ACTUAL}/build"
echo "  Building C++ test node..."
cmake .. -DCMAKE_PREFIX_PATH="${TEMP_DIR}/cpp-prefix"
cmake --build . --config Release
echo "  C++ node build OK"

# ============================================================================
# Cleanup
# ============================================================================
echo ""
echo "=== Build Test Complete ==="
