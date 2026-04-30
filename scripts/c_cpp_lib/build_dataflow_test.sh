#!/usr/bin/env bash
#
# Build-test C and C++ dataflow APIs using dora new + CMake.
#
# Generates dataflows with `dora new --kind dataflow`, stages libraries to ./temp,
# and builds all nodes via root CMakeLists.txt with add_subdirectory.
#
# Usage:
#   ./scripts/c_cpp_lib/build_dataflow_test.sh [--verbose]
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
echo "=== Build Test: C/C++ Dataflow APIs ==="
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
# Build C dataflow (using dora new --kind dataflow)
# ============================================================================
echo "--- Stage 2: Build C dataflow ---"

C_DATAFLOW_DIR="${TEMP_DIR}/c-dataflow-test"
rm -rf "$C_DATAFLOW_DIR"

mkdir -p "$C_DATAFLOW_DIR"
cd "$C_DATAFLOW_DIR"
dora new my-c-dataflow --kind dataflow --lang c 2>/dev/null
cd "$ROOT"

C_DATAFLOW_ACTUAL="${C_DATAFLOW_DIR}/my-c-dataflow"
mkdir -p "${C_DATAFLOW_ACTUAL}/build"
cd "${C_DATAFLOW_ACTUAL}/build"
echo "  Building C dataflow..."
cmake .. -DCMAKE_PREFIX_PATH="${TEMP_DIR}/c-prefix"
cmake --build . --config Release
echo "  C dataflow build OK"

# ============================================================================
# Build C++ dataflow (using dora new --kind dataflow)
# ============================================================================
echo ""
echo "--- Stage 3: Build C++ dataflow ---"

CPP_DATAFLOW_DIR="${TEMP_DIR}/cpp-dataflow-test"
rm -rf "$CPP_DATAFLOW_DIR"
mkdir -p "$CPP_DATAFLOW_DIR"
cd "$CPP_DATAFLOW_DIR"

dora new my-cpp-dataflow --kind dataflow --lang cxx 2>/dev/null
cd "$ROOT"

CPP_DATAFLOW_ACTUAL="${CPP_DATAFLOW_DIR}/my-cpp-dataflow"
mkdir -p "${CPP_DATAFLOW_ACTUAL}/build"
cd "${CPP_DATAFLOW_ACTUAL}/build"
echo "  Building C++ dataflow..."
cmake .. -DCMAKE_PREFIX_PATH="${TEMP_DIR}/cpp-prefix"
cmake --build . --config Release
echo "  C++ dataflow build OK"

# ============================================================================
# Summary
# ============================================================================
echo ""
echo "=== Build Test Complete ==="
