#!/usr/bin/env bash
#
# Smoke-test all safe example dataflows using the same high-level behavior as
# `tests/example-smoke.rs`.
#
# Networked lifecycle:
#   dora up -> dora build -> dora start --detach -> poll dora list
#   -> dora stop -> dora down
#
# What PASS means: the dataflow ran without crashing (no "Failed" state /
# nonzero exit) for the smoke window. It does NOT assert correct OUTPUT --
# the semantic contract assertions live in the Rust suite
# (`cargo test --test example-smoke`, the contract_* tests). Treat a green
# smoke-all.sh as "nothing blew up", not "output verified".
#
# Usage:
#   ./scripts/smoke-all.sh               # Run all examples
#   ./scripts/smoke-all.sh --rust-only   # Rust examples only
#   ./scripts/smoke-all.sh --python-only # Python examples only
#   ./scripts/smoke-all.sh --verbose     # Stream dora stdout/stderr live
#
# Prerequisites: cargo. For Python examples also `uv` and `maturin` -- the
# script builds the WORKSPACE Python bindings into target/smoke-venv so dep-less
# Python nodes (which run in the ambient env, not a per-node managed env) use the
# workspace dora instead of whatever `dora-rs` is otherwise installed. If uv or
# maturin is missing, Python examples are still attempted but may fail.
# Skips examples that need webcam, CUDA, ROS2, C/C++ toolchain, etc.

set -euo pipefail

# Surface the failing line if a build/setup step aborts the run (set -e).
# Example pass/fail is handled explicitly via log_fail; this only fires on an
# unguarded failure such as a broken cargo build, which would otherwise abort
# with no summary and no obvious cause.
trap 'rc=$?; echo; echo "smoke-all.sh: aborted (exit $rc) near line $LINENO -- likely a build/setup failure above"; exit $rc' ERR

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT"

PASS=0
FAIL=0
SKIP=0
FAILURES=()
RUN_RUST=true
RUN_PYTHON=true
VERBOSE=false
# Default ON: this script is for humans running smokes locally, so they
# should see what the dataflow actually did. CI/nightly uses the Rust
# suite at tests/example-smoke.rs, not this script.
SHOW_OUTPUT=true

for arg in "$@"; do
    case "$arg" in
        --rust-only)      RUN_PYTHON=false ;;
        --python-only)    RUN_RUST=false ;;
        -v|--verbose)     VERBOSE=true ;;
        -q|--quiet)       SHOW_OUTPUT=false ;;
        -s|--show-output) SHOW_OUTPUT=true ;;  # kept for back-compat; now a no-op by default
        -h|--help)
            cat <<'USAGE'
Usage: ./scripts/smoke-all.sh [OPTIONS]

  --rust-only         Run Rust examples only.
  --python-only       Run Python examples only.
  -v, --verbose       Stream dora stdout/stderr live (includes coordinator
                      and daemon chatter). Use when debugging a hang.
  -q, --quiet         Suppress the tail of dora output after each PASS.
                      Keeps the step progress + PASS/FAIL summary.
  -s, --show-output   (default) Dump the tail of dora output after each
                      example. Kept as an explicit opt-in for symmetry
                      with -q; equivalent to the default.
  -h, --help          This message.
USAGE
            exit 0
            ;;
        *) echo "Unknown option: $arg"; exit 1 ;;
    esac
done

# Startup hint so the developer knows the escape hatches.
if [ "$VERBOSE" = false ] && [ "$SHOW_OUTPUT" = true ]; then
    echo "(tip: -v streams dora live; -q suppresses the per-example tail)"
fi

DORA="${DORA_BIN:-$ROOT/target/debug/dora}"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

log_pass() { echo "  PASS: $1"; PASS=$((PASS + 1)); }
log_fail() { echo "  FAIL: $1"; FAIL=$((FAIL + 1)); FAILURES+=("$1"); }
log_skip() { echo "  SKIP: $1 ($2)"; SKIP=$((SKIP + 1)); }

# Run a dora sub-command. In --verbose mode, stream output live. Otherwise
# capture it into a file so we can dump it on failure.
#
#   run_dora_step <logfile> <args...>
#
# Returns the sub-command's exit status.
run_dora_step() {
    local logfile="$1"
    shift
    if [ "$VERBOSE" = true ]; then
        "$DORA" "$@" 2>&1 | tee -a "$logfile"
        return "${PIPESTATUS[0]}"
    else
        "$DORA" "$@" > "$logfile" 2>&1
    fi
}

# Dump the last N lines of a captured log file, indented. Used on failure
# so the developer sees WHY the step failed without having to re-run in
# --verbose mode.
dump_tail() {
    local logfile="$1" lines="${2:-20}"
    if [ -s "$logfile" ]; then
        echo "    ---- last $lines lines ----"
        tail -n "$lines" "$logfile" | sed 's/^/    | /'
        echo "    --------------------------"
    fi
}

cleanup_stale() {
    "$DORA" stop --all > /dev/null 2>&1 || true
    # `destroy` is just an alias for `down`; one call is enough.
    "$DORA" down > /dev/null 2>&1 || true
    # Backstop: a daemon does not always self-exit the instant its coordinator
    # goes away (see dora-rs/dora#1996), so reap any repo-local coordinator/daemon
    # stragglers. Otherwise they hold port 6013 or leave stale "Running" entries
    # that contaminate the next example's verdict.
    pkill -f "$ROOT/target/debug/dora .*(coordinator|daemon)" > /dev/null 2>&1 || true
    pkill -f "$ROOT/target/debug/dora-(coordinator|daemon|runtime)" > /dev/null 2>&1 || true
    # Reap orphan example nodes that bind a FIXED port -- e.g. the MAVLink bridge
    # on udp:14550. If one lingers (hard-killed local run, or not self-exiting on
    # coordinator loss), the next mavlink example dies with "Address already in
    # use". These patterns only match the mavlink example binaries, so they're
    # no-ops for every other example.
    pkill -f "$ROOT/target/(debug|release)/dora-mavlink2-bridge-node" > /dev/null 2>&1 || true
    pkill -f "$ROOT/target/(debug|release)/mavlink2-bridge-example" > /dev/null 2>&1 || true
    sleep 0.5
}

needs_uv() {
    local yaml="$1"
    python3 - "$yaml" <<'PY'
import pathlib
import sys

content = pathlib.Path(sys.argv[1]).read_text()
sys.exit(0 if ".py" in content or "pip install" in content else 1)
PY
}

# Python examples must run against the WORKSPACE dora bindings. Under `--uv`,
# dora only builds a per-node managed env (with the workspace dora wheel) for
# nodes that declare pip/build deps; a dep-less Python node instead runs in the
# AMBIENT Python env. So those nodes pick up whatever `dora-rs` happens to be
# installed there -- a stale editable install, nothing, or the PyPI package --
# none of which matches a workspace-built daemon, e.g. "version mismatch:
# message format vX.Y.Z is not compatible with v1.0.0-rc1" (or
# ModuleNotFoundError if dora isn't installed at all).
# Fix: build + activate a venv with the workspace `-e apis/python/node` so the
# ambient env those nodes resolve has the workspace bindings. Mirrors
# scripts/qa/ci-nightly-jobs.sh. Idempotent.
PY_BINDINGS_READY=false
PY_BINDINGS_OK=false
ensure_python_bindings() {
    [ "$PY_BINDINGS_READY" = true ] && return 0
    PY_BINDINGS_READY=true
    if ! command -v uv > /dev/null 2>&1; then
        echo "  WARN: uv not found -- Python examples use the ambient dora-rs, which may"
        echo "        version-mismatch the workspace daemon (install uv to fix)."
        return 0
    fi
    if ! command -v maturin > /dev/null 2>&1; then
        echo "  WARN: maturin not found -- cannot build workspace Python bindings;"
        echo "        Python examples may fail with a version mismatch (pip install maturin)."
        return 0
    fi
    local venv="$ROOT/target/smoke-venv"
    echo "Setting up workspace Python bindings in target/smoke-venv (maturin build, first time ~1-3 min)..."
    uv venv --seed -p 3.12 "$venv" > /dev/null 2>&1 || uv venv --seed "$venv" > /dev/null 2>&1
    # shellcheck disable=SC1091
    source "$venv/bin/activate"
    uv pip install -q pyarrow numpy > /dev/null 2>&1 || true
    if uv pip install -q -e "$ROOT/apis/python/node" > /dev/null 2>&1; then
        PY_BINDINGS_OK=true
        echo "  workspace dora-rs: $(python -c 'import dora; print(dora.__version__)' 2>/dev/null || echo '?')"
    else
        echo "  WARN: failed to build workspace Python bindings; Python examples may version-mismatch."
    fi
}

# Re-pin the workspace bindings into the active venv. Some examples' build deps
# pull dora-rs from PyPI and clobber the local version, so call this after a
# Python example's `dora build` and before its `dora start`/`run`.
repin_python_bindings() {
    [ "$PY_BINDINGS_OK" = true ] || return 0
    uv pip install -q -e "$ROOT/apis/python/node" > /dev/null 2>&1 || true
}

# Run a dataflow through the full up/start/stop/down lifecycle (networked).
run_networked() {
    local name="$1" yaml="$2" timeout="${3:-30}"
    echo "=> $name (networked, ${timeout}s)"

    local full_yaml="$ROOT/$yaml"
    local uv_args=()
    if needs_uv "$full_yaml"; then
        uv_args=(--uv)
        ensure_python_bindings
    fi

    local logfile
    logfile=$(mktemp -t "smoke-${name}.XXXXXX")
    # Always clean the log up on return, no matter which branch we exit through.
    trap 'rm -f "$logfile"' RETURN

    cleanup_stale

    printf '  up'
    if ! run_dora_step "$logfile" up; then
        echo " FAIL"
        dump_tail "$logfile"
        log_fail "$name (dora up failed)"
        return
    fi
    printf ' ok |  build'

    if ! run_dora_step "$logfile" build "$full_yaml" ${uv_args[@]+"${uv_args[@]}"}; then
        echo " FAIL"
        dump_tail "$logfile"
        log_fail "$name (dora build failed)"
        cleanup_stale
        return
    fi
    # The build may have pulled PyPI dora-rs; re-pin the workspace bindings.
    [ ${#uv_args[@]} -gt 0 ] && repin_python_bindings
    printf ' ok |  start'

    if ! run_dora_step "$logfile" start "$full_yaml" --detach ${uv_args[@]+"${uv_args[@]}"}; then
        echo " FAIL"
        dump_tail "$logfile"
        log_fail "$name (dora start failed)"
        cleanup_stale
        return
    fi
    printf ' ok | running'

    # Poll until dataflow finishes or timeout. Emit a heartbeat dot + elapsed
    # count every 2s so the developer can see it's alive. The line is rewritten
    # in place via \r so it doesn't flood stdout.
    local elapsed=0
    local failed=false
    while [ "$elapsed" -lt "$timeout" ]; do
        sleep 2
        elapsed=$((elapsed + 2))
        printf '\r  up ok |  build ok |  start ok | running %ds/%ds ' "$elapsed" "$timeout"
        local list_out
        # NOTE: `dora list --json` exits non-zero (2) even on success when there
        # are no dataflows, so the exit code is NOT a reliable failure signal --
        # parse stdout only, matching the authoritative Rust suite
        # (tests/example-smoke.rs, which uses .output().ok() and ignores status).
        list_out=$("$DORA" list --json 2>/dev/null || true)
        if echo "$list_out" | grep -q "Failed"; then
            failed=true
            break
        fi
        # Nothing left Running (empty list or all Finished) -> done.
        if ! echo "$list_out" | grep -q "Running"; then
            break
        fi
    done
    echo ""

    cleanup_stale

    if [ "$failed" = true ]; then
        dump_tail "$logfile"
        log_fail "$name"
    else
        [ "$SHOW_OUTPUT" = true ] && dump_tail "$logfile" 15
        log_pass "$name"
    fi
}

# Run a dataflow with dora run --stop-after (local/in-process mode).
# Uses a hard kill timeout (2x stop-after) to prevent hangs.
run_local() {
    local name="$1" yaml="$2" timeout="${3:-15}"
    local hard_timeout=$((timeout * 2 + 5))
    local full_yaml="$ROOT/$yaml"
    local uv_args=()
    if needs_uv "$full_yaml"; then
        uv_args=(--uv)
        ensure_python_bindings
    fi
    echo "=> $name (local, ${timeout}s, hard-kill ${hard_timeout}s)"

    local logfile
    logfile=$(mktemp -t "smoke-${name}.XXXXXX")
    trap 'rm -f "$logfile"' RETURN

    # Always run dora directly into the logfile in the background so $pid is
    # dora's PID. Piping to `tee` would make $pid the tee PID, so `wait` would
    # return tee's (always-0) status and mask a node-level failure as PASS, and
    # the hard-kill would target tee, orphaning dora. In verbose mode, stream
    # the log live with a separate `tail -f`.
    "$DORA" run "$full_yaml" --stop-after "${timeout}s" \
        ${uv_args[@]+"${uv_args[@]}"} > "$logfile" 2>&1 &
    local pid=$!
    local tail_pid=""
    if [ "$VERBOSE" = true ]; then
        tail -f "$logfile" &
        tail_pid=$!
    fi
    local elapsed=0
    while kill -0 "$pid" 2>/dev/null; do
        if [ "$elapsed" -ge "$hard_timeout" ]; then
            pkill -9 -P "$pid" 2>/dev/null || true   # reap spawned nodes/daemon
            kill -9 "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
            [ -n "$tail_pid" ] && kill "$tail_pid" 2>/dev/null || true
            echo ""
            dump_tail "$logfile"
            log_fail "$name (hard-killed after ${hard_timeout}s)"
            return
        fi
        sleep 1
        elapsed=$((elapsed + 1))
        if [ "$VERBOSE" != true ]; then
            printf '\r  running %ds/%ds (hard-kill %ds) ' "$elapsed" "$timeout" "$hard_timeout"
        fi
    done
    [ "$VERBOSE" != true ] && echo ""
    # Guarded wait: capture dora's real exit status without letting `set -e`
    # abort the whole script when one example exits nonzero (which would kill
    # the run mid-way with no FAIL line and no summary).
    local rc=0
    wait "$pid" 2>/dev/null || rc=$?
    [ -n "$tail_pid" ] && kill "$tail_pid" 2>/dev/null || true
    if [ "$rc" -eq 0 ]; then
        [ "$SHOW_OUTPUT" = true ] && dump_tail "$logfile" 15
        log_pass "$name"
    else
        dump_tail "$logfile"
        log_fail "$name"
    fi
}

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

echo "Building CLI..."
cargo build -p dora-cli 2>&1 | tail -1

echo "Running CLI unit tests..."
cargo test -p dora-cli 2>&1 | tail -3

if [ "$RUN_RUST" = true ]; then
    echo "Building Rust example nodes..."
    cargo build \
        -p rust-dataflow-example-node \
        -p rust-dataflow-example-status-node \
        -p rust-dataflow-example-sink \
        -p rust-dataflow-example-sink-dynamic \
        2>&1 | tail -1

    echo "Building benchmark nodes (release)..."
    cargo build --release \
        -p benchmark-example-node \
        -p benchmark-example-sink \
        2>&1 | tail -1

    echo "Building log-sink nodes..."
    cargo build \
        -p log-sink-file \
        -p log-sink-alert \
        -p log-sink-tcp \
        2>&1 | tail -1

    echo "Building service example nodes..."
    cargo build \
        -p service-example-client \
        -p service-example-server \
        2>&1 | tail -1

    echo "Building action example nodes..."
    cargo build \
        -p action-example-client \
        -p action-example-server \
        2>&1 | tail -1

    echo "Building validated-pipeline nodes..."
    cargo build \
        -p validated-pipeline-source \
        -p validated-pipeline-transform \
        -p validated-pipeline-sink \
        2>&1 | tail -1
fi

# mavlink2-bridge example: Rust variant runs in --rust-only mode; Python
# variant adds the python printer; C++ variant is its own cargo example.
# Build the Rust nodes (sim + emitter + bridge + rust printer) whenever
# RUN_RUST is on; they're shared across the rust + python YAML variants.
if [ "$RUN_RUST" = true ]; then
    echo "Building mavlink2-bridge example nodes..."
    cargo build \
        -p dora-mavlink2-bridge-node \
        -p mavlink2-bridge-example-mavlink-sim \
        -p mavlink2-bridge-example-heartbeat-emitter \
        -p mavlink2-bridge-example-telemetry-printer-rust \
        2>&1 | tail -1
fi

# ---------------------------------------------------------------------------
# Rust examples
# ---------------------------------------------------------------------------

if [ "$RUN_RUST" = true ]; then
    echo ""
    echo "=== Rust examples (networked) ==="
    run_networked "rust-dataflow"         "examples/rust-dataflow/dataflow.yml"
    run_networked "rust-dataflow-dynamic" "examples/rust-dataflow/dataflow_dynamic.yml"
    run_networked "rust-dataflow-url"     "examples/rust-dataflow-url/dataflow.yml"
    run_networked "benchmark"             "examples/benchmark/dataflow.yml"

    echo ""
    echo "=== Log-sink examples (networked, Rust + Python) ==="
    run_networked "log-sink-file"  "examples/log-sink-file/dataflow.yml"
    run_networked "log-sink-alert" "examples/log-sink-alert/dataflow.yml"
    run_networked "log-sink-tcp"   "examples/log-sink-tcp/dataflow.yml"

    echo ""
    echo "=== Service/Action examples (networked) ==="
    run_networked "service-example" "examples/service-example/dataflow.yml"
    run_networked "action-example"  "examples/action-example/dataflow.yml"

    echo ""
    echo "=== Service/Action examples (local) ==="
    run_local "local-service-example" "examples/service-example/dataflow.yml" 10
    run_local "local-action-example"  "examples/action-example/dataflow.yml" 15

    echo ""
    echo "=== Validated pipeline (deterministic source -> transform -> sink) ==="
    run_networked "validated-pipeline"       "examples/validated-pipeline/dataflow.yml" 30
    run_local "local-validated-pipeline" "examples/validated-pipeline/dataflow.yml" 15
fi

# ---------------------------------------------------------------------------
# Python examples
# ---------------------------------------------------------------------------

if [ "$RUN_PYTHON" = true ]; then
    echo ""
    echo "=== Module example (networked + local, Python) ==="
    run_networked "module-dataflow" "examples/module-dataflow/dataflow.yml" 30
    run_local "local-module-dataflow" "examples/module-dataflow/dataflow.yml" 10

    echo ""
    echo "=== Python examples (networked) ==="
    run_networked "python-dataflow"        "examples/python-dataflow/dataflow.yml" 30
    run_networked "python-async"           "examples/python-async/dataflow.yaml" 15
    run_networked "python-echo"            "examples/python-echo/dataflow.yml" 15
    run_networked "python-drain"           "examples/python-drain/dataflow.yaml" 15
    run_networked "python-log"             "examples/python-log/dataflow.yaml" 15
    run_networked "python-logging"         "examples/python-logging/dataflow.yml" 15
    run_networked "python-multiple-arrays" "examples/python-multiple-arrays/dataflow.yml" 15
    run_networked "python-concurrent-rw"   "examples/python-concurrent-rw/dataflow.yml" 15
    run_networked "log-aggregator"          "examples/log-aggregator/dataflow.yml" 15
    run_networked "typed-dataflow"          "examples/typed-dataflow/dataflow.yml" 15
    run_networked "streaming-example"       "examples/streaming-example/dataflow.yml" 15
    run_networked "python-recv-async"      "examples/python-recv-async/dataflow.yml" 30
    # Zero-copy send_output_raw -- the API backing dora's zero-copy guarantee.
    # contract_dataflow.yml self-checks the buffer-protocol safety contract and
    # exits nonzero on violation (validated by the run_local exit-code check).
    run_networked "python-zero-copy-send"          "examples/python-zero-copy-send/dataflow.yml" 30
    run_networked "python-zero-copy-send-contract" "examples/python-zero-copy-send/contract_dataflow.yml" 30

    echo ""
    echo "=== Python examples (local) ==="
    run_local "local-python-dataflow"        "examples/python-dataflow/dataflow.yml" 30
    run_local "local-python-async"           "examples/python-async/dataflow.yaml" 10
    run_local "local-python-echo"            "examples/python-echo/dataflow.yml" 10
    run_local "local-python-drain"           "examples/python-drain/dataflow.yaml" 10
    run_local "local-python-log"             "examples/python-log/dataflow.yaml" 10
    run_local "local-python-logging"         "examples/python-logging/dataflow.yml" 10
    run_local "local-python-multiple-arrays" "examples/python-multiple-arrays/dataflow.yml" 10
    run_local "local-python-concurrent-rw"   "examples/python-concurrent-rw/dataflow.yml" 10
    run_local "local-log-aggregator"          "examples/log-aggregator/dataflow.yml" 10
    run_local "local-typed-dataflow"          "examples/typed-dataflow/dataflow.yml" 10
    run_local "local-streaming-example"       "examples/streaming-example/dataflow.yml" 10
    run_local "local-python-recv-async"    "examples/python-recv-async/dataflow.yml" 15
    run_local "local-python-zero-copy-send"          "examples/python-zero-copy-send/dataflow.yml" 15
    run_local "local-python-zero-copy-send-contract" "examples/python-zero-copy-send/contract_dataflow.yml" 15

    echo ""
    echo "=== Queue/timeout regression tests (local, timing-sensitive) ==="
    run_local "local-queue-size-and-timeout"         "tests/queue_size_and_timeout_python/dataflow.yaml" 20
    run_local "local-queue-size-latest-data-python"  "tests/queue_size_latest_data_python/dataflow.yaml" 20

    echo ""
    echo "=== Memory-pool CPU transport (requires torch) ==="
    if python3 -c "import torch, tqdm" 2>/dev/null; then
        run_networked "memory-pool-cpu2cpu"       "examples/memory-pool/cpu2cpu.yml" 60
        run_local     "local-memory-pool-cpu2cpu" "examples/memory-pool/cpu2cpu.yml" 60
    else
        log_skip "memory-pool-cpu2cpu" "requires torch + tqdm (pip install torch tqdm)"
    fi
fi

# ---------------------------------------------------------------------------
# Queue/timeout regression with Rust receiver (requires both Rust + Python)
# ---------------------------------------------------------------------------

if [ "$RUN_RUST" = true ] && [ "$RUN_PYTHON" = true ]; then
    echo "Building queue_size_latest_data Rust receiver..."
    cargo build -p receive_data 2>&1 | tail -1

    run_local "local-queue-size-latest-data-rust" "tests/queue_size_latest_data_rust/dataflow.yaml" 20
fi

# ---------------------------------------------------------------------------
# Cross-language examples (Rust + Python, requires both)
# ---------------------------------------------------------------------------

if [ "$RUN_RUST" = true ] && [ "$RUN_PYTHON" = true ]; then
    echo "Building cross-language nodes..."
    cargo build \
        -p cross-language-rust-sender \
        -p cross-language-rust-receiver \
        2>&1 | tail -1

    echo ""
    echo "=== Cross-language examples (networked) ==="
    run_networked "cross-language-rust-to-python" "examples/cross-language/rust-to-python.yml" 30
    run_networked "cross-language-python-to-rust" "examples/cross-language/python-to-rust.yml" 30

    echo ""
    echo "=== Cross-language examples (local) ==="
    run_local "local-cross-language-rust-to-python" "examples/cross-language/rust-to-python.yml" 15
    run_local "local-cross-language-python-to-rust" "examples/cross-language/python-to-rust.yml" 15

    echo ""
    echo "=== MAVLink 2 bridge — Python variant ==="
    run_networked "mavlink2-bridge-python"       "examples/mavlink2-bridge/dataflow-python.yml" 30
    run_local     "local-mavlink2-bridge-python" "examples/mavlink2-bridge/dataflow-python.yml" 10
else
    log_skip "cross-language"          "requires both Rust and Python"
    log_skip "mavlink2-bridge-python"  "requires both Rust and Python"
fi

# MAVLink 2 bridge — Rust variant runs without Python at all.
if [ "$RUN_RUST" = true ]; then
    echo ""
    echo "=== MAVLink 2 bridge — Rust variant ==="
    run_networked "mavlink2-bridge-rust"       "examples/mavlink2-bridge/dataflow-rust.yml" 30
    run_local     "local-mavlink2-bridge-rust" "examples/mavlink2-bridge/dataflow-rust.yml" 10
fi

# ---------------------------------------------------------------------------
# Skipped examples
# ---------------------------------------------------------------------------

echo ""
echo "=== Skipped (special dependencies) ==="
log_skip "python-dataflow-dynamic" "webcam + opencv"
log_skip "python-operator-dataflow" "webcam + YOLOv5 + torch"
log_skip "cuda-benchmark" "CUDA toolkit"
log_skip "ros2-bridge" "ROS2"
log_skip "multiple-daemons" "multi-machine deploy"
log_skip "rust-dataflow-git" "external git clone"
log_skip "c-dataflow" "C compiler"
log_skip "c++-dataflow" "C++ compiler"
log_skip "c++-arrow-dataflow" "C++ + Arrow libs"
log_skip "cmake-dataflow" "CMake + C++"
log_skip "python-dataflow-builder" "no YAML (API-based)"
log_skip "dynamic-add-remove" "interactive dynamic topology CLI"
log_skip "dynamic-agent-tools" "interactive dynamic topology CLI"
log_skip "rust-dynamic-add-remove" "dynamic topology lifecycle (covered by node-lifecycle-e2e)"
log_skip "cxx-dynamic-add-remove" "C++/CMake + dynamic topology (covered by node-lifecycle-e2e)"
log_skip "mavlink2-bridge-cxx" "C++ + Arrow C++ libs (covered by cxx examples job)"
log_skip "cpu-affinity-probe" "Linux-only cpu_affinity (covered by example-smoke.rs + nightly)"
log_skip "error-propagation" "deliberate node failure demo (success == nonzero exit)"
log_skip "python-parquet-recorder" "webcam + opencv"
log_skip "python-yolo-detection" "webcam + YOLO + torch"
log_skip "mavlink2-bridge-sitl-mission" "external ArduPilot SITL on udp:14550"
log_skip "ros2-comparison" "ROS2 rclpy comparison (and dataflow.yml node paths are stale, see #issue)"

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

echo ""
echo "========================================"
echo "  PASS: $PASS  FAIL: $FAIL  SKIP: $SKIP"
echo "========================================"

if [ "${#FAILURES[@]}" -gt 0 ]; then
    echo ""
    echo "Failed examples:"
    for f in "${FAILURES[@]}"; do
        echo "  - $f"
    done
    exit 1
fi
