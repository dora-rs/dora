#!/usr/bin/env bash
# scripts/qa/ci-nightly-jobs.sh -- local driver for the GHA nightly jobs.
#
# The GHA nightly workflow (.github/workflows/nightly.yml) has 18 test jobs
# post-#1716. `cargo test -p dora-examples --test example-smoke` (run by
# qa-nightly's example-smoke step) covers 4 of them (smoke-suite + log-sinks
# + service-action + streaming). This script covers the other 14, with
# platform-aware dispatch -- on macOS dev machines it runs the macOS subset,
# on Linux it runs the Linux subset, etc. (#1716).
#
#   Integration smokes (all platforms unless noted):
#   - record-replay             record + replay with build: directives INTACT.
#   - cluster-smoke             dora up + cluster status + start --detach + cluster down
#   - topic-and-top-smoke       dora top/trace/topic/self update against a zenoh-debug fixture
#   - cpu-affinity-smoke        Linux-only. sched_getaffinity regression (#252).
#   - redb-backend-smoke        coord restart reads daemon records back (#253).
#   - daemon-reconnect-smoke    Linux-only. SIGSTOP+watchdog+SIGCONT reconnect (#254).
#   - state-reconstruction-smoke Running -> Recovering on coord restart (#255, partial).
#
#   Moved from ci.yml in #1716 (now only run in nightly):
#   - test-cross-platform       Full test/check/build/cargo test on macOS/Windows.
#                               Linux dev: skipped (redundant with qa-test).
#   - examples                  Rust/C/C++/CMake example dataflows.
#                               Windows: Rust only. Linux: all (incl. cmake).
#                               macOS: all minus cmake.
#                               Arrow C++ skipped unless libarrow installed.
#   - cli-tests                 `dora new` templates + Python dynamic node + queue
#                               latency + error-event. C/C++ template: Linux only.
#                               Python portion needs uv + Python 3.12.
#   - bench-example             All platforms: cargo run --example benchmark --release.
#   - msrv                      All platforms: cargo-hack check --rust-version.
#                               Skipped if cargo-hack not installed.
#   - cross-check               Native target for the dev's OS. Full cross-matrix
#                               (mingw/musl/aarch64) is CI-only -- those need
#                               toolchains not typically present on dev machines.
#   - ros2-bridge               Linux + ROS2 Humble only. Skipped elsewhere.
#
# Each job is a shell function mirroring the GHA step bodies. When GHA nightly
# changes, update the matching function here -- they must stay in lockstep so
# a green local run predicts a green CI run.
#
# PROCESS SAFETY
# The script never runs a broad `pkill -f 'dora (daemon|coordinator)'` --
# that would terminate unrelated Dora processes the user happens to have
# running on other ports. Instead, cleanup is scoped two ways:
#   1. Explicit `$!` PIDs of dora processes this script started in the
#      background are tracked in MANAGED_PIDS and killed by PID.
#   2. For `dora up`-spawned children we don't directly `$!`, we match
#      only processes whose command line includes our scratch CLI path
#      ($CLI_ROOT/bin/dora, a unique mktemp dir per script run). Other
#      dora binaries on the machine never match this path.
#
# See GitHub issue #1707 for design rationale.

set -euo pipefail

cd "$(dirname "$0")/../.."

# -----------------------------------------------------------------------------
# Platform + env setup
# -----------------------------------------------------------------------------

OS="$(uname -s)"
FAILED=()
SKIPPED=()
MANAGED_PIDS=()

# -----------------------------------------------------------------------------
# Portable `timeout` shim
# -----------------------------------------------------------------------------
# GNU coreutils `timeout` is standard on Linux but not on default macOS.
# Homebrew's coreutils package installs it as `gtimeout` to avoid colliding
# with BSD tools. If neither is available, fall back to running unbounded
# with a loud warning so the user knows the time limit isn't enforced.
if ! command -v timeout > /dev/null 2>&1; then
  if command -v gtimeout > /dev/null 2>&1; then
    timeout() { gtimeout "$@"; }
  else
    echo
    echo "WARN: no 'timeout' or 'gtimeout' binary found."
    echo "      On macOS, install via: brew install coreutils"
    echo "      Jobs will run UNBOUNDED until they complete or you Ctrl-C."
    echo
    timeout() {
      # Silently drop the duration arg and run the command unbounded.
      shift
      "$@"
    }
  fi
fi

# Install dora CLI into a scratch root we own, so we don't clobber the user's
# ~/.cargo/bin/dora. Prepend to PATH for this script only. CLI_ROOT is also
# the unique pattern we use to identify our own processes (see PROCESS SAFETY
# in the header).
CLI_ROOT="$(mktemp -d -t dora-qa-XXXXXX)"
trap 'cleanup_all_managed; rm -rf "$CLI_ROOT"' EXIT

echo
echo "=== install dora CLI to $CLI_ROOT (so ~/.cargo/bin/dora is not clobbered) ==="
cargo install --path binaries/cli --locked --root "$CLI_ROOT" --quiet
export PATH="$CLI_ROOT/bin:$PATH"
dora --version

# Port 6013 is the default coordinator WS port. If something is already
# listening, bail out with a clear message instead of racing.
if command -v lsof > /dev/null 2>&1 && lsof -iTCP:6013 -sTCP:LISTEN > /dev/null 2>&1; then
  echo "ERROR: port 6013 is already in use. A dora coordinator is likely already"
  echo "running. Stop that coordinator with \`dora destroy\` before re-running."
  exit 1
fi

# -----------------------------------------------------------------------------
# Safe process management (replaces broad pkill)
# -----------------------------------------------------------------------------

# Track a PID we just started in the background (typically the $! of the
# most recent `dora ... &` invocation). Call immediately after launch.
track_pid() {
  MANAGED_PIDS+=("$1")
}

# Graceful kill for a specific PID. Safe on already-dead PIDs.
terminate_pid() {
  local pid="$1"
  # SIGCONT first in case it was SIGSTOPped (daemon-reconnect-smoke does this
  # deliberately); TERM would otherwise be queued but never delivered.
  kill -CONT "$pid" 2>/dev/null || true
  kill -TERM "$pid" 2>/dev/null || true
}

# Kill dora processes that were spawned by our scratch CLI but that we don't
# have an explicit PID for (e.g., coord/daemon started via `dora up`, which
# itself returns after forking them off). Matching on CLI_ROOT ensures we
# never touch dora binaries installed elsewhere -- $CLI_ROOT is a unique
# per-run mktemp path.
terminate_our_dora_children() {
  if ! command -v pgrep > /dev/null 2>&1; then
    return 0
  fi
  local pids
  pids=$(pgrep -f "$CLI_ROOT/bin/dora" 2>/dev/null || true)
  if [ -z "$pids" ]; then
    return 0
  fi
  local pid
  for pid in $pids; do
    # Don't target ourselves or the parent shell.
    if [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ]; then
      continue
    fi
    terminate_pid "$pid"
  done
}

# Full cleanup between jobs and at exit: kill explicit tracked PIDs, then
# sweep any leftover dora children that belong to our scratch CLI. Both
# paths are scoped to this script -- nothing else on the machine is touched.
cleanup_all_managed() {
  local pid
  if [ ${#MANAGED_PIDS[@]} -gt 0 ]; then
    for pid in "${MANAGED_PIDS[@]}"; do
      terminate_pid "$pid"
    done
  fi
  terminate_our_dora_children
  # Give TERM a moment; then KILL anything still alive that we started.
  sleep 1
  if [ ${#MANAGED_PIDS[@]} -gt 0 ]; then
    for pid in "${MANAGED_PIDS[@]}"; do
      kill -KILL "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    done
  fi
  if command -v pgrep > /dev/null 2>&1; then
    local remaining
    remaining=$(pgrep -f "$CLI_ROOT/bin/dora" 2>/dev/null || true)
    if [ -n "$remaining" ]; then
      local rpid
      for rpid in $remaining; do
        if [ "$rpid" = "$$" ] || [ "$rpid" = "$PPID" ]; then
          continue
        fi
        kill -KILL "$rpid" 2>/dev/null || true
      done
    fi
  fi
  MANAGED_PIDS=()
}

run_job() {
  local name="$1"
  local fn="$2"
  echo
  echo "============================================================"
  echo "=== $name ==="
  echo "============================================================"
  if "$fn"; then
    echo "  PASS: $name"
  else
    echo "  FAIL: $name"
    FAILED+=("$name")
  fi
  # Between-jobs cleanup: only PIDs we started + only dora procs spawned from
  # our scratch CLI. Unrelated dora processes on the machine are untouched.
  cleanup_all_managed
  sleep 1
}

# -----------------------------------------------------------------------------
# Job 1: record-replay (build: directives INTACT)
#
# Mirrors .github/workflows/nightly.yml `record-replay` job. The whole point
# of this job is to exercise `dora record` + `dora replay` against a dataflow
# whose descriptor has live `build: cargo build -p <node>` directives. That
# path has been a recurring source of /tmp cargo bugs (#1673/#1674 for
# record, #1691 for replay).
#
# example-smoke.rs's contract_record_replay_* test uses a fixture that
# deliberately STRIPS the build directives (see the comment at
# write_absolute_path_validated_pipeline_fixture), so it's blind to this
# class of bug. Keep this function in lockstep with the GHA step bodies.
# -----------------------------------------------------------------------------
job_record_replay() {
  cargo build --quiet \
    -p rust-dataflow-example-node \
    -p rust-dataflow-example-status-node \
    -p rust-dataflow-example-sink \
    -p dora-record-node \
    -p dora-replay-node

  # Write the .drec next to its dataflow (not at the workspace root) so
  # replay's working_dir is examples/rust-dataflow/ -- avoids picking up
  # $WORKSPACE/types/std/* as "user types" (build/mod.rs rejects those).
  local DREC=examples/rust-dataflow/run.drec
  rm -f "$DREC"

  # Bound each step with a hard timeout so a hung daemon/node doesn't
  # stall qa-nightly indefinitely. 300s matches the GHA
  # record-replay job (.github/workflows/nightly.yml:261, tuned in
  # #1705 for cold-cache rebuild variance). Replay is normally ~2s;
  # 120s is generous but still bounded.
  if ! timeout 300s dora record examples/rust-dataflow/dataflow.yml -o "$DREC"; then
    echo "ERROR: dora record failed or exceeded 300s"
    return 1
  fi
  if [ ! -s "$DREC" ]; then
    echo "ERROR: dora record did not produce a non-empty .drec"
    return 1
  fi
  echo "OK: recording size: $(wc -c < "$DREC") bytes"

  if ! timeout 120s dora replay "$DREC"; then
    echo "ERROR: dora replay failed or exceeded 120s"
    rm -f "$DREC"
    return 1
  fi
  rm -f "$DREC"
  echo "OK: record-replay round-trip with build: directives intact"
}

# -----------------------------------------------------------------------------
# Job 2: cluster-smoke
# -----------------------------------------------------------------------------
job_cluster_smoke() {
  cargo build --quiet \
    -p rust-dataflow-example-node \
    -p rust-dataflow-example-status-node \
    -p rust-dataflow-example-sink

  dora up
  sleep 2

  local out
  out=$(dora cluster status)
  echo "$out"
  echo "$out" | grep -q 'DAEMON ID' || { echo "ERROR: status output missing header"; return 1; }
  local daemon_lines
  daemon_lines=$(echo "$out" | grep -E '^[0-9a-f]{8}-' | wc -l | tr -d ' ')
  test "$daemon_lines" -ge 1 || { echo "ERROR: no daemons listed"; return 1; }
  echo "$out" | grep -q 'Active dataflows:' || { echo "ERROR: missing dataflow summary"; return 1; }

  dora start examples/rust-dataflow/dataflow.yml --detach
  local i
  for i in $(seq 1 30); do
    sleep 1
    if ! dora list 2>/dev/null | grep -q rust-dataflow; then
      echo "dataflow completed after ${i}s"
      break
    fi
    if [ "$i" -eq 30 ]; then
      echo "ERROR: dataflow did not finish within 30s"
      dora stop --name rust-dataflow 2>/dev/null || true
      return 1
    fi
  done

  dora cluster down
  sleep 1
  if dora cluster status 2>/dev/null; then
    echo "ERROR: cluster status succeeded after 'cluster down'"
    return 1
  fi
  # Leftover-process check: scoped to our scratch CLI path only.
  sleep 1
  if command -v pgrep > /dev/null 2>&1; then
    local leftover
    leftover=$(pgrep -f "$CLI_ROOT/bin/dora" 2>/dev/null || true)
    if [ -n "$leftover" ]; then
      echo "ERROR: leftover dora processes after cluster down (from our CLI):"
      pgrep -fa "$CLI_ROOT/bin/dora" || true
      return 1
    fi
  fi
}

# -----------------------------------------------------------------------------
# Job 2: topic-and-top-smoke
# -----------------------------------------------------------------------------
job_topic_and_top() {
  # Python venv with local dora-rs matching the daemon's message format.
  if ! command -v uv > /dev/null 2>&1; then
    echo "SKIP: uv not installed (needed for python source nodes in fixture)"
    SKIPPED+=("topic-and-top-smoke: uv missing")
    return 0
  fi

  local venv_dir
  venv_dir="$(mktemp -d -t dora-qa-venv-XXXXXX)"
  uv venv --seed -p 3.12 "$venv_dir" > /dev/null
  # shellcheck disable=SC1091
  source "$venv_dir/bin/activate"
  uv pip install --quiet pyarrow
  uv pip install --quiet -e apis/python/node

  dora up
  sleep 2

  local out
  # top --once with no dataflow must produce JSON array
  out=$(dora top --once || true)
  echo "$out"
  echo "$out" | grep -qE '^\[' || { echo "ERROR: top --once didn't produce JSON array"; return 1; }

  # trace list empty-state
  out=$(dora trace list)
  echo "$out" | grep -q "No traces captured yet." \
    || { echo "ERROR: trace list empty-state message missing"; return 1; }

  # trace view absent UUID
  out=$(dora trace view 00000000-0000-0000-0000-000000000000)
  echo "$out" | grep -q "No spans found for this trace." \
    || { echo "ERROR: trace view absent-UUID message missing"; return 1; }

  # trace view unresolvable prefix -> non-zero exit + "no trace found"
  if out=$(dora trace view definitely-not-a-trace 2>&1); then
    echo "ERROR: expected non-zero exit for unresolvable prefix"
    return 1
  fi
  echo "$out" | grep -q "no trace found matching prefix" \
    || { echo "ERROR: trace view missing prefix-error message"; return 1; }

  # self update --check-only read path
  out=$(timeout 30 dora self update --check-only 2>&1) || true
  echo "$out" | grep -q "Checking for updates" \
    || { echo "ERROR: --check-only didn't enter update-check path"; return 1; }
  if echo "$out" | grep -qE "latest version|update is available"; then
    :
  elif echo "$out" | grep -qE "rate limit|403|429"; then
    echo "WARN: GitHub API rate-limited -- skipping outcome assertion"
  else
    echo "ERROR: --check-only produced no recognized outcome message"
    return 1
  fi

  # Start debug-enabled dataflow (Python source nodes -> runs forever)
  dora start tests/fixtures/zenoh-debug-dataflow.yml --name tui-smoke --detach
  sleep 5
  dora list

  # topic list NDJSON
  out=$(dora topic list -d tui-smoke --format json)
  echo "$out" | python3 -c "
import json, sys
rows = [json.loads(line) for line in sys.stdin if line.strip()]
assert rows, 'no NDJSON rows produced'
assert any(r.get('node') == 'ticker' and r.get('name') == 'count' for r in rows), \
    f'ticker/count not in topic list: {rows}'
"

  # topic info --duration 3
  local info_out
  info_out=$(dora topic info -d tui-smoke ticker/count --duration 3 2>&1)
  echo "$info_out" | grep -q 'ticker/count' || { echo "ERROR: info output missing topic"; return 1; }
  local total
  total=$(echo "$info_out" | grep -oE 'Total messages: [0-9]+' | grep -oE '[0-9]+' || echo 0)
  if [ "$total" -lt 10 ]; then
    echo "ERROR: info received $total messages in 3s on a 10Hz fixture; expected >= 10"
    return 1
  fi

  # topic echo --count 5
  local echo_out
  echo_out=$(timeout 15 dora topic echo -d tui-smoke ticker/count --count 5 --format json 2>&1)
  local lines
  lines=$(echo "$echo_out" | grep -c '"name":"ticker/count"' || echo 0)
  if [ "$lines" -lt 5 ]; then
    echo "ERROR: echo --count 5 produced $lines frames; expected 5"
    return 1
  fi

  # topic hz --duration 3
  local hz_out
  hz_out=$(dora topic hz -d tui-smoke ticker/count --duration 3 2>&1)
  local samples
  samples=$(echo "$hz_out" | awk -F'\t' '/^ticker\/count/ { print $NF }')
  if [ -z "$samples" ] || [ "$samples" -lt 10 ]; then
    echo "ERROR: hz received ${samples:-0} samples in 3s on a 10Hz fixture; expected >= 10"
    return 1
  fi

  # topic pub --count 3
  local pub_out
  pub_out=$(dora topic pub -d tui-smoke ticker/count '42' --count 3 2>&1)
  echo "$pub_out" | grep -q 'Published message 3/3' \
    || { echo "ERROR: pub didn't emit all 3 messages"; return 1; }

  # Teardown (graceful; cleanup_all_managed in run_job will catch any leftover).
  dora stop --name tui-smoke --grace-duration 5s 2>/dev/null || true
  dora destroy 2>/dev/null || true
  deactivate 2>/dev/null || true
  rm -rf "$venv_dir"
}

# -----------------------------------------------------------------------------
# Job 3: cpu-affinity-smoke (Linux-only)
# -----------------------------------------------------------------------------
job_cpu_affinity() {
  if [ "$OS" != "Linux" ]; then
    echo "SKIP: cpu-affinity-smoke is Linux-only (uses sched_getaffinity)"
    SKIPPED+=("cpu-affinity-smoke: non-Linux ($OS)")
    return 0
  fi

  cargo build --quiet -p cpu-affinity-probe

  cd examples/cpu-affinity-probe
  local out
  out=$(dora run dataflow.yml --stop-after 3s 2>&1 || true)
  cd - > /dev/null
  echo "$out"

  local mask
  mask=$(echo "$out" | grep -oE 'AFFINITY_MASK:[^[:space:]]+' | head -1 | cut -d: -f2)
  if [ -z "$mask" ]; then
    echo "ERROR: probe did not print AFFINITY_MASK"
    return 1
  fi
  if [ "$mask" != "0,1" ]; then
    echo "ERROR: expected AFFINITY_MASK:0,1, got: $mask"
    return 1
  fi
  echo "OK: probe ran with expected affinity mask: $mask"
}

# -----------------------------------------------------------------------------
# Job 4: redb-backend-smoke
# -----------------------------------------------------------------------------
job_redb_backend() {
  cargo build --quiet -p rust-dataflow-example-node

  local STORE=/tmp/dora-redb-smoke.db
  rm -f "$STORE"

  # Inline dataflow fixture
  cat > examples/rust-dataflow/redb-smoke.yml <<'EOF'
nodes:
  - id: source
    path: ../../target/debug/rust-dataflow-example-node
    inputs:
      tick: dora/timer/millis/500
    outputs:
      - random
EOF

  # Session 1: write
  dora coordinator --store "redb:$STORE" > /tmp/coord1.log 2>&1 &
  local COORD1_PID=$!
  track_pid "$COORD1_PID"
  sleep 2
  dora daemon > /tmp/daemon1.log 2>&1 &
  track_pid "$!"
  sleep 2
  dora start examples/rust-dataflow/redb-smoke.yml --name redb-smoke --detach
  sleep 6

  local failed=0
  if [ ! -f "$STORE" ]; then
    echo "ERROR: redb store not created"
    failed=1
  elif ! strings "$STORE" | grep -q "redb-smoke"; then
    echo "ERROR: redb store does not contain the dataflow name"
    failed=1
  else
    echo "OK: redb store contains dataflow record"
  fi

  # Simulate coord crash by targeting only the coord we launched, by PID.
  kill -TERM "$COORD1_PID" 2>/dev/null || true
  sleep 2

  if [ $failed -eq 0 ]; then
    # Session 2: read
    dora coordinator --store "redb:$STORE" > /tmp/coord2.log 2>&1 &
    track_pid "$!"
    sleep 4
    echo "=== coord2 log ==="
    cat /tmp/coord2.log
    if ! grep -q "cleared.*stale daemon records" /tmp/coord2.log; then
      echo "ERROR: session 2 coord did not emit stale-daemon cleanup log"
      failed=1
    elif grep -qE "corrupt|panicked|ERROR" /tmp/coord2.log; then
      echo "ERROR: session 2 coord log contains corruption or panic markers"
      failed=1
    else
      echo "OK: redb store read cleanly on restart"
    fi
  fi

  rm -f examples/rust-dataflow/redb-smoke.yml
  return $failed
}

# -----------------------------------------------------------------------------
# Job 5: daemon-reconnect-smoke (Linux-only)
# -----------------------------------------------------------------------------
job_daemon_reconnect() {
  if [ "$OS" != "Linux" ]; then
    echo "SKIP: daemon-reconnect-smoke is Linux-only (relies on SIGSTOP/SIGCONT semantics)"
    SKIPPED+=("daemon-reconnect-smoke: non-Linux ($OS)")
    return 0
  fi

  dora coordinator > /tmp/coord.log 2>&1 &
  track_pid "$!"
  sleep 2
  dora daemon > /tmp/daemon.log 2>&1 &
  local DAEMON_PID=$!
  track_pid "$DAEMON_PID"
  sleep 3

  if ! grep -q "daemon.*reports.*running dataflow" /tmp/coord.log; then
    echo "ERROR: daemon did not register"
    cat /tmp/coord.log
    return 1
  fi
  echo "OK: initial daemon registration confirmed"

  kill -STOP "$DAEMON_PID"
  echo "daemon frozen at $(date +%s)"
  sleep 35
  if ! grep -q "Disconnecting daemons that failed watchdog" /tmp/coord.log; then
    echo "ERROR: coord did not log watchdog disconnect after 35s pause"
    cat /tmp/coord.log
    # Resume the daemon so cleanup_all_managed's TERM is deliverable.
    kill -CONT "$DAEMON_PID" 2>/dev/null || true
    return 1
  fi
  echo "OK: coord timed out the frozen daemon"

  kill -CONT "$DAEMON_PID"
  echo "daemon resumed at $(date +%s)"
  sleep 30

  if ! grep -q "daemon disconnected from coordinator" /tmp/daemon.log; then
    echo "ERROR: daemon did not detect disconnect post-resume"
    cat /tmp/daemon.log
    return 1
  fi

  local report_count
  report_count=$(grep -c "daemon.*reports.*running dataflow" /tmp/coord.log || true)
  if [ "$report_count" -lt 2 ]; then
    echo "ERROR: expected >= 2 daemon reports (initial + post-reconnect), got $report_count"
    cat /tmp/coord.log
    return 1
  fi
  echo "OK: daemon reconnected and re-registered ($report_count reports)"
}

# -----------------------------------------------------------------------------
# Job 6: state-reconstruction-smoke
# -----------------------------------------------------------------------------
job_state_reconstruction() {
  cargo build --quiet -p rust-dataflow-example-node

  local STORE=/tmp/state-reconstruct.db
  rm -f "$STORE"

  cat > examples/rust-dataflow/long-running.yml <<'EOF'
nodes:
  - id: source
    path: ../../target/debug/rust-dataflow-example-node
    inputs:
      tick: dora/timer/millis/500
    outputs:
      - random
EOF

  # Session 1: run long-lived dataflow, kill coord
  dora coordinator --store "redb:$STORE" > /tmp/sr-c1.log 2>&1 &
  local COORD1_PID=$!
  track_pid "$COORD1_PID"
  sleep 2
  dora daemon > /tmp/sr-d1.log 2>&1 &
  track_pid "$!"
  sleep 2
  dora start examples/rust-dataflow/long-running.yml --name sr-test --detach
  sleep 4

  local failed=0
  if ! strings "$STORE" | grep -q "sr-test"; then
    echo "ERROR: dataflow name not in store"
    failed=1
  else
    echo "OK: Running dataflow persisted to store"
  fi

  # Simulate coord crash: target only the coord we started, by PID.
  kill -TERM "$COORD1_PID" 2>/dev/null || true
  sleep 2

  if [ $failed -eq 0 ]; then
    # Session 2: restart coord, assert Recovering transition
    dora coordinator --store "redb:$STORE" > /tmp/sr-c2.log 2>&1 &
    track_pid "$!"
    sleep 4
    echo "=== sr-c2 log ==="
    cat /tmp/sr-c2.log
    if ! grep -q "coordinator restarted:.*-> Recovering" /tmp/sr-c2.log; then
      echo "ERROR: coord did not mark previously-Running dataflow as Recovering"
      failed=1
    else
      echo "OK: coord hydrated Running record as Recovering"
    fi
  fi

  rm -f examples/rust-dataflow/long-running.yml
  return $failed
}

# =============================================================================
# Jobs moved from ci.yml to nightly.yml in #1716 -- these are the other half of
# "local qa-nightly = CI nightly parity". Each runs the subset of coverage that
# applies to the dev's OS, matching what the GHA nightly would test on that
# same platform.
# =============================================================================

# -----------------------------------------------------------------------------
# Job 8: test-cross-platform (non-Linux only; redundant with qa-test on Linux)
# Mirrors nightly.yml `test-cross-platform` (macOS + Windows).
# -----------------------------------------------------------------------------
job_test_cross_platform() {
  if [ "$OS" = "Linux" ]; then
    echo "SKIP: test-cross-platform is redundant with qa-test on Linux (PR CI already covers ubuntu-latest)"
    SKIPPED+=("test-cross-platform: redundant on Linux")
    return 0
  fi
  cargo check --all
  cargo check --examples
  cargo build --all \
    --exclude dora-node-api-python \
    --exclude dora-operator-api-python \
    --exclude dora-ros2-bridge-python
  cargo test --all \
    --exclude dora-node-api-python \
    --exclude dora-operator-api-python \
    --exclude dora-ros2-bridge-python \
    --exclude dora-cli-api-python \
    --exclude dora-examples
}

# -----------------------------------------------------------------------------
# Job 9: examples (platform-aware)
# Mirrors nightly.yml `examples`.
# -----------------------------------------------------------------------------
job_examples() {
  cargo build --quiet --examples
  cargo build --quiet -p dora-cli
  # NOTE: $CLI_ROOT/bin/dora is already on PATH (set by the script
  # preamble), so the daemon's which::which("dora") will find it. Do
  # NOT prepend target/debug here -- a naive `export PATH=...` leaks
  # past this function and subsequent jobs' spawned `dora up` children
  # would end up at target/debug/dora, which `terminate_our_dora_children`
  # (scoped to $CLI_ROOT/bin/dora via pgrep) can't see -> leaked procs.

  timeout 600s cargo run --example rust-dataflow
  timeout 600s cargo run --example rust-dataflow-git
  timeout 600s cargo run --example multiple-daemons

  case "$OS" in
    MINGW*|CYGWIN*|MSYS*|Windows_NT)
      echo "SKIP c/cxx dataflow examples: Windows local dev typically lacks the toolchain CI sets up"
      SKIPPED+=("examples: C/C++ on Windows")
      ;;
    *)
      # C / C++ examples need a working C/C++ toolchain reachable by cargo.
      timeout 600s cargo run --example c-dataflow
      timeout 600s cargo run --example cxx-dataflow

      # Arrow C++ library: detect via pkg-config or brew. Skip if neither.
      if command -v pkg-config > /dev/null 2>&1 && pkg-config --exists arrow 2>/dev/null; then
        timeout 600s cargo run --example cxx-arrow-dataflow
      elif [ "$OS" = "Darwin" ] && command -v brew > /dev/null 2>&1 && brew list apache-arrow > /dev/null 2>&1; then
        timeout 600s cargo run --example cxx-arrow-dataflow
      else
        echo "SKIP cxx-arrow-dataflow: arrow C++ library not found (install libarrow-dev or 'brew install apache-arrow')"
        SKIPPED+=("examples: cxx-arrow-dataflow (arrow C++ missing)")
      fi
      ;;
  esac

  if [ "$OS" = "Linux" ]; then
    timeout 600s cargo run --example cmake-dataflow
  else
    echo "SKIP cmake-dataflow: GHA runs this on Linux only"
    SKIPPED+=("examples: cmake-dataflow (non-Linux)")
  fi
}

# -----------------------------------------------------------------------------
# Job 10: cli-tests (platform-aware)
# Mirrors nightly.yml `cli-tests` (was the `cli` job in old ci.yml).
# -----------------------------------------------------------------------------
job_cli_tests() {
  # CLI already installed at $CLI_ROOT/bin/dora by this script's preamble.
  # cargo install --path binaries/cli --locked is redundant here; skip it
  # to save ~45 min per local run.

  # Rust template project: all platforms
  local tmpd
  tmpd=$(mktemp -d -t dora-tpl-XXXXXX)
  (
    cd "$tmpd"
    dora new test_rust_project --internal-create-with-path-dependencies
    cd test_rust_project
    cargo build --all
    timeout 120s dora run dataflow.yml --stop-after 10s
  )
  rm -rf "$tmpd"

  # Dynamic Rust Dataflow
  dora up
  sleep 2
  dora build examples/rust-dataflow/dataflow_dynamic.yml
  dora start examples/rust-dataflow/dataflow_dynamic.yml --name ci-rust-dynamic --detach
  timeout 60s cargo run -p rust-dataflow-example-sink-dynamic
  sleep 5
  dora stop --name ci-rust-dynamic --grace-duration 5s 2>/dev/null || true
  dora destroy 2>/dev/null || true

  # Error Event Example
  dora build examples/error-propagation/dataflow.yml
  local err_out
  err_out=$(timeout 60s dora run examples/error-propagation/dataflow.yml 2>&1 || true)
  echo "$err_out" | grep -q "Received error from node" \
    || { echo "ERROR: error-propagation did not receive NodeFailed event"; return 1; }

  # Python tests require uv + Python 3.12
  if ! command -v uv > /dev/null 2>&1; then
    echo "SKIP Python cli-tests: uv not installed"
    SKIPPED+=("cli-tests: Python portion (uv missing)")
  else
    local venv_dir
    venv_dir=$(mktemp -d -t dora-cli-venv-XXXXXX)
    uv venv --seed -p 3.12 "$venv_dir" > /dev/null
    # shellcheck disable=SC1091
    source "$venv_dir/bin/activate"
    uv pip install --quiet pyarrow "ruff>=0.9" pytest
    uv pip install --quiet -e apis/python/node

    # Python template: dora new + dora build + ruff + pytest + dora run
    local tmpd2
    tmpd2=$(mktemp -d -t dora-pytpl-XXXXXX)
    (
      cd "$tmpd2"
      dora new test_python_project --lang python --internal-create-with-path-dependencies
      cd test_python_project
      dora build dataflow.yml --uv
      uv run ruff check .
      uv run pytest
      export OPERATING_MODE=SAVE
      dora build dataflow.yml --uv
      timeout 120s dora run dataflow.yml --uv --stop-after 10s
    )
    rm -rf "$tmpd2"

    # Python Node example
    dora build examples/python-dataflow/dataflow.yml --uv
    timeout 60s dora run examples/python-dataflow/dataflow.yml --uv --stop-after 10s

    # Python Dynamic Node example
    # dora-hub deps pull dora-rs from PyPI, overriding local build;
    # re-install local version to avoid version mismatch (matches GHA).
    dora build examples/python-dataflow/dataflow_dynamic.yml --uv
    uv pip install --quiet -e apis/python/node
    dora up
    sleep 2
    dora start examples/python-dataflow/dataflow_dynamic.yml --name ci-python-dynamic --detach --uv
    timeout 60s uv run opencv-plot || true
    sleep 10
    dora stop --name ci-python-dynamic --grace-duration 30s 2>/dev/null || true
    dora destroy 2>/dev/null || true

    # Python Operator example
    # dora-hub deps pull dora-rs from PyPI; re-install local version.
    dora build examples/python-operator-dataflow/dataflow.yml --uv
    uv pip install --quiet -e apis/python/node
    timeout 120s dora run examples/python-operator-dataflow/dataflow.yml --uv --stop-after 20s

    # Python Multiple Arrays
    dora build examples/python-multiple-arrays/dataflow.yml --uv
    timeout 120s dora run examples/python-multiple-arrays/dataflow.yml --uv --stop-after 30s

    # Python Async example (5-min run; skipped on Windows per GHA)
    case "$OS" in
      MINGW*|CYGWIN*|MSYS*|Windows_NT)
        echo "SKIP Python Async example: GHA runs this on non-Windows only"
        SKIPPED+=("cli-tests: python-async on Windows")
        ;;
      *)
        timeout 360s dora run examples/python-async/dataflow.yaml --uv --stop-after 5m
        ;;
    esac

    # Python Drain example
    OTEL_SDK_DISABLED=true timeout 120s dora run examples/python-drain/dataflow.yaml --uv

    # Python Dataflow Builder API (YAML generation contract test)
    # `simple_example.py` itself can't run here -- its build: directives
    # pip-install hub packages (opencv-video-capture, dora-yolo, opencv-plot)
    # which pull PyPI `dora-rs` 0.5.0 and clobber the local workspace version.
    # Exercise the builder API itself via the hub-package-free YAML
    # generation contract test (matches GHA's #1654 approach).
    timeout 60s uv run --with PyYAML python examples/python-dataflow-builder/test_builder_api.py

    # Python Queue Latency Test
    timeout 120s dora run tests/queue_size_latest_data_python/dataflow.yaml --uv

    # Python Queue Latency + Timeout Test
    timeout 120s dora run tests/queue_size_and_timeout_python/dataflow.yaml --uv

    # Rust Queue Latency Test
    dora build tests/queue_size_latest_data_rust/dataflow.yaml --uv
    timeout 120s dora run tests/queue_size_latest_data_rust/dataflow.yaml --uv

    deactivate 2>/dev/null || true
    rm -rf "$venv_dir"
  fi

  # C / C++ template tests: Linux only per GHA (`if: runner.os == 'Linux'`)
  if [ "$OS" = "Linux" ]; then
    local tmpd3
    tmpd3=$(mktemp -d -t dora-ctpl-XXXXXX)
    (
      cd "$tmpd3"
      dora new test_c_project --lang c --internal-create-with-path-dependencies
      cd test_c_project
      cmake -B build
      cmake --build build
      cmake --install build
      timeout 120s dora run dataflow.yml --stop-after 10s
    )
    rm -rf "$tmpd3"

    local tmpd4
    tmpd4=$(mktemp -d -t dora-cxxtpl-XXXXXX)
    (
      cd "$tmpd4"
      dora new test_cxx_project --lang cxx --internal-create-with-path-dependencies
      cd test_cxx_project
      cmake -B build
      cmake --build build
      cmake --install build
      timeout 120s dora run dataflow.yml --stop-after 10s
    )
    rm -rf "$tmpd4"
  else
    echo "SKIP C/C++ template tests: GHA runs these on Linux only"
    SKIPPED+=("cli-tests: C/C++ template (non-Linux)")
  fi
}

# -----------------------------------------------------------------------------
# Job 11: bench-example (all platforms)
# Mirrors nightly.yml `bench-example`.
# -----------------------------------------------------------------------------
job_bench_example() {
  timeout 2700s cargo run --example benchmark --release
}

# -----------------------------------------------------------------------------
# Job 12: msrv (all platforms)
# Mirrors nightly.yml `msrv`.
# -----------------------------------------------------------------------------
job_msrv() {
  if ! command -v cargo-hack > /dev/null 2>&1; then
    echo "SKIP msrv: cargo-hack not installed (cargo install cargo-hack)"
    SKIPPED+=("msrv: cargo-hack missing")
    return 0
  fi
  cargo hack check --rust-version --workspace --ignore-private --locked
}

# -----------------------------------------------------------------------------
# Job 13: cross-check (native target for dev's OS)
# Mirrors nightly.yml `cross-check`, but ONLY the native target -- the
# cross-compile matrix (aarch64, musl, mingw) needs `cross` or mingw
# toolchains that aren't always present locally. CI does the full matrix.
# -----------------------------------------------------------------------------
job_cross_check() {
  case "$OS" in
    Linux)
      cargo check --target x86_64-unknown-linux-gnu --all \
        --exclude dora-node-api-python \
        --exclude dora-operator-api-python \
        --exclude dora-ros2-bridge-python \
        --exclude dora-cli-api-python
      ;;
    Darwin)
      # uname -m is arm64 on Apple Silicon, x86_64 on Intel.
      local arch
      arch="$(uname -m)"
      if [ "$arch" = "arm64" ]; then
        cargo check --target aarch64-apple-darwin --all \
          --exclude dora-node-api-python \
          --exclude dora-operator-api-python \
          --exclude dora-ros2-bridge-python \
          --exclude dora-cli-api-python
      else
        cargo check --target x86_64-apple-darwin --all \
          --exclude dora-node-api-python \
          --exclude dora-operator-api-python \
          --exclude dora-ros2-bridge-python \
          --exclude dora-cli-api-python
      fi
      ;;
    *)
      echo "SKIP cross-check: native target for $OS not in the local matrix"
      SKIPPED+=("cross-check: $OS not dispatched")
      return 0
      ;;
  esac
}

# -----------------------------------------------------------------------------
# Job 14: ros2-bridge (Linux + ROS2 Humble)
# Mirrors nightly.yml `ros2-bridge`.
# -----------------------------------------------------------------------------
job_ros2_bridge() {
  if [ "$OS" != "Linux" ]; then
    echo "SKIP ros2-bridge: GHA runs this on ubuntu-22.04 only"
    SKIPPED+=("ros2-bridge: non-Linux ($OS)")
    return 0
  fi
  if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "SKIP ros2-bridge: /opt/ros/humble/setup.bash not found"
    echo "  install with: see https://docs.ros.org/en/humble/Installation.html"
    SKIPPED+=("ros2-bridge: ROS2 Humble not installed")
    return 0
  fi
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  timeout 600s cargo test -p dora-ros2-bridge-python
  timeout 1800s env QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example rust-ros2-dataflow
}

# -----------------------------------------------------------------------------
# Dispatch
# -----------------------------------------------------------------------------

run_job "record-replay"             job_record_replay
run_job "cluster-smoke"             job_cluster_smoke
run_job "topic-and-top-smoke"       job_topic_and_top
run_job "cpu-affinity-smoke"        job_cpu_affinity
run_job "redb-backend-smoke"        job_redb_backend
run_job "daemon-reconnect-smoke"    job_daemon_reconnect
run_job "state-reconstruction-smoke" job_state_reconstruction
run_job "test-cross-platform"       job_test_cross_platform
run_job "examples"                  job_examples
run_job "cli-tests"                 job_cli_tests
run_job "bench-example"             job_bench_example
run_job "msrv"                      job_msrv
run_job "cross-check"               job_cross_check
run_job "ros2-bridge"               job_ros2_bridge

echo
echo "============================================================"
if [ ${#SKIPPED[@]} -gt 0 ]; then
  echo "SKIPPED:"
  printf '  - %s\n' "${SKIPPED[@]}"
fi
if [ ${#FAILED[@]} -eq 0 ]; then
  echo "All CI-nightly jobs passed."
  exit 0
else
  echo "FAILED:"
  printf '  - %s\n' "${FAILED[@]}"
  exit 1
fi
