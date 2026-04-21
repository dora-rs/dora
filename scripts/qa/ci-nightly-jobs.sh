#!/usr/bin/env bash
# scripts/qa/ci-nightly-jobs.sh -- local driver for the 6 GHA-only nightly jobs.
#
# The GHA nightly workflow (.github/workflows/nightly.yml) has 11 test jobs.
# `cargo test -p dora-examples --test example-smoke` (run by qa-nightly's
# example-smoke step) covers 5 of them (smoke-suite + log-sinks +
# service-action + streaming + record-replay). This script covers the other 6:
#
#   - cluster-smoke             dora up + cluster status + start --detach + cluster down
#   - topic-and-top-smoke       dora top/trace/topic/self update against a zenoh-debug fixture
#   - cpu-affinity-smoke        Linux-only. sched_getaffinity regression (#252).
#   - redb-backend-smoke        coord restart reads daemon records back (#253).
#   - daemon-reconnect-smoke    Linux-only. SIGSTOP+watchdog+SIGCONT reconnect (#254).
#   - state-reconstruction-smoke Running -> Recovering on coord restart (#255, partial).
#
# Each job is a shell function mirroring the GHA step bodies. When GHA nightly
# changes, update the matching function here -- they must stay in lockstep so
# a green local run predicts a green CI run.
#
# See GitHub issue #1707 for design rationale.

set -euo pipefail

cd "$(dirname "$0")/../.."

# -----------------------------------------------------------------------------
# Platform + env setup
# -----------------------------------------------------------------------------

OS="$(uname -s)"
LINUX_ONLY=()
FAILED=()
SKIPPED=()

# Install dora CLI into a scratch root we own, so we don't clobber the user's
# ~/.cargo/bin/dora. Prepend to PATH for this script only.
CLI_ROOT="$(mktemp -d -t dora-qa-XXXXXX)"
trap 'rm -rf "$CLI_ROOT"' EXIT

echo
echo "=== install dora CLI to $CLI_ROOT (so ~/.cargo/bin/dora is not clobbered) ==="
cargo install --path binaries/cli --locked --root "$CLI_ROOT" --quiet
export PATH="$CLI_ROOT/bin:$PATH"
dora --version

# Port 6013 is the default coordinator WS port. If something is already
# listening, bail out with a clear message instead of racing.
if command -v lsof > /dev/null 2>&1 && lsof -iTCP:6013 -sTCP:LISTEN > /dev/null 2>&1; then
  echo "ERROR: port 6013 is already in use. Stop the running coordinator first:"
  echo "  dora destroy   # or: pkill -f 'dora (coordinator|daemon)'"
  exit 1
fi

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
  # Aggressive teardown between jobs. Some jobs leave background coord/daemon
  # processes; don't let them contaminate the next job.
  pkill -CONT -f "dora (daemon|coordinator)" 2>/dev/null || true
  pkill -f    "dora (daemon|coordinator)" 2>/dev/null || true
  sleep 1
}

# -----------------------------------------------------------------------------
# Job 1: cluster-smoke
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
  sleep 1
  if pgrep -fa 'dora (daemon|coordinator)' > /dev/null; then
    echo "ERROR: leftover dora processes after cluster down:"
    pgrep -fa 'dora (daemon|coordinator)'
    return 1
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

  # Teardown
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
  sleep 2
  dora daemon > /tmp/daemon1.log 2>&1 &
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

  pkill -TERM -f "dora coordinator" 2>/dev/null || true
  sleep 2

  if [ $failed -eq 0 ]; then
    # Session 2: read
    dora coordinator --store "redb:$STORE" > /tmp/coord2.log 2>&1 &
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
  sleep 2
  dora daemon > /tmp/daemon.log 2>&1 &
  local DAEMON_PID=$!
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
  sleep 2
  dora daemon > /tmp/sr-d1.log 2>&1 &
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

  pkill -TERM -f "dora coordinator" 2>/dev/null || true
  sleep 2

  if [ $failed -eq 0 ]; then
    # Session 2: restart coord, assert Recovering transition
    dora coordinator --store "redb:$STORE" > /tmp/sr-c2.log 2>&1 &
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

# -----------------------------------------------------------------------------
# Dispatch
# -----------------------------------------------------------------------------

run_job "cluster-smoke"             job_cluster_smoke
run_job "topic-and-top-smoke"       job_topic_and_top
run_job "cpu-affinity-smoke"        job_cpu_affinity
run_job "redb-backend-smoke"        job_redb_backend
run_job "daemon-reconnect-smoke"    job_daemon_reconnect
run_job "state-reconstruction-smoke" job_state_reconstruction

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
