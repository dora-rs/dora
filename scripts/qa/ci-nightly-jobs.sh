#!/usr/bin/env bash
# scripts/qa/ci-nightly-jobs.sh -- local driver for the GHA nightly jobs.
#
# The GHA nightly workflow (.github/workflows/nightly.yml) has 21 test jobs
# (post-#1716, plus cluster-record-replay from #2013 and kani-proofs).
# `cargo test -p dora-examples --test example-smoke` (run by qa-nightly's
# example-smoke step)
# covers 4 of them (smoke-suite + log-sinks + service-action + streaming).
# This script covers the other 17, with
# platform-aware dispatch -- on macOS dev machines it runs the macOS subset,
# on Linux it runs the Linux subset, etc. (#1716).
#
#   Integration smokes (all platforms unless noted):
#   - record-replay             record + replay with build: directives INTACT.
#   - cluster-smoke             dora up + cluster status + start --detach + cluster down
#   - cluster-e2e               Linux-only. Real-sshd end-to-end:
#                               dora cluster up/status/down against 3 SSH-spawned
#                               daemons on a loopback sshd. Hard-fails if
#                               openssh-server is not installed.
#   - cluster-record-replay     Linux-only. Stitches cluster-e2e + record-replay:
#                               record a dataflow spread across the 3 SSH daemons,
#                               then replay it locally and validate the replayed
#                               state against the seed(42) baseline (#2013).
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
#   - kani-proofs               All platforms: formal-verification proof
#                               harnesses (scripts/qa/kani.sh). Skipped if
#                               Kani not installed (make qa-kani-install).
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
SELECTED_JOBS=("$@")
SELECTED_JOB_COUNT=$#
CLI_ROOT=""
CLI_INSTALLED=0

# Scan a dataflow's `out/<UUID>/log_*.jsonl` for per-node failure
# markers. `dora run --stop-after Ns` exits 0 if the daemon ran for N
# seconds regardless of whether any node actually succeeded inside —
# so callers need an explicit assertion on top to catch silent
# crashes. Without this, the cli-tests phase reports PASS even when
# `listener_1` failed to build and `talker_1` never received an event
# (#1863).
#
# Patterns chosen because `level:error` in dora's JSONL is noisy
# (zenoh emits "Unable to publish transport event: session closed"
# on every clean teardown). The patterns below are unambiguous node
# failures that don't appear in the happy path.
assert_clean_dataflow_run() {
  local outdir="${1:-out}"
  local fail_re='panicked at|failed to build node|ModuleNotFoundError:|Traceback \(most recent|^Caused by:|"level":"stderr".*"msg":"Error:'
  local issues=()

  if [ ! -d "$outdir" ]; then
    # No output at all means `dora run` produced nothing — the
    # caller's own exit-code check already covered that case.
    return 0
  fi

  while IFS= read -r logfile; do
    local node
    node=$(basename "$logfile" .jsonl | sed 's/^log_//')
    # Internal dora bookkeeping nodes (record/replay etc.) — skip.
    case "$node" in __dora_*) continue ;; esac
    if grep -qE "$fail_re" "$logfile" 2>/dev/null; then
      issues+=("$node (see $logfile)")
    fi
  done < <(find "$outdir" -name 'log_*.jsonl' -type f 2>/dev/null)

  if [ ${#issues[@]} -gt 0 ]; then
    echo "ERROR: cli-tests assertion: dora run exit was clean but these nodes had failures:" >&2
    printf '  - %s\n' "${issues[@]}" >&2
    return 1
  fi
  return 0
}

known_job() {
  case "$1" in
    record-replay|cluster-smoke|cluster-e2e|cluster-record-replay|topic-and-top-smoke|cpu-affinity-smoke|redb-backend-smoke|daemon-reconnect-smoke|state-reconstruction-smoke|test-cross-platform|examples|cli-tests|bench-example|msrv|cross-check|ros2-bridge|kani-proofs)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

usage() {
  cat <<'EOF'
Usage: scripts/qa/ci-nightly-jobs.sh [job ...]

Run local equivalents for the long-running GitHub Actions nightly jobs.
With no job names, runs every supported job for the current platform.

Supported jobs:
  record-replay
  cluster-smoke
  cluster-e2e
  cluster-record-replay
  topic-and-top-smoke
  cpu-affinity-smoke
  redb-backend-smoke
  daemon-reconnect-smoke
  state-reconstruction-smoke
  test-cross-platform
  examples
  cli-tests
  bench-example
  msrv
  cross-check
  ros2-bridge
  kani-proofs
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

if [ "$SELECTED_JOB_COUNT" -gt 0 ]; then
  for selected in "${SELECTED_JOBS[@]}"; do
    if ! known_job "$selected"; then
      echo "ERROR: unknown nightly job: $selected" >&2
      usage >&2
      exit 2
    fi
  done
fi

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

trap 'cleanup_all_managed; if [ -n "$CLI_ROOT" ]; then rm -rf "$CLI_ROOT"; fi' EXIT

ensure_cli_installed() {
  if [ "$CLI_INSTALLED" -eq 1 ]; then
    return 0
  fi

  # Port 6013 is the default coordinator WS port. Jobs that spawn dora services
  # need it free; pure cargo jobs should not be blocked by this preflight.
  if command -v lsof > /dev/null 2>&1 && lsof -iTCP:6013 -sTCP:LISTEN > /dev/null 2>&1; then
    echo "ERROR: port 6013 is already in use. A dora coordinator is likely already"
    echo "running. Stop that coordinator with \`dora destroy\` before re-running."
    exit 1
  fi

  # Install dora CLI into a scratch root we own, so we don't clobber the user's
  # ~/.cargo/bin/dora. Prepend to PATH for this script only. CLI_ROOT is also
  # the unique pattern we use to identify our own processes (see PROCESS SAFETY
  # in the header).
  CLI_ROOT="$(mktemp -d -t dora-qa-XXXXXX)"
  # CI fast-path: if `DORA_QA_CLI_BIN` points to a prebuilt dora binary
  # (e.g. the `build-cli` artifact in .github/workflows/nightly.yml), copy
  # it into CLI_ROOT instead of rebuilding. Local dev typically leaves this
  # unset and pays the cargo install cost. The CLI_ROOT mktemp dir is still
  # unique per run, so /proc/<pid>/exe-based process matching keeps working.
  if [ -n "${DORA_QA_CLI_BIN:-}" ] && [ -x "$DORA_QA_CLI_BIN" ]; then
    mkdir -p "$CLI_ROOT/bin"
    cp "$DORA_QA_CLI_BIN" "$CLI_ROOT/bin/dora"
    chmod +x "$CLI_ROOT/bin/dora"
    echo
    echo "=== reuse prebuilt dora from \$DORA_QA_CLI_BIN ($DORA_QA_CLI_BIN -> $CLI_ROOT/bin/dora) ==="
  else
    echo
    echo "=== install dora CLI to $CLI_ROOT (so ~/.cargo/bin/dora is not clobbered) ==="
    cargo install --path binaries/cli --locked --root "$CLI_ROOT" --quiet
  fi
  export PATH="$CLI_ROOT/bin:$PATH"
  dora --version
  CLI_INSTALLED=1
}

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

# List PIDs that we spawned via our scratch CLI. Combines two methods:
#   1. `pgrep -f "$CLI_ROOT/bin/dora"` — matches argv (full cmdline). Catches
#      processes invoked by absolute path, which is what `dora up` does for
#      its coord/daemon children.
#   2. `/proc/<pid>/exe` symlink resolution — matches the actual binary
#      regardless of argv[0]. Catches PATH-resolved children whose argv[0]
#      is just "dora" (e.g. `nohup dora daemon ...` over SSH in cluster-e2e
#      with `environment="PATH=$CLI_ROOT/bin:..."` in authorized_keys).
# Either alone misses one of these spawn patterns; together they catch both
# without false positives, since CLI_ROOT is a unique per-run mktemp path.
# Skips $$ and $PPID. /proc is Linux-only; macOS uses pgrep only.
find_our_dora_pids() {
  if [ -z "$CLI_ROOT" ]; then
    return 0
  fi
  local target="$CLI_ROOT/bin/dora"
  local pids=""
  if command -v pgrep > /dev/null 2>&1; then
    pids=$(pgrep -f "$target" 2>/dev/null || true)
  fi
  if [ -d /proc ]; then
    local entry pid exe
    for entry in /proc/[0-9]*; do
      pid="${entry##/proc/}"
      if [ "$pid" = "$$" ] || [ "$pid" = "$PPID" ]; then
        continue
      fi
      exe=$(readlink "/proc/$pid/exe" 2>/dev/null || true)
      if [ "$exe" = "$target" ]; then
        pids="$pids $pid"
      fi
    done
  fi
  # Dedup + drop blanks.
  echo "$pids" | tr ' ' '\n' | sort -u | grep -v '^$' || true
}

# Kill dora processes that were spawned by our scratch CLI but that we don't
# have an explicit PID for (e.g., coord/daemon started via `dora up`, or
# remote daemons spawned over SSH by cluster-e2e). Matching on CLI_ROOT
# ensures we never touch dora binaries installed elsewhere -- $CLI_ROOT is
# a unique per-run mktemp path.
terminate_our_dora_children() {
  local pid
  for pid in $(find_our_dora_pids); do
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
  local rpid
  for rpid in $(find_our_dora_pids); do
    if [ "$rpid" = "$$" ] || [ "$rpid" = "$PPID" ]; then
      continue
    fi
    kill -KILL "$rpid" 2>/dev/null || true
  done
  MANAGED_PIDS=()
}

run_job() {
  local name="$1"
  local fn="$2"
  if [ "$SELECTED_JOB_COUNT" -gt 0 ]; then
    local selected found=0
    for selected in "${SELECTED_JOBS[@]}"; do
      if [ "$selected" = "$name" ]; then
        found=1
        break
      fi
    done
    if [ "$found" -ne 1 ]; then
      return 0
    fi
  fi

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
  ensure_cli_installed

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
  ensure_cli_installed

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
# Shared scaffolding: bring up a loopback SSH cluster
#
# Stands up a real sshd on a random loopback port with a throwaway key, then
# starts it and writes a 3-machine cluster.yml (m1/m2/m3, all on localhost,
# distinct ssh + daemon ports, shared zenoh_peer). Used by both cluster-e2e
# and cluster-record-replay so the SSH/key/sshd boilerplate lives in one place.
#
# Linux-only; callers must gate on $OS before calling. Hard-fails (return 1)
# if openssh-server is missing. On success, sets these globals for the caller:
#   WORK        scratch tempdir (caller removes with `rm -rf "$WORK"`)
#   SSHD_PID    sshd pid (caller stops with `terminate_pid "$SSHD_PID"`)
#   COORD_PORT  coordinator WS port to export as DORA_COORDINATOR_PORT
# and leaves the SSH wrapper at "$WORK/bin/ssh" and the descriptor at
# "$WORK/cluster.yml". On failure it removes its own tempdir before returning.
#
# Arg 1 (optional): a label folded into the mktemp name for easier debugging.
# -----------------------------------------------------------------------------
setup_loopback_ssh_cluster() {
  local label="${1:-cluster}"

  local SSHD_BIN
  if [ -x /usr/sbin/sshd ]; then
    SSHD_BIN=/usr/sbin/sshd
  elif command -v sshd > /dev/null 2>&1; then
    SSHD_BIN=$(command -v sshd)
  else
    echo "ERROR: openssh-server not installed. Install with:"
    echo "  sudo apt-get install -y openssh-server"
    return 1
  fi

  ensure_cli_installed

  WORK=$(mktemp -d -t "dora-${label}-XXXXXX")

  # Pick 6 free loopback ports in one shot: 1 sshd, 1 coordinator, 3 daemons,
  # 1 inter-daemon Zenoh rendezvous (the shared peer endpoint plumbed through
  # `cluster.zenoh_peer` -> `dora daemon --zenoh-peer` in the cluster-zenoh-peer
  # PR; without it, daemons rely on multicast scouting which doesn't work on
  # loopback in nested containers, and the spread-across-machines dataflow
  # below hangs). bind+release leaves a small race window but is the standard
  # approach and is fine for a single-runner integration test.
  local PORTS
  PORTS=$(python3 - <<'PY'
import socket
ss = [socket.socket() for _ in range(6)]
for s in ss:
    s.bind(('127.0.0.1', 0))
print(*(s.getsockname()[1] for s in ss))
for s in ss:
    s.close()
PY
)
  # shellcheck disable=SC2086
  set -- $PORTS
  local SSHD_PORT=$1 D1=$3 D2=$4 D3=$5 ZENOH_PORT=$6
  COORD_PORT=$2

  # SSH key material. The wrapper script below forces ssh to use this key
  # and ignore the user's real ~/.ssh — we never touch the user's identity.
  # NOTE on error handling: `set -e` is disabled inside functions invoked via
  # `if "$fn"; then` (run_job's pattern), so failures do NOT abort
  # automatically — every command on a critical path needs an explicit check.
  # We do so for ssh-keygen and the sshd-startup wait below. Pure local-FS
  # operations (mkdir/chmod on $WORK) are left unchecked: $WORK is a fresh
  # mktemp dir we own, so they only fail under disk pressure, which surfaces
  # later anyway.
  mkdir -p "$WORK/.ssh"
  chmod 700 "$WORK/.ssh"
  if ! ssh-keygen -t ed25519 -f "$WORK/.ssh/id_ed25519" -N "" -q; then
    echo "ERROR: failed to generate test identity key"
    rm -rf "$WORK"
    return 1
  fi
  if ! ssh-keygen -t ed25519 -f "$WORK/host_key" -N "" -q; then
    echo "ERROR: failed to generate sshd host key"
    rm -rf "$WORK"
    return 1
  fi

  # Prefix the pubkey with an `environment=` option so PATH=<scratch CLI>
  # is set for sessions that authenticate with this key. The remote command
  # built by `dora cluster up` is `dora daemon ...` — without this, the
  # SSH session's default PATH wouldn't include $CLI_ROOT/bin and the
  # daemon would silently fail to spawn. Requires `PermitUserEnvironment`
  # in sshd_config below.
  local REMOTE_PATH="$CLI_ROOT/bin:/usr/local/bin:/usr/bin:/bin"
  printf 'environment="PATH=%s" ' "$REMOTE_PATH" > "$WORK/.ssh/authorized_keys"
  cat "$WORK/.ssh/id_ed25519.pub" >> "$WORK/.ssh/authorized_keys"
  chmod 600 "$WORK/.ssh/id_ed25519" "$WORK/.ssh/authorized_keys" "$WORK/host_key"

  # PATH-shadow ssh wrapper. The ssh client reads the default IdentityFile
  # from the user's passwd home (~/.ssh/id_*), NOT from $HOME — so setting
  # HOME=$WORK does not redirect the key lookup. `dora cluster` invokes ssh
  # without `-i`, so the only way to force our test key without modifying
  # the user's real ~/.ssh is to wrap ssh on PATH. Same for scp (unused
  # here, but kept in lockstep for any future path that calls it).
  mkdir -p "$WORK/bin"
  local REAL_SSH REAL_SCP
  REAL_SSH=$(command -v ssh)
  REAL_SCP=$(command -v scp || echo "")
  cat > "$WORK/bin/ssh" <<EOF
#!/bin/sh
# Test wrapper: force our key + isolate from the user's SSH config.
# -F /dev/null skips ~/.ssh/config (a local \`Host localhost\` rule could
# otherwise rewrite HostName or inject ProxyCommand). UserKnownHostsFile
# + GlobalKnownHostsFile keep all host-key state inside \$WORK.
exec $REAL_SSH -F /dev/null \\
     -i "$WORK/.ssh/id_ed25519" \\
     -o IdentitiesOnly=yes \\
     -o UserKnownHostsFile="$WORK/.ssh/known_hosts" \\
     -o GlobalKnownHostsFile=/dev/null \\
     "\$@"
EOF
  chmod +x "$WORK/bin/ssh"
  if [ -n "$REAL_SCP" ]; then
    cat > "$WORK/bin/scp" <<EOF
#!/bin/sh
exec $REAL_SCP -F /dev/null \\
     -i "$WORK/.ssh/id_ed25519" \\
     -o IdentitiesOnly=yes \\
     -o UserKnownHostsFile="$WORK/.ssh/known_hosts" \\
     -o GlobalKnownHostsFile=/dev/null \\
     "\$@"
EOF
    chmod +x "$WORK/bin/scp"
  fi

  # Minimal sshd_config, loopback only. StrictModes=no because $WORK isn't
  # root-owned. UsePAM=no so we don't need /etc/pam.d/sshd. PidFile in $WORK
  # avoids needing /var/run write access.
  cat > "$WORK/sshd_config" <<EOF
Port $SSHD_PORT
ListenAddress 127.0.0.1
HostKey $WORK/host_key
PidFile $WORK/sshd.pid
PubkeyAuthentication yes
PasswordAuthentication no
KbdInteractiveAuthentication no
AuthorizedKeysFile $WORK/.ssh/authorized_keys
PermitUserEnvironment yes
StrictModes no
UsePAM no
PrintMotd no
EOF

  "$SSHD_BIN" -D -f "$WORK/sshd_config" -e &
  SSHD_PID=$!
  track_pid "$SSHD_PID"

  # Wait for sshd to accept connections (max 5s). bash's /dev/tcp avoids a
  # netcat dependency.
  local i ok=0
  for i in $(seq 1 50); do
    if (exec 3<>/dev/tcp/127.0.0.1/"$SSHD_PORT") 2>/dev/null; then
      exec 3<&-
      ok=1
      break
    fi
    sleep 0.1
  done
  if [ "$ok" -ne 1 ]; then
    echo "ERROR: sshd did not start on 127.0.0.1:$SSHD_PORT within 5s"
    rm -rf "$WORK"
    return 1
  fi
  echo "OK: sshd up on 127.0.0.1:$SSHD_PORT"

  # cluster.yml: 3 machines all on localhost, distinct ssh + daemon ports,
  # shared zenoh_peer so cross-daemon discovery works without multicast
  # (the spread-across-m1/m2/m3 dataflows below depend on this).
  local SSH_USER
  SSH_USER=$(id -un)
  cat > "$WORK/cluster.yml" <<EOF
coordinator:
  addr: 127.0.0.1
  port: $COORD_PORT
zenoh_peer: tcp/127.0.0.1:$ZENOH_PORT
machines:
  - id: m1
    host: localhost
    user: $SSH_USER
    port: $SSHD_PORT
    daemon_port: $D1
  - id: m2
    host: localhost
    user: $SSH_USER
    port: $SSHD_PORT
    daemon_port: $D2
  - id: m3
    host: localhost
    user: $SSH_USER
    port: $SSHD_PORT
    daemon_port: $D3
EOF

  return 0
}

# Bring the cluster up via SSH and assert all 3 daemons connected. Shared by
# cluster-e2e and cluster-record-replay. Must run inside the per-job subshell
# (where PATH points at "$WORK/bin" and DORA_COORDINATOR_PORT is exported);
# returns non-zero on any check so the caller can `|| exit 1`.
cluster_up_and_verify() {
  echo "=== dora cluster up ==="
  local up_out
  if ! up_out=$(dora cluster up "$WORK/cluster.yml" 2>&1); then
    echo "$up_out"
    echo "ERROR: dora cluster up failed"
    return 1
  fi
  echo "$up_out"
  if ! echo "$up_out" | grep -q "Cluster is up: coordinator + 3 daemon(s)"; then
    echo "ERROR: 'cluster up' did not report 3 daemons up"
    return 1
  fi

  echo "=== dora cluster status ==="
  local status_out
  status_out=$(dora cluster status 2>&1)
  echo "$status_out"
  if ! echo "$status_out" | grep -q 'DAEMON ID'; then
    echo "ERROR: status output missing header"
    return 1
  fi
  # Named daemons show as `<machine-id>-<uuid>`. Count rows for m1/m2/m3.
  local rows
  rows=$(echo "$status_out" | grep -cE '^m[1-3]-' || true)
  if [ "$rows" -lt 3 ]; then
    echo "ERROR: expected 3 daemons (m1/m2/m3) in status, got $rows"
    return 1
  fi
  echo "OK: 3 daemons connected via SSH"
  return 0
}

# Tear the cluster down and assert it's gone with no leftover processes from
# our scratch CLI. Shared by cluster-e2e and cluster-record-replay. Must run
# inside the per-job subshell; returns non-zero on any check.
assert_cluster_torn_down() {
  echo "=== dora cluster down ==="
  if ! dora cluster down; then
    echo "ERROR: dora cluster down returned non-zero"
    return 1
  fi
  sleep 1
  if dora cluster status 2>/dev/null; then
    echo "ERROR: cluster status succeeded after 'cluster down'"
    return 1
  fi

  # Leftover-process check: scoped to our scratch CLI binary. Uses both
  # pgrep -f (argv match) and /proc/<pid>/exe (binary match) — the latter
  # is essential here because SSH-spawned daemons have argv[0]="dora", not
  # "$CLI_ROOT/bin/dora", and a pgrep-only check would miss them silently.
  sleep 1
  local leftover
  leftover=$(find_our_dora_pids | tr '\n' ' ')
  if [ -n "${leftover// /}" ]; then
    echo "ERROR: leftover dora processes after cluster down (from our CLI): $leftover"
    if command -v pgrep > /dev/null 2>&1; then
      pgrep -fa "$CLI_ROOT/bin/dora" 2>/dev/null || true
    fi
    return 1
  fi
  return 0
}

# Poll `dora list` until the given dataflow (by name or UUID) reaches Finished.
# Returns 0 on Finished, 1 on Failed or 30s timeout. Shared by cluster-e2e and
# cluster-record-replay; must run inside the per-job subshell (talks to the
# cluster coordinator on DORA_COORDINATOR_PORT). Reaching Finished doubles as a
# cross-daemon liveness check: downstream nodes only exit once their
# inter-daemon inputs close, so a broken Zenoh data plane leaves the dataflow
# Running forever and this poll times out.
poll_dataflow_finished() {
  local id="$1"
  echo "=== poll dataflow $id for Finished (max 30s) ==="
  local i list_out status=""
  for i in $(seq 1 60); do
    list_out=$(dora list 2>/dev/null || true)
    status=$(echo "$list_out" | grep "$id" | awk '{print $3}' | head -1)
    case "$status" in
      Finished)
        echo "OK: dataflow $id reached Finished (cross-daemon data plane verified)"
        return 0
        ;;
      Failed)
        echo "$list_out"
        echo "ERROR: dataflow $id reached Failed status"
        return 1
        ;;
    esac
    sleep 0.5
  done
  echo "$list_out"
  echo "ERROR: dataflow $id did not reach Finished within 30s (status: ${status:-<not in list>})."
  echo "       This usually means the inter-daemon Zenoh data plane isn't carrying"
  echo "       messages -- check that cluster.yml's zenoh_peer is reachable and that"
  echo "       the daemons started with --zenoh-peer."
  return 1
}

# -----------------------------------------------------------------------------
# Job 3: cluster-e2e
#
# End-to-end test of the `dora cluster` SSH lifecycle. cluster-smoke (above)
# uses `dora up` to launch a local coordinator+daemon and exercises only the
# post-up commands (status/down) — it does NOT cover the SSH path of
# `cluster up`, which is the most failure-prone part of the feature.
#
# This job stands up a real sshd on a random loopback port, then runs
# `dora cluster up` against a cluster.yml with 3 machines all pointing at
# localhost (via the `port` and `daemon_port` fields added by the cluster-ssh-port
# and cluster-daemon-port PRs). Validates that:
#   1. ssh-based daemon spawn works (3 daemons reach the coordinator)
#   2. `cluster status` lists all 3
#   3. `cluster down` tears everything down with no leftover processes
#
# Linux-only: macOS sshd has different defaults and locked-down config paths
# that don't map cleanly to this fixture, and the only Windows path would be
# OpenSSH-server-for-Windows which isn't worth the maintenance. The script
# does NOT install openssh-server itself — it hard-fails with an install
# hint if sshd is missing (per the agreed policy in the PR plan). GHA
# installs openssh-server in the workflow before invoking this script.
# -----------------------------------------------------------------------------
job_cluster_e2e() {
  case "$OS" in
    Linux) ;;
    *)
      echo "SKIP: cluster-e2e is Linux-only ($OS)"
      SKIPPED+=("cluster-e2e: $OS not supported")
      return 0
      ;;
  esac

  # Bring up the loopback SSH cluster (sshd + keys + cluster.yml). Sets
  # $WORK, $SSHD_PID, $COORD_PORT; hard-fails if openssh-server is missing.
  if ! setup_loopback_ssh_cluster cluster-e2e; then
    return 1
  fi

  # The test body runs in a subshell so PATH and DORA_COORDINATOR_PORT
  # exports stay scoped to this job — this script runs subsequent jobs
  # (record-replay, topic-and-top-smoke, etc.) in the same shell, and
  # an exported coord port would misroute their CLI traffic. The
  # subshell inherits CLI_ROOT, $WORK, $SSHD_PID, etc., but env mutations
  # don't propagate back. Failure paths use `exit 1` (which exits the
  # subshell); the outer `terminate_pid + rm -rf` cleanup runs regardless
  # of pass/fail.
  local test_rc=0
  (
    # Prepend $WORK/bin so dora's `Command::new("ssh")` picks up the wrapper.
    # Export DORA_COORDINATOR_PORT so every subsequent dora CLI call picks
    # up our random port via CoordinatorOptions' env hook (binaries/cli/src/
    # common.rs) — no need to pass --coordinator-port to each command.
    export PATH="$WORK/bin:$PATH"
    export DORA_COORDINATOR_PORT="$COORD_PORT"

    cluster_up_and_verify || exit 1

    # === Distributed dataflow run on the SSH-spawned cluster ===
    # Build the rust-dataflow example nodes and run a dataflow spread
    # across m1/m2/m3 via `_unstable_deploy.machine`. With the shared
    # `zenoh_peer` set in cluster.yml above, daemons find each other via
    # the explicit rendezvous endpoint (one binds, the rest connect) —
    # cross-daemon messages flow over the inter-daemon Zenoh data plane
    # rather than the within-daemon shared-memory shortcut, exercising
    # the actual distributed code path even though every daemon is on the
    # same loopback host.
    echo "=== build dataflow nodes ==="
    if ! cargo build --quiet \
         -p rust-dataflow-example-node \
         -p rust-dataflow-example-status-node \
         -p rust-dataflow-example-sink; then
      echo "ERROR: cargo build of dataflow nodes failed"
      exit 1
    fi

    local REPO_ROOT
    REPO_ROOT="$(pwd)"
    cat > "$WORK/dataflow.yml" <<EOF
nodes:
  - id: rust-node
    _unstable_deploy:
      machine: m1
    path: $REPO_ROOT/target/debug/rust-dataflow-example-node
    inputs:
      tick: dora/timer/millis/10
    outputs:
      - random
    output_types:
      random: std/core/v1/UInt64
  - id: rust-status-node
    _unstable_deploy:
      machine: m2
    path: $REPO_ROOT/target/debug/rust-dataflow-example-status-node
    inputs:
      tick: dora/timer/millis/100
      random: rust-node/random
    input_types:
      random: std/core/v1/UInt64
    outputs:
      - status
    output_types:
      status: std/core/v1/String
  - id: rust-sink
    _unstable_deploy:
      machine: m3
    path: $REPO_ROOT/target/debug/rust-dataflow-example-sink
    inputs:
      message: rust-status-node/status
    input_types:
      message: std/core/v1/String
EOF

    echo "=== dora start (distributed across m1/m2/m3) ==="
    if ! dora start "$WORK/dataflow.yml" --detach --name cluster-e2e 2>&1; then
      echo "ERROR: dora start failed"
      exit 1
    fi

    # Poll until the dataflow reaches Running state (max 15s). We don't use
    # a fixed sleep because cold-start on the SSH-spawned daemons + cross-
    # daemon Zenoh handshake adds variable overhead. Using `Running` as the
    # signal also means we then call `cluster restart` against an actively
    # running dataflow — RestartByName only matches running_dataflows, not
    # archived ones (binaries/coordinator/src/lib.rs::restart_dataflow).
    echo "=== wait for cluster-e2e to reach Running ==="
    local i list_out
    for i in $(seq 1 30); do
      list_out=$(dora list 2>/dev/null || true)
      if echo "$list_out" | grep cluster-e2e | grep -q Running; then
        break
      fi
      sleep 0.5
    done
    if ! echo "$list_out" | grep cluster-e2e | grep -q Running; then
      echo "$list_out"
      echo "ERROR: dataflow cluster-e2e did not reach Running within 15s"
      exit 1
    fi
    echo "OK: dataflow reached Running"
    echo "$list_out"

    # `dora cluster restart` against a real running dataflow. Two positional
    # args: cluster.yml + dataflow name. Sends RestartByName to the
    # coordinator, which stops the old instance and starts a new one with
    # the same descriptor — exercises the full restart control path
    # (coordinator -> daemons -> nodes -> coordinator) and uniquely
    # validates that SSH-spawned daemons can be addressed by name across
    # subsequent control requests, not just the initial `cluster up`.
    echo "=== dora cluster restart cluster.yml cluster-e2e ==="
    local restart_out
    if ! restart_out=$(dora cluster restart "$WORK/cluster.yml" cluster-e2e 2>&1); then
      echo "$restart_out"
      echo "ERROR: dora cluster restart failed"
      exit 1
    fi
    echo "$restart_out"
    # CLI prints "dataflow restarted: <old_uuid> -> <new_uuid>" on success
    # (binaries/cli/src/command/cluster/restart.rs).
    local new_uuid
    new_uuid=$(echo "$restart_out" | sed -nE 's/.*dataflow restarted: [^ ]+ -> ([^ ]+).*/\1/p')
    if [ -z "$new_uuid" ]; then
      echo "ERROR: couldn't parse new UUID from restart output"
      exit 1
    fi

    # Poll the restarted instance for Finished. This is the assertion that
    # proves the cross-daemon Zenoh data plane actually carries messages:
    # rust-status-node on m2 only exits when its `random` input (from
    # rust-node on m1) closes, and rust-sink on m3 only exits when its
    # `message` input (from rust-status-node on m2) closes — both closures
    # propagate over inter-daemon Zenoh.
    poll_dataflow_finished "$new_uuid" || exit 1

    # Stop any still-running instance(s) so cluster down has a clean slate.
    # `dora stop --name` matches by name; a non-zero exit usually means the
    # dataflow already finished — either way the cluster is clean after.
    echo "=== dora stop --name cluster-e2e ==="
    dora stop --name cluster-e2e 2>&1 || echo "(dora stop returned non-zero; dataflow may have already finished)"
    sleep 1

    assert_cluster_torn_down || exit 1
  ) || test_rc=$?

  # Cleanup happens regardless of pass/fail. Sshd is also in MANAGED_PIDS
  # so cleanup_all_managed would catch it between jobs anyway, but
  # terminating explicitly lets us remove the per-job tempdir while sshd
  # is definitely down.
  terminate_pid "$SSHD_PID"
  wait "$SSHD_PID" 2>/dev/null || true
  rm -rf "$WORK"
  if [ "$test_rc" -eq 0 ]; then
    echo "OK: cluster-e2e completed cleanly"
  fi
  return $test_rc
}

# -----------------------------------------------------------------------------
# Job 3b: cluster-record-replay
#
# Stitches the two existing nightly ingredients -- `cluster-e2e` (multi-daemon
# SSH cluster) and `record-replay` (record + replay round-trip) -- into one
# end-to-end flow (issue #2013, "B1"):
#
#   1. bring up a 3-daemon SSH cluster (shared setup_loopback_ssh_cluster)
#   2. start the rust-dataflow example spread across m1/m2/m3, with a
#      `__dora_record__` node deployed on m3 capturing both upstream outputs
#      (rust-node/random from m1, rust-status-node/status from m2) -- so the
#      recording itself flows over the inter-daemon Zenoh data plane
#   3. let the dataflow finish (the source self-terminates after 100 ticks)
#   4. tear the cluster down
#   5. replay the recording locally (`dora replay`, a single-daemon `dora run`)
#   6. validate the *replayed state*: the replayed sink output must reproduce
#      the deterministic fastrand::seed(42) random-value sequence committed in
#      tests/sample-inputs/expected-outputs-rust-status-node.jsonl.
#
# The descriptor embedded in the .drec (DORA_RECORD_DESCRIPTOR) is the
# deploy-free `plain.yml`, because local `dora run`/`dora replay` reject
# `_unstable_deploy`; the cluster-started descriptor is that same plain.yml
# with `_unstable_deploy.machine` injected per node. The record node is
# generated by the real `dora record --output-yaml` so the record-node env
# contract stays authoritative rather than hand-duplicated here.
#
# Linux-only for the same reasons as cluster-e2e (loopback sshd). Hard-fails
# if openssh-server is missing.
# -----------------------------------------------------------------------------
job_cluster_record_replay() {
  case "$OS" in
    Linux) ;;
    *)
      echo "SKIP: cluster-record-replay is Linux-only ($OS)"
      SKIPPED+=("cluster-record-replay: $OS not supported")
      return 0
      ;;
  esac

  ensure_cli_installed

  # Pre-build every binary the cluster run and the local replay need, so
  # neither `dora start` (cluster) nor `dora replay` (local) has to build
  # anything at run time. Absolute target/debug paths in the descriptors
  # below depend on these existing.
  if ! cargo build --quiet \
       -p rust-dataflow-example-node \
       -p rust-dataflow-example-status-node \
       -p rust-dataflow-example-sink \
       -p dora-record-node \
       -p dora-replay-node; then
    echo "ERROR: cargo build of dataflow + record/replay nodes failed"
    return 1
  fi

  if ! setup_loopback_ssh_cluster cluster-record-replay; then
    return 1
  fi

  local DREC="$WORK/cluster-run.drec"
  local test_rc=0
  (
    export PATH="$WORK/bin:$PATH"
    export DORA_COORDINATOR_PORT="$COORD_PORT"

    cluster_up_and_verify || exit 1

    local REPO_ROOT
    REPO_ROOT="$(pwd)"

    # plain.yml: the deploy-free, build-free, absolute-path descriptor. This
    # is what gets embedded in the .drec and replayed locally, so it must NOT
    # carry `_unstable_deploy` (local `dora run` rejects it) or `build:`
    # directives (the local replay runs from a scratch dir with no Cargo.toml
    # reachable -- absolute prebuilt paths sidestep that).
    cat > "$WORK/plain.yml" <<EOF
nodes:
  - id: rust-node
    path: $REPO_ROOT/target/debug/rust-dataflow-example-node
    inputs:
      tick: dora/timer/millis/10
    outputs:
      - random
    output_types:
      random: std/core/v1/UInt64
  - id: rust-status-node
    path: $REPO_ROOT/target/debug/rust-dataflow-example-status-node
    inputs:
      tick: dora/timer/millis/100
      random: rust-node/random
    input_types:
      random: std/core/v1/UInt64
    outputs:
      - status
    output_types:
      status: std/core/v1/String
  - id: rust-sink
    path: $REPO_ROOT/target/debug/rust-dataflow-example-sink
    inputs:
      message: rust-status-node/status
    input_types:
      message: std/core/v1/String
EOF

    # Inject the record node with the real CLI (keeps the record-node env
    # contract authoritative). Writes the .drec to an absolute loopback path
    # so whichever daemon hosts the record node can write it and the later
    # local replay can read it back.
    echo "=== dora record --output-yaml (inject __dora_record__) ==="
    if ! dora record --output-yaml "$WORK/recorded.yml" -o "$DREC" "$WORK/plain.yml" 2>&1; then
      echo "ERROR: dora record --output-yaml failed"
      exit 1
    fi

    # Spread the 4 nodes across m1/m2/m3 by inserting `_unstable_deploy.machine`
    # right after each top-level `- id: <name>` line. serde_yaml emits those
    # lines verbatim at column 0 with node properties at 2-space indent, so the
    # 2-/4-space inserted block lands at the correct sibling level. Putting the
    # record node on m3 while its inputs originate on m1 (random) and m2
    # (status) forces the recording across the inter-daemon Zenoh data plane.
    awk '
      /^- id: rust-node$/        { print; print "  _unstable_deploy:"; print "    machine: m1"; next }
      /^- id: rust-status-node$/ { print; print "  _unstable_deploy:"; print "    machine: m2"; next }
      /^- id: rust-sink$/        { print; print "  _unstable_deploy:"; print "    machine: m3"; next }
      /^- id: __dora_record__$/  { print; print "  _unstable_deploy:"; print "    machine: m3"; next }
      { print }
    ' "$WORK/recorded.yml" > "$WORK/cluster.dataflow.yml"

    # Guard against serde_yaml emission drift: if the `- id:` line shape ever
    # changes, the awk above silently injects nothing and every node defaults
    # to one machine -- which still reaches Finished, quietly defeating the
    # cross-daemon point of this job. Assert all 4 deploy blocks were inserted.
    local injected
    injected=$(grep -c '^  _unstable_deploy:$' "$WORK/cluster.dataflow.yml")
    if [ "$injected" -ne 4 ]; then
      echo "ERROR: expected to inject 4 _unstable_deploy blocks, injected $injected"
      echo "       (the record CLI's YAML emission format may have changed)"
      exit 1
    fi

    echo "=== dora start (record across m1/m2/m3) ==="
    if ! dora start "$WORK/cluster.dataflow.yml" --detach --name cluster-record 2>&1; then
      echo "ERROR: dora start failed"
      exit 1
    fi

    # The source node sends 100 ticks at 10ms (~1s) then exits; the record
    # node exits when its inputs close. If the inter-daemon data plane were
    # broken, the record node on m3 would never see the m1/m2 outputs close
    # and this poll would time out.
    poll_dataflow_finished cluster-record || exit 1

    echo "=== dora stop --name cluster-record ==="
    dora stop --name cluster-record 2>&1 || echo "(dora stop returned non-zero; dataflow may have already finished)"
    sleep 1

    assert_cluster_torn_down || exit 1

    if [ ! -s "$DREC" ]; then
      echo "ERROR: recording is empty -- the record node captured nothing"
      exit 1
    fi
    echo "OK: recording size: $(wc -c < "$DREC") bytes"
  ) || test_rc=$?

  # sshd teardown happens regardless of pass/fail.
  terminate_pid "$SSHD_PID"
  wait "$SSHD_PID" 2>/dev/null || true

  if [ "$test_rc" -ne 0 ]; then
    rm -rf "$WORK"
    return "$test_rc"
  fi

  # === Replay the cluster recording locally and validate the replayed state ===
  # `dora replay` runs a local single-daemon `dora run` from the deploy-free
  # descriptor stored in the .drec. The recorded source nodes (rust-node and
  # rust-status-node) are swapped for replay nodes; rust-sink reprocesses the
  # replayed status strings. Invoked from the repo root so the replay-node
  # binary resolves under target/debug; the run's working_dir is the .drec's
  # parent ($WORK), so node logs land in $WORK/out/.
  #
  # Replay at the recorded (real-time) speed -- NOT `--speed 0`. The exact
  # baseline match below requires lossless delivery into rust-sink, but
  # dora inputs default to `queue_size: 10` with the `DropOldest` overflow
  # policy. `--speed 0` blasts all ~100 status outputs near-instantly, so the
  # sink's size-10 queue overflows and silently drops the events it can't keep
  # up with -- a flaky, scheduling-dependent ~80-of-100 sequence that diverges
  # mid-stream (#2089). Real-time pacing (~10ms between outputs) lets the sink
  # drain each message before the next arrives, which is exactly why the
  # non-cluster `record-replay` job (also default speed) is stable.
  echo "=== dora replay $DREC (local single-daemon run, real-time speed) ==="
  rm -rf "$WORK/out"
  if ! timeout 120s dora replay "$DREC"; then
    echo "ERROR: dora replay failed or exceeded 120s"
    rm -rf "$WORK"
    return 1
  fi

  # Guard against a silent node crash: `dora replay` can exit 0 while a node
  # actually failed mid-run (same class of gap assert_clean_dataflow_run was
  # written for in the cli-tests job).
  if ! assert_clean_dataflow_run "$WORK/out"; then
    rm -rf "$WORK"
    return 1
  fi

  local sink_log
  sink_log=$(find "$WORK/out" -name 'log_rust-sink.jsonl' -type f 2>/dev/null | head -1)
  if [ -z "$sink_log" ]; then
    echo "ERROR: no rust-sink log produced by replay (expected $WORK/out/<uuid>/log_rust-sink.jsonl)"
    rm -rf "$WORK"
    return 1
  fi

  # Validate that the replayed random-value sequence exactly reproduces the
  # committed seed(42) baseline. At real-time replay speed the sink keeps up
  # with every recorded output (ordered, no queue overflow), so the replayed
  # sink must see the full baseline sequence in order -- any drop, reorder, or
  # value change is a genuine record/replay regression, not expected noise.
  echo "=== validate replayed state against seed(42) baseline ==="
  local baseline="tests/sample-inputs/expected-outputs-rust-status-node.jsonl"
  if ! python3 - "$sink_log" "$baseline" <<'PY'; then
import re
import sys

sink_log, baseline_path = sys.argv[1], sys.argv[2]


def values(path):
    out = []
    with open(path) as f:
        for line in f:
            out.extend(re.findall(r"random value (0x[0-9a-f]+)", line))
    return out


replayed = values(sink_log)
baseline = values(baseline_path)

if not replayed:
    print("ERROR: replay produced no random values in the sink log")
    sys.exit(1)
if replayed != baseline:
    print(
        f"ERROR: replayed sequence ({len(replayed)} values) does not match the "
        f"seed(42) baseline ({len(baseline)} values)"
    )
    for i, (r, b) in enumerate(zip(replayed, baseline)):
        if r != b:
            print(f"  first divergence at index {i}: replayed={r} baseline={b}")
            break
    else:
        print("  sequences share a common prefix but differ in length")
    sys.exit(1)

print(f"OK: replayed {len(replayed)} random values exactly reproduce the seed(42) baseline")
PY
    rm -rf "$WORK"
    return 1
  fi

  rm -rf "$WORK"
  echo "OK: cluster-record-replay completed cleanly"
}

# -----------------------------------------------------------------------------
# Job 4: topic-and-top-smoke
# -----------------------------------------------------------------------------
job_topic_and_top() {
  # Python venv with local dora-rs matching the daemon's message format.
  if ! command -v uv > /dev/null 2>&1; then
    echo "SKIP: uv not installed (needed for python source nodes in fixture)"
    SKIPPED+=("topic-and-top-smoke: uv missing")
    return 0
  fi
  ensure_cli_installed

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
  ensure_cli_installed

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
  ensure_cli_installed

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
  ensure_cli_installed

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
  ensure_cli_installed

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
  ensure_cli_installed

  cargo build --quiet --examples
  cargo build --quiet -p dora-cli
  # NOTE: $CLI_ROOT/bin/dora is already on PATH (set by the script
  # ensure_cli_installed), so the daemon's which::which("dora") will find it. Do
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
  ensure_cli_installed

  # CLI already installed at $CLI_ROOT/bin/dora by ensure_cli_installed.
  # cargo install --path binaries/cli --locked is redundant here; skip it
  # to save ~45 min per local run.

  # Rust template project: all platforms. Generated at repo root so
  # the CLI template's relative path dep `dora-node-api = { path =
  # "../../apis/rust/node" }` resolves to the actual workspace
  # (#1862). Matches `.github/workflows/nightly.yml::cli-tests`.
  # Pre-clean in case a prior interrupted run left state behind.
  rm -rf test_rust_project
  (
    dora new test_rust_project --internal-create-with-path-dependencies
    cd test_rust_project
    cargo build --all
    timeout 120s dora run dataflow.yml --stop-after 10s
    # Assert no per-node failures hid behind dora-run's 10s timer (#1863).
    assert_clean_dataflow_run "out"
  )
  rm -rf test_rust_project

  # Dynamic Rust Dataflow
  dora up
  sleep 2
  dora build examples/rust-dataflow/dataflow_dynamic.yml
  dora start examples/rust-dataflow/dataflow_dynamic.yml --name ci-rust-dynamic --detach
  # The dynamic sink blocks on recv until its input closes. The upstream
  # status-node runs an infinite timer and never exits, so the input only closes
  # when the dataflow is stopped. Let the sink drain the producer's 100 messages
  # (~10s), then stop the dataflow so the sink exits cleanly.
  cargo run -p rust-dataflow-example-sink-dynamic &
  sink_pid=$!
  sleep 15
  dora stop --name ci-rust-dynamic --grace-duration 5s
  deadline=$((SECONDS + 30))
  while kill -0 "$sink_pid" 2>/dev/null; do
    if [ "$SECONDS" -ge "$deadline" ]; then
      echo "dynamic sink did not exit within 30s after dora stop"
      kill "$sink_pid" 2>/dev/null || true
      wait "$sink_pid" 2>/dev/null || true
      dora destroy 2>/dev/null || true
      return 1
    fi
    sleep 1
  done
  wait "$sink_pid"
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
    local dora_python_api
    # Capture an absolute path before the template subshell changes directory.
    dora_python_api="$PWD/apis/python/node"

    # Python template: dora new + dora build + ruff + pytest + dora run.
    # Generated at repo root for the same reason as the Rust template
    # above — the CLI template's relative path-dep needs the dora
    # workspace as a parent (#1862). Matches
    # `.github/workflows/nightly.yml::cli-tests`.
    rm -rf test_python_project
    (
      dora new test_python_project --lang python --internal-create-with-path-dependencies
      cd test_python_project
      dora build dataflow.yml --uv
      uv pip install --quiet -e "$dora_python_api" -e talker-1 -e talker-2 -e listener-1
      uv run ruff check .
      uv run pytest
      export OPERATING_MODE=SAVE
      dora build dataflow.yml --uv
      timeout 120s dora run dataflow.yml --uv --stop-after 10s
      # Assert no per-node failures hid behind dora-run's 10s timer (#1863).
      assert_clean_dataflow_run "out"
    )
    rm -rf test_python_project

    # Python Node example. `dora run X/dataflow.yml` puts `out/` next
    # to the yaml, so the assertion scans `examples/python-dataflow/out`.
    # Pre-clean stale logs from prior runs so the assert only sees
    # this invocation's per-node behavior (#1863).
    rm -rf examples/python-dataflow/out
    dora build examples/python-dataflow/dataflow.yml --uv
    timeout 60s dora run examples/python-dataflow/dataflow.yml --uv --stop-after 10s
    assert_clean_dataflow_run examples/python-dataflow/out

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
    rm -rf examples/python-operator-dataflow/out
    dora build examples/python-operator-dataflow/dataflow.yml --uv
    uv pip install --quiet -e apis/python/node
    timeout 120s dora run examples/python-operator-dataflow/dataflow.yml --uv --stop-after 20s
    assert_clean_dataflow_run examples/python-operator-dataflow/out

    # Python Multiple Arrays
    rm -rf examples/python-multiple-arrays/out
    dora build examples/python-multiple-arrays/dataflow.yml --uv
    timeout 120s dora run examples/python-multiple-arrays/dataflow.yml --uv --stop-after 30s
    assert_clean_dataflow_run examples/python-multiple-arrays/out

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

  local pyo3_python="${PYO3_PYTHON:-}"
  if [[ -z "$pyo3_python" ]]; then
    pyo3_python="$(command -v python || command -v python3 || true)"
  fi

  if [[ -n "$pyo3_python" ]]; then
    (
      unset PYO3_NO_PYTHON
      PYO3_PYTHON="$pyo3_python" cargo hack check --rust-version --workspace --ignore-private --locked
    )
  else
    cargo hack check --rust-version --workspace --ignore-private --locked
  fi
}

# -----------------------------------------------------------------------------
# Job: kani-proofs (all platforms)
# Mirrors nightly.yml `kani-proofs`. Runs the formal-verification proof
# harnesses (docs/formal-verification.md). Skipped when Kani isn't
# installed/set up (make qa-kani-install).
# -----------------------------------------------------------------------------
job_kani_proofs() {
  if ! cargo kani --version > /dev/null 2>&1 \
    || ! ls "${KANI_HOME:-$HOME/.kani}"/kani-*/ > /dev/null 2>&1; then
    echo "SKIP kani-proofs: Kani not installed/set up (make qa-kani-install)"
    SKIPPED+=("kani-proofs: kani missing")
    return 0
  fi
  scripts/qa/kani.sh
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
# Job 14: ros2-bridge (Linux)
# Mirrors nightly.yml `ros2-bridge`: basic checks that need no ROS distro.
# When a ROS2 Humble env is present (e.g. the ros2dev image) it ALSO runs the
# full bridge examples for extra local coverage -- CI no longer does, because
# ros-tooling/setup-ros pulls external release assets that 404 and break
# nightly; those examples live in scripts/ros2dev.sh qa (release QA) instead.
# -----------------------------------------------------------------------------
job_ros2_bridge() {
  if [ "$OS" != "Linux" ]; then
    echo "SKIP ros2-bridge: GHA runs this on ubuntu-latest only"
    SKIPPED+=("ros2-bridge: non-Linux ($OS)")
    return 0
  fi
  # Basic checks -- mirror nightly.yml `ros2-bridge` (no ROS distro required).
  timeout 600s cargo check -p dora-ros2-bridge --no-default-features
  timeout 600s cargo test -p dora-ros2-bridge-msg-gen
  # dora-ros2-bridge-python's test_utils.py imports numpy + pyarrow. The
  # workflow installs them (`pip install pyarrow numpy`); do the same here in a
  # throwaway venv so this runs cleanly on a Linux box without ROS2 instead of
  # failing with ModuleNotFoundError.
  local ros2_basic_venv
  ros2_basic_venv="$(mktemp -d -t dora-ros2-basic-venv-XXXXXX)"
  uv venv --seed -p 3.12 "$ros2_basic_venv" >/dev/null
  # shellcheck disable=SC1091
  source "$ros2_basic_venv/bin/activate"
  uv pip install --quiet pyarrow numpy
  timeout 600s cargo test -p dora-ros2-bridge-python
  deactivate 2>/dev/null || true
  rm -rf "$ros2_basic_venv"

  # Extra local coverage: full bridge examples when ROS2 Humble is available.
  if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "SKIP ros2-bridge full examples: /opt/ros/humble/setup.bash not found (basic checks ran)"
    SKIPPED+=("ros2-bridge: full examples (ROS2 Humble not installed)")
    return 0
  fi
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  timeout 1800s env QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example rust-ros2-dataflow
  # Self-contained parameter example: declares + exercises ROS2 parameters via
  # the local API (no discovery), so it runs on every platform incl. arm64.
  timeout 1800s env QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example rust-ros2-dataflow-parameter
  timeout 1800s env QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example cxx-ros2-dataflow --features ros2-examples

  # C++ service server, driven by the rclcpp minimal client. Mirrors the
  # nightly.yml "C++ ROS2 Bridge service-server example" step. The example peer
  # `examples_rclcpp_minimal_client` must be present in the ROS2 env (the
  # ros2dev image / a dev ROS2 setup). We do NOT escalate privileges
  # interactively here -- a `sudo apt-get` that prompts for a password would
  # hang this driver with no timeout. So: install only non-interactively (as
  # root directly, or via `sudo -n`), and if the package still isn't available,
  # skip with a clear message rather than hang or fail the whole job.
  if ! ros2 pkg executables examples_rclcpp_minimal_client 2>/dev/null | grep -q client_main; then
    if [ "$(id -u)" = 0 ]; then
      apt-get install -y ros-humble-examples-rclcpp-minimal-client >/dev/null 2>&1 || true
    else
      sudo -n apt-get install -y ros-humble-examples-rclcpp-minimal-client >/dev/null 2>&1 || true
    fi
  fi
  if ros2 pkg executables examples_rclcpp_minimal_client 2>/dev/null | grep -q client_main; then
    timeout 1800s env QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example cxx-ros2-dataflow-service-server --features ros2-examples
  else
    echo "SKIP cxx-ros2-dataflow-service-server: examples_rclcpp_minimal_client not installed (apt-get install ros-humble-examples-rclcpp-minimal-client)"
    SKIPPED+=("ros2-bridge: cxx-service-server (examples_rclcpp_minimal_client missing)")
  fi
  # Note: the Rust action client/server AND the C++ action examples are NOT
  # mirrored here, and as of PR #1988 are no longer in nightly.yml either --
  # their deferred get_result round-trip is flaky in upstream ros2-client/rustdds
  # (it repeatedly hung the x86 nightly job and stalls on arm64/Docker, which
  # this local driver runs on). Validate them via `scripts/ros2dev.sh qa` on x86
  # Linux at release time (tracked in #1170). The C++ *service* server above is
  # mirrored because it completes reliably.

  # Python service client/server examples need the workspace node bindings.
  uv venv --seed -p 3.12 .venv-ros2-bridge >/dev/null
  # shellcheck disable=SC1091
  source .venv-ros2-bridge/bin/activate
  uv pip install -q -e apis/python/node pyarrow
  timeout 1800s env QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example python-ros2-dataflow-service-client
  timeout 1800s env QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example python-ros2-dataflow-service-server
  deactivate
}

# -----------------------------------------------------------------------------
# Dispatch
# -----------------------------------------------------------------------------

run_job "record-replay"             job_record_replay
run_job "cluster-smoke"             job_cluster_smoke
run_job "cluster-e2e"               job_cluster_e2e
run_job "cluster-record-replay"     job_cluster_record_replay
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
run_job "kani-proofs"               job_kani_proofs

echo
echo "============================================================"
if [ ${#SKIPPED[@]} -gt 0 ]; then
  echo "SKIPPED:"
  printf '  - %s\n' "${SKIPPED[@]}"
fi
if [ ${#FAILED[@]} -eq 0 ]; then
  if [ "$SELECTED_JOB_COUNT" -gt 0 ]; then
    echo "All selected CI-nightly jobs passed."
  else
    echo "All CI-nightly jobs passed."
  fi
  exit 0
else
  echo "FAILED:"
  printf '  - %s\n' "${FAILED[@]}"
  exit 1
fi
