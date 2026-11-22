#!/usr/bin/env bash
# ROS 2 DDS interop matrix: dora (Python `Ros2Context` and the YAML bridge)
# against real rclpy peers over Fast-DDS and CycloneDDS. See
# tests/ros2-dds/README.md.
#
# Usage: scripts/ros2-dds-interop.sh <humble|kilted> [<rmw|all>] [<case|all>]
#        scripts/ros2-dds-interop.sh --self-test
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
DISTROS=(humble kilted)
RMWS=(rmw_fastrtps_cpp rmw_cyclonedds_cpp)
PY_CASES=(topic-pub topic-sub service-client service-server action-client action-server graph domain namespace qos-transient-local parameters rosout)
YAML_CASES=(yaml-service-server yaml-service-client yaml-service-client-concurrent yaml-service-client-sequential yaml-action-server yaml-action-server-cancel yaml-action-server-accept yaml-action-server-reject yaml-action-client)
EXPECTED=${DORA_ROS2_DDS_EXPECTED:-$ROOT/tests/ros2-dds/expected-failures.txt}

in_list() { local x=$1; shift; [[ " $* " == *" $x "* ]]; }

# Load and check the expected-failure list into ENTRIES ("distro rmw case issue
# [flaky]"); exit 2 on the first bad entry, so a typo can never silently stop
# matching its cell.
ENTRIES=()
load_expected() {
  local n=0 line distro rmw case issue flag rest
  [[ -f "$EXPECTED" ]] || return 0
  while IFS= read -r line || [[ -n "$line" ]]; do
    n=$((n + 1))
    line=${line%%#[^0-9]*}          # drop a trailing "# reason", keep "#<issue>"
    [[ "$line" =~ ^[[:space:]]*(#|$) ]] && continue
    read -r distro rmw case issue flag rest <<<"$line"
    if ! { [[ $distro == '*' ]] || in_list "$distro" "${DISTROS[@]}"; } ||
       ! { [[ $rmw == '*' ]] || in_list "$rmw" "${RMWS[@]}"; } ||
       ! in_list "$case" "${PY_CASES[@]}" "${YAML_CASES[@]}" ||
       [[ ! "$issue" =~ ^#[0-9]+$ ]] || [[ -n "$flag" && $flag != flaky ]] || [[ -n "$rest" ]]; then
      echo "$EXPECTED:$n: invalid entry (want '<distro|*> <rmw|*> <case> #<issue> [flaky]'): $line" >&2
      exit 2
    fi
    ENTRIES+=("$distro $rmw $case $issue $flag")
  done <"$EXPECTED"
}

# Print "<issue> [flaky]" for the expected-failure entry matching a cell, if any.
expected_issue() { # <distro> <rmw> <case>
  local entry distro rmw case issue flag
  for entry in "${ENTRIES[@]}"; do
    read -r distro rmw case issue flag <<<"$entry"
    if [[ ($distro == '*' || $distro == "$1") && ($rmw == '*' || $rmw == "$2") && $case == "$3" ]]; then
      echo "$issue${flag:+ $flag}"
      return
    fi
  done
}

# Print the result line for one cell. Returns 0 for PASS/XFAIL, 1 for FAIL/XPASS.
# A cell listed as flaky (a timing-dependent bug) may pass or fail: both are ok.
classify() { # <distro> <rmw> <case> <case-exit-status>
  local issue
  issue=$(expected_issue "$1" "$2" "$3")
  if [[ -n "$issue" ]]; then
    if [[ $4 -ne 0 ]]; then echo "XFAIL $1 $2 $3 ($issue)"; return 0; fi
    if [[ $issue == *flaky ]]; then echo "PASS $1 $2 $3 ($issue)"; return 0; fi
    echo "XPASS $1 $2 $3 ($issue)"; return 1
  fi
  if [[ $4 -eq 0 ]]; then echo "PASS $1 $2 $3"; return 0; fi
  echo "FAIL $1 $2 $3"; return 1
}

self_test() {
  local tmp rc=0 out
  tmp=$(mktemp -d)
  # shellcheck disable=SC2064
  trap "rm -rf '$tmp'" RETURN
  cat >"$tmp/ok.txt" <<'EOF'
# comment
#
humble rmw_cyclonedds_cpp graph #101  # listed for one cell
* * yaml-action-server-cancel #102   # wildcards
* rmw_cyclonedds_cpp service-client #103 flaky  # timing-dependent
EOF
  expect() { # <name> <want-output> <want-rc> <got-output> <got-rc>
    if [[ "$4" == "$2" && "$5" == "$3" ]]; then echo "ok   $1"
    else echo "FAIL $1: want '$2' rc=$3, got '$4' rc=$5"; rc=1; fi
  }
  for c in \
    "listed+fail|humble rmw_cyclonedds_cpp graph 1|XFAIL humble rmw_cyclonedds_cpp graph (#101)|0" \
    "listed+pass|humble rmw_cyclonedds_cpp graph 0|XPASS humble rmw_cyclonedds_cpp graph (#101)|1" \
    "unlisted+fail|kilted rmw_cyclonedds_cpp graph 1|FAIL kilted rmw_cyclonedds_cpp graph|1" \
    "unlisted+pass|humble rmw_fastrtps_cpp graph 0|PASS humble rmw_fastrtps_cpp graph|0" \
    "wildcard+fail|kilted rmw_fastrtps_cpp yaml-action-server-cancel 1|XFAIL kilted rmw_fastrtps_cpp yaml-action-server-cancel (#102)|0" \
    "flaky+fail|humble rmw_cyclonedds_cpp service-client 1|XFAIL humble rmw_cyclonedds_cpp service-client (#103 flaky)|0" \
    "flaky+pass|kilted rmw_cyclonedds_cpp service-client 0|PASS kilted rmw_cyclonedds_cpp service-client (#103 flaky)|0"; do
    IFS='|' read -r name args want_out want_rc <<<"$c"
    # shellcheck disable=SC2086
    out=$(DORA_ROS2_DDS_EXPECTED="$tmp/ok.txt" "$0" --classify $args 2>&1) && got=0 || got=$?
    expect "$name" "$want_out" "$want_rc" "$out" "$got"
  done
  printf '%s\n' "humble rmw_fastrtps_cpp no-such-case #1" >"$tmp/bad-case.txt"
  printf '%s\n' "humble rmw_fastrtps_cpp graph   # no issue" >"$tmp/bad-issue.txt"
  printf '%s\n' "jazzy rmw_fastrtps_cpp graph #1" >"$tmp/bad-distro.txt"
  printf '%s\n' "humble rmw_connext graph #1" >"$tmp/bad-rmw.txt"
  printf '%s\n' "humble rmw_fastrtps_cpp graph #1 sometimes" >"$tmp/bad-flag.txt"
  for f in bad-case bad-issue bad-distro bad-rmw bad-flag; do
    DORA_ROS2_DDS_EXPECTED="$tmp/$f.txt" "$0" --classify humble rmw_fastrtps_cpp graph 0 >/dev/null 2>&1 && got=0 || got=$?
    expect "rejects $f" "" 2 "" "$got"
  done
  return $rc
}

case "${1:-}" in
  --self-test) self_test; exit $? ;;
  --classify) shift; load_expected; classify "$@"; exit $? ;;
esac

DISTRO=${1:-} RMW_ARG=${2:-all} CASE_ARG=${3:-all}
usage="usage: $0 <humble|kilted> [<rmw_fastrtps_cpp|rmw_cyclonedds_cpp|all>] [<case|all>]"
in_list "$DISTRO" "${DISTROS[@]}" || { echo "$usage" >&2; exit 2; }
[[ $RMW_ARG == all ]] || in_list "$RMW_ARG" "${RMWS[@]}" || { echo "unknown rmw: $RMW_ARG" >&2; exit 2; }
[[ $CASE_ARG == all ]] || in_list "$CASE_ARG" "${PY_CASES[@]}" "${YAML_CASES[@]}" ||
  { echo "unknown case: $CASE_ARG" >&2; exit 2; }
load_expected
if [[ $RMW_ARG == all ]]; then rmws=("${RMWS[@]}"); else rmws=("$RMW_ARG"); fi
if [[ $CASE_ARG == all ]]; then cases=("${PY_CASES[@]}" "${YAML_CASES[@]}"); else cases=("$CASE_ARG"); fi

# Preflight the host tools before spending minutes booting a ROS container.
missing=()
for tool in docker maturin; do command -v "$tool" >/dev/null || missing+=("$tool"); done
docker compose version >/dev/null 2>&1 || missing+=("docker-compose-plugin")
if ((${#missing[@]})); then
  echo "missing required tool(s): ${missing[*]}" >&2
  exit 1
fi
if [[ -z "${DORA_ROS2_DDS_PYTHON_EXECUTABLE:-}" ]]; then
  for candidate in python3.12 python3.11 python3; do
    if command -v "$candidate" >/dev/null &&
       "$candidate" -c 'import sys; raise SystemExit(sys.version_info < (3, 11))'; then
      DORA_ROS2_DDS_PYTHON_EXECUTABLE=$(command -v "$candidate")
      break
    fi
  done
fi
: "${DORA_ROS2_DDS_PYTHON_EXECUTABLE:?Python 3.11 or newer is required}"
export DORA_ROS2_DDS_PYTHON_EXECUTABLE
# A dedicated domain keeps other DDS traffic on the host out of the matrix.
export DORA_ROS2_DDS_DOMAIN=${DORA_ROS2_DDS_DOMAIN:-23}
# The dora side runs on the host: tell it which distro its peers run, instead of
# inheriting a sourced workspace's ROS_DISTRO (the bridge warns about a distro
# whose Gid width differs from its build).
export ROS_DISTRO="$DISTRO"

# shellcheck source=cargo-target-dir.sh
source "$ROOT/scripts/cargo-target-dir.sh"
TARGET_DIR="$(cargo_target_dir "$ROOT")"
PROJECT="dora-rmw-dds-${DISTRO}-$$"
SERVICE="${DISTRO}-dds"
compose=(docker compose -f "$ROOT/docker-compose.ros2-dds.yml" --profile "$DISTRO" -p "$PROJECT")
cleanup() {
  # Stop any case still running (Ctrl-C), then the container.
  pkill -P $$ 2>/dev/null || true
  "${compose[@]}" down --volumes --remove-orphans >/dev/null 2>&1 || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# Start the container first: its apt install overlaps the builds below.
"${compose[@]}" up -d "$SERVICE"
# The bridge is built per distro (the Gid layout differs from Iron on), into
# a per-distro directory so a Humble build is never reused for Kilted.
#
# DORA_ROS2_DDS_PREBUILT=<dir> runs against bridge artifacts built elsewhere
# (<dir>/python: the installed wheel + pyarrow; <dir>/bin/dora-ros2-bridge-node),
# e.g. an older dora, to check that the matrix catches a known bug.
BUILD="${DORA_ROS2_DDS_PREBUILT:-$TARGET_DIR/ros2-dds-$DISTRO}"
export DORA_ROS2_DDS_PYTHON="$BUILD/python"
if [[ -z "${DORA_ROS2_DDS_PREBUILT:-}" ]]; then
  rm -rf "$BUILD/wheels" "$DORA_ROS2_DDS_PYTHON"
  mkdir -p "$BUILD/wheels" "$DORA_ROS2_DDS_PYTHON"
  maturin build --release --features "ros2-$DISTRO" --out "$BUILD/wheels" \
    --manifest-path "$ROOT/libraries/extensions/ros2-bridge/python/Cargo.toml"
  "$DORA_ROS2_DDS_PYTHON_EXECUTABLE" -m pip install --quiet --no-deps \
    --target "$DORA_ROS2_DDS_PYTHON" pyarrow "$BUILD"/wheels/dora_ros2_bridge_python-*.whl
fi

# The YAML cases run examples/ros2-bridge dataflows. Their nodes are prebuilt
# where the dataflows expect them (target/debug), so `dora run`'s own build
# step is a no-op. Only the bridge node depends on the distro: it is copied
# into a per-distro bin dir that goes first on PATH, where `dora run` finds it.
#
# Each node package is built by its own `cargo build -p <package>`, the command
# its dataflow runs. One combined `-p a -p b` build resolves features across
# all the packages, so the dataflow's single-package build would not find its
# dependencies built and would recompile them inside the case's time limit.
if [[ " ${cases[*]} " == *" yaml-"* ]]; then
  cargo build -p dora-cli
  for package in yaml-bridge-service-handler yaml-bridge-service-requester \
    yaml-bridge-action-server-handler yaml-bridge-action-goal; do
    cargo build -p "$package"
  done
  mkdir -p "$BUILD/bin"
  if [[ -z "${DORA_ROS2_DDS_PREBUILT:-}" ]]; then
    cargo build -p dora-ros2-bridge-node --features "ros2-$DISTRO"
    cp "$TARGET_DIR/debug/dora-ros2-bridge-node" "$BUILD/bin/"
  fi
  # The CLI itself does not depend on the distro or the bridge.
  export PATH="$BUILD/bin:$TARGET_DIR/debug:$PATH"
fi

# The container installs its packages while the bridge builds above; wait for it now.
container="${PROJECT}-${SERVICE}-1"
if ! timeout -k 30s 180 bash -c 'until [[ $(docker inspect -f "{{.State.Health.Status}}" "$1" 2>/dev/null) == healthy ]]; do sleep 2; done' _ "$container"; then
  echo "$SERVICE never became healthy within 180s; container log follows:" >&2
  "${compose[@]}" logs --tail 100 "$SERVICE" >&2 || true
  exit 1
fi
# Packages are unpinned (see the compose file); record what landed.
"${compose[@]}" exec -T "$SERVICE" dpkg-query -W -f '${Package}=${Version}\n' \
  "ros-${DISTRO}-rmw-fastrtps-cpp" "ros-${DISTRO}-rmw-cyclonedds-cpp" "ros-${DISTRO}-rclpy" | tr -d '\r'

export DORA_ROS2_DDS_ARTIFACTS="$TARGET_DIR/ros2-dds-logs/$PROJECT"
export DORA_ROS2_DDS_AMENT="$DORA_ROS2_DDS_ARTIFACTS/ament"
mkdir -p "$DORA_ROS2_DDS_AMENT"
docker cp "$container:/opt/ros/$DISTRO/share" "$DORA_ROS2_DDS_AMENT/"

failed=0
results=()
for rmw in "${rmws[@]}"; do
  for case in "${cases[@]}"; do
    echo "CASE $DISTRO $rmw $case"
    runner=run-case.sh
    [[ $case == yaml-* ]] && runner=run-yaml-case.sh
    "$ROOT/tests/ros2-dds/fixtures/$runner" "$DISTRO" "$rmw" "$case" "$PROJECT" "$SERVICE" &&
      status=0 || status=$?
    line=$(classify "$DISTRO" "$rmw" "$case" "$status") || failed=1
    echo "$line"
    results+=("$line")
  done
done

echo "== $DISTRO summary (logs: $DORA_ROS2_DDS_ARTIFACTS)"
printf '%s\n' "${results[@]}"
exit "$failed"
