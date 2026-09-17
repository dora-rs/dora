#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# shellcheck source=cargo-target-dir.sh
source "$ROOT/scripts/cargo-target-dir.sh"
TARGET_DIR="$(cargo_target_dir "$ROOT")"
COMPOSE=(docker compose -f "$ROOT/docker-compose.ros2-zenoh.yml" --profile)
CASES=(topic-pub topic-sub service-client service-server action-client action-server graph domain namespace qos-transient-local)
DISTRO=${1:-}
CASE=${2:-all}

if [[ "$DISTRO" != humble && "$DISTRO" != kilted ]]; then
  echo "usage: $0 <humble|kilted> <case|all>" >&2
  exit 2
fi
if [[ "$CASE" != all ]] && [[ ! " ${CASES[*]} " =~ " $CASE " ]]; then
  echo "unknown case: $CASE" >&2
  exit 2
fi

# Preflight the host tools before spending ~2 min pulling and booting a
# ROS container. `maturin` in particular used to surface only as a bare
# `line 49: maturin: command not found` (exit 127) *after* the pull, on
# every nightly from #2790 until #2742 — far enough from the cause to
# read as a ROS problem rather than a missing build tool.
missing=()
for tool in docker maturin; do
  command -v "$tool" >/dev/null || missing+=("$tool")
done
if ((${#missing[@]})); then
  echo "missing required tool(s): ${missing[*]}" >&2
  echo "  docker:  https://docs.docker.com/engine/install/" >&2
  echo "  maturin: pip install maturin" >&2
  exit 1
fi

PROJECT="dora-rmw-zenoh-${DISTRO}-$$"
compose=("${COMPOSE[@]}" "$DISTRO" -p "$PROJECT")
service="${DISTRO}-router"
cleanup() { "${compose[@]}" down --volumes --remove-orphans >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

container="${PROJECT}-${service}-1"

"${compose[@]}" up -d "$service"
# The router installs its rmw_zenoh packages on boot, so "still booting" and
# "the apt install failed" look identical from out here — both are just an
# unhealthy container. Dump the log when the wait runs out, or the whole
# failure is a bare `exit 124` three minutes in: that is how a month of
# nightlies read an unresolvable apt pin as a docker hiccup (#3263). The
# pins are gone now, but a mirror or network hiccup lands here the same way.
if ! timeout -k 30s 180 bash -c 'until [[ $(docker inspect -f "{{.State.Health.Status}}" "$1" 2>/dev/null) == healthy ]]; do sleep 2; done' _ "$container"; then
  echo "$service never became healthy within 180s; container log follows:" >&2
  "${compose[@]}" logs --tail 100 "$service" >&2 || true
  exit 1
fi
# The compose file installs rmw_zenoh unpinned, because an exact `=<version>`
# apt pin stops resolving as soon as upstream publishes a new build and takes
# the whole job down with it — that rot, not any interop bug, is every failure
# these jobs have ever had (#3263, #2987). What the pin was actually good for
# is caught here instead: record the version that landed, so a future breakage
# names it, and floor it, so a build too old to interop fails by name rather
# than as ten confusing case failures.
case "$DISTRO" in
  humble) rmw_floor=0.1.9 ;;
  kilted) rmw_floor=0.6.7 ;;
esac
rmw_installed=$("${compose[@]}" exec -T "$service" \
  dpkg-query -W -f '${Version}' "ros-${DISTRO}-rmw-zenoh-cpp" | tr -d '\r')
echo "ros-${DISTRO}-rmw-zenoh-cpp=$rmw_installed (floor $rmw_floor)"
if ! "${compose[@]}" exec -T "$service" \
  dpkg --compare-versions "$rmw_installed" ge "$rmw_floor"; then
  echo "rmw_zenoh_cpp $rmw_installed is older than the $rmw_floor floor" >&2
  exit 1
fi
"${compose[@]}" exec -T "$service" bash -lc \
  "source /opt/ros/$DISTRO/setup.bash; ros2 pkg executables rmw_zenoh_cpp"

export DORA_ROS2_ZENOH_ARTIFACTS="$TARGET_DIR/ros2-zenoh-logs/$PROJECT"
export DORA_ROS2_ZENOH_PYTHON="$DORA_ROS2_ZENOH_ARTIFACTS/python"
export DORA_ROS2_ZENOH_AMENT="$DORA_ROS2_ZENOH_ARTIFACTS/ament"
if [[ -z "${DORA_ROS2_ZENOH_PYTHON_EXECUTABLE:-}" ]]; then
  for candidate in python3.12 python3.11 python3; do
    if command -v "$candidate" >/dev/null && "$candidate" -c \
      'import sys; raise SystemExit(sys.version_info < (3, 11))'
    then
      DORA_ROS2_ZENOH_PYTHON_EXECUTABLE=$(command -v "$candidate")
      break
    fi
  done
fi
: "${DORA_ROS2_ZENOH_PYTHON_EXECUTABLE:?Python 3.11 or newer is required}"
export DORA_ROS2_ZENOH_PYTHON_EXECUTABLE
WHEEL_DIR="$TARGET_DIR/wheels"
mkdir -p "$DORA_ROS2_ZENOH_PYTHON" "$DORA_ROS2_ZENOH_AMENT"
docker cp "$container:/opt/ros/$DISTRO/share" "$DORA_ROS2_ZENOH_AMENT/"
maturin build --release --manifest-path "$ROOT/libraries/extensions/ros2-bridge/python/Cargo.toml"
"$DORA_ROS2_ZENOH_PYTHON_EXECUTABLE" -m pip install --quiet --no-deps \
  --target "$DORA_ROS2_ZENOH_PYTHON" pyarrow \
  "$WHEEL_DIR"/dora_ros2_bridge_python-*.whl

run_case() {
  local name=$1
  echo "CASE $DISTRO $name"
  "$ROOT/tests/ros2-zenoh/fixtures/run-case.sh" "$DISTRO" "$name" "$PROJECT" "$service"
  echo "PASS $DISTRO $name"
}

if [[ "$CASE" == all ]]; then
  for item in "${CASES[@]}"; do run_case "$item"; done
else
  run_case "$CASE"
fi
