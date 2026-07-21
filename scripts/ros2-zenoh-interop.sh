#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
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

PROJECT="dora-rmw-zenoh-${DISTRO}-$$"
compose=("${COMPOSE[@]}" "$DISTRO" -p "$PROJECT")
service="${DISTRO}-router"
cleanup() { "${compose[@]}" down --volumes --remove-orphans >/dev/null 2>&1 || true; }
trap cleanup EXIT INT TERM

"${compose[@]}" up -d "$service"
timeout 180 bash -c 'until [[ $(docker inspect -f "{{.State.Health.Status}}" "$1" 2>/dev/null) == healthy ]]; do sleep 2; done' _ "${PROJECT}-${service}-1"
"${compose[@]}" exec -T "$service" bash -lc \
  "source /opt/ros/$DISTRO/setup.bash; dpkg-query -W 'ros-${DISTRO}-rmw-zenoh-cpp'; ros2 pkg executables rmw_zenoh_cpp"

container="${PROJECT}-${service}-1"
export DORA_ROS2_ZENOH_ARTIFACTS="$ROOT/target/ros2-zenoh-logs/$PROJECT"
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
WHEEL_DIR="${CARGO_TARGET_DIR:-$ROOT/target}/wheels"
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
