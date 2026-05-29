#!/usr/bin/env bash
# ROS2-bridge dev harness for macOS — issue #1170, Phase 0.
#
# Reproduces the nightly `ros2-bridge` CI job (.github/workflows/nightly.yml)
# inside a single Linux+ROS2-Humble container, since ROS2 cannot run natively on
# macOS. dora and the ROS2 nodes share one network namespace, so DDS discovery
# behaves exactly as on the CI runner.
#
#   scripts/ros2dev.sh build     Build the dev image.
#   scripts/ros2dev.sh shell      Open an interactive shell (ROS2 pre-sourced).
#   scripts/ros2dev.sh verify     Run the full CI ros2-bridge job (test + 3 examples).
#   scripts/ros2dev.sh verify --quick   Test + Rust example only (faster smoke).
set -euo pipefail

cd "$(dirname "$0")/.."
COMPOSE=(docker compose -f docker-compose.ros2dev.yml)

require_daemon() {
  if ! docker info >/dev/null 2>&1; then
    echo "error: Docker daemon not reachable. Start Docker Desktop (open -a Docker) and retry." >&2
    exit 1
  fi
}

cmd="${1:-verify}"
shift || true

case "$cmd" in
  build)
    require_daemon
    "${COMPOSE[@]}" build
    ;;

  shell)
    require_daemon
    "${COMPOSE[@]}" run --rm ros2dev bash
    ;;

  verify)
    require_daemon
    quick=false
    [ "${1:-}" = "--quick" ] && quick=true

    # The body runs INSIDE the container. Mirrors nightly.yml `ros2-bridge`
    # step-for-step. The Rust example self-spawns turtlesim_node and
    # examples_rclcpp_minimal_service via `ros2 run`, so no separate ROS2
    # process is needed.
    read -r -d '' SCRIPT <<'EOF' || true
# No `set -u`: ROS2's setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail
source /opt/ros/humble/setup.bash
echo "== AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH}"
echo "== ros2 distro: $ROS_DISTRO  rustc: $(rustc --version)  python: $(python --version)"

echo "::: [1/4] cargo test -p dora-ros2-bridge-python"
cargo test -p dora-ros2-bridge-python

echo "::: [2/4] Rust ROS2 bridge example (rust-ros2-dataflow)"
QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example rust-ros2-dataflow

if [ "${QUICK:-false}" = "true" ]; then
  echo "::: quick mode — skipping C++ and Python examples"
  echo "::: ros2-bridge quick verify OK"
  exit 0
fi

echo "::: [3/4] C++ ROS2 bridge example (cxx-ros2-dataflow)"
QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example cxx-ros2-dataflow --features ros2-examples

echo "::: [4/4] Python ROS2 bridge examples (topic + service client/server)"
# /opt/venv already has pyarrow; add the workspace node bindings (matches CI's
# `uv pip install -e apis/python/node`).
uv pip install -e apis/python/node
QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example python-ros2-dataflow
QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example python-ros2-dataflow-service-client
QT_QPA_PLATFORM=offscreen cargo run -p dora-ros2-bridge --example python-ros2-dataflow-service-server

echo "::: ros2-bridge full verify OK"
EOF

    "${COMPOSE[@]}" run --rm -e QUICK="$quick" ros2dev bash -lc "$SCRIPT"
    ;;

  *)
    echo "usage: scripts/ros2dev.sh {build|shell|verify [--quick]}" >&2
    exit 2
    ;;
esac
