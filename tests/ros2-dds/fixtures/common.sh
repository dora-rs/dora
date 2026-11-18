# shellcheck shell=bash
# Shared by run-case.sh and run-yaml-case.sh. The caller sets DISTRO, RMW,
# CASE, PROJECT and SERVICE, and scripts/ros2-dds-interop.sh exports the
# DORA_ROS2_DDS_* variables used here.

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
# The rclpy peers are shared with the Zenoh harness; they are RMW-agnostic.
PEERS=/workspace/tests/ros2-zenoh/peers
compose=(docker compose -f "$ROOT/docker-compose.ros2-dds.yml" --profile "$DISTRO" -p "$PROJECT")

log_dir="$DORA_ROS2_DDS_ARTIFACTS/$RMW"
mkdir -p "$log_dir"
peer_log="$log_dir/${CASE}-rclpy.log"
dora_log="$log_dir/${CASE}-dora.log"
peer_pid=

# Run a shell command in the ROS container with this cell's RMW and domain.
in_container() {
  "${compose[@]}" exec -T -e RMW_IMPLEMENTATION="$RMW" -e ROS_DOMAIN_ID="$DORA_ROS2_DDS_DOMAIN" \
    "$SERVICE" bash -lc "source /opt/ros/$DISTRO/setup.bash && $1"
}

# Start an rclpy peer ("<script> [args]" in the peers dir) in the background.
start_peer() {
  in_container "timeout 60 python3 $PEERS/$1" >"$peer_log" 2>&1 &
  peer_pid=$!
}

# Killing the local `compose exec` does not stop the process inside the
# container, so also kill any peer left there: a stale server from a failed
# cell would otherwise answer the next cell's requests. This runs from EXIT
# traps under `set -e`, so no step may fail: that would replace the cell's
# exit status.
stop_peers() {
  if [[ -n $peer_pid ]]; then kill "$peer_pid" 2>/dev/null || true; fi
  "${compose[@]}" exec -T "$SERVICE" pkill -f "$PEERS/" >/dev/null 2>&1 || true
}

# wait_for <seconds> <condition...>: poll until the condition holds.
wait_for() {
  local deadline=$(( $(date +%s) + $1 )); shift
  until "$@"; do (( $(date +%s) < deadline )) || return 1; sleep .2; done
}

peer_said() { grep -q "\"event\": \"$1\"" "$peer_log"; }

# The peer printed READY. A peer that has already exited fails the cell at
# once instead of after the timeout (e.g. an RMW that is not installed).
peer_ready() {
  peer_said READY && return
  kill -0 "$peer_pid" 2>/dev/null || { echo "rclpy peer exited before READY" >&2; exit 1; }
  return 1
}
