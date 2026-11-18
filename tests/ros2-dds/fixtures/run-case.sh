#!/usr/bin/env bash
# One Python-surface cell of the DDS interop matrix: an rclpy peer in the ROS
# container against `dora_peer.py --transport dds` on the host. Exits 0 when
# both sides pass. Called by scripts/ros2-dds-interop.sh, which exports the
# DORA_ROS2_DDS_* variables used here.
set -euo pipefail

DISTRO=$1 RMW=$2 CASE=$3 PROJECT=$4 SERVICE=$5
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
PEERS=/workspace/tests/ros2-zenoh/peers
compose=(docker compose -f "$ROOT/docker-compose.ros2-dds.yml" --profile "$DISTRO" -p "$PROJECT")

# The rclpy peers are shared with the Zenoh harness; they are RMW-agnostic.
case "$CASE" in
  topic-pub) peer="topic_peer.py subscribe" ;;
  topic-sub) peer="topic_peer.py publish" ;;
  service-client) peer="service_peer.py server" ;;
  service-server) peer="service_peer.py client" ;;
  action-client) peer="action_peer.py server" ;;
  action-server) peer="action_peer.py client" ;;
  graph) peer="graph_peer.py graph" ;;
  domain) peer="graph_peer.py domain" ;;
  namespace) peer="topic_peer.py subscribe --topic /robot/chatter" ;;
  qos-transient-local) peer="topic_peer.py publish-transient" ;;
  parameters) peer="parameter_peer.py" ;;
  rosout) peer="rosout_peer.py --transport dds" ;;
  *) echo "unknown Python case: $CASE" >&2; exit 2 ;;
esac

log_dir="$DORA_ROS2_DDS_ARTIFACTS/$RMW"
mkdir -p "$log_dir"
peer_log="$log_dir/${CASE}-rclpy.log"
dora_log="$log_dir/${CASE}-dora.log"

timeout -k 30s 60 "${compose[@]}" exec -T \
  -e RMW_IMPLEMENTATION="$RMW" -e ROS_DOMAIN_ID="$DORA_ROS2_DDS_DOMAIN" "$SERVICE" \
  bash -lc "source /opt/ros/$DISTRO/setup.bash && python3 $PEERS/$peer" \
  >"$peer_log" 2>&1 &
peer_pid=$!
# Killing the local `compose exec` does not stop the process inside the
# container, so also kill any peer left there: a stale server from a failed
# cell would otherwise answer the next cell's requests.
cleanup() {
  kill "$peer_pid" 2>/dev/null || true
  "${compose[@]}" exec -T "$SERVICE" pkill -f "$PEERS/" >/dev/null 2>&1 || true
}
trap cleanup EXIT

timeout -k 30s 30 bash -c 'until grep -q '"'"'"event": "READY"'"'"' "$1"; do kill -0 "$2" 2>/dev/null || exit 1; sleep .2; done' _ "$peer_log" "$peer_pid"

RMW_IMPLEMENTATION="$RMW" ROS_DOMAIN_ID="$DORA_ROS2_DDS_DOMAIN" \
  PYTHONPATH="$DORA_ROS2_DDS_PYTHON" AMENT_PREFIX_PATH="$DORA_ROS2_DDS_AMENT" \
  timeout -k 30s 60 "$DORA_ROS2_DDS_PYTHON_EXECUTABLE" \
    "$ROOT/tests/ros2-zenoh/peers/dora_peer.py" "$CASE" \
    --transport dds --profile humble --ament "$DORA_ROS2_DDS_AMENT" \
    >"$dora_log" 2>&1
wait "$peer_pid"
grep -q '"event": "PASS"' "$peer_log"
