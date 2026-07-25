#!/usr/bin/env bash
set -euo pipefail

DISTRO=$1 CASE=$2 PROJECT=$3 SERVICE=$4
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
compose=(docker compose -f "$ROOT/docker-compose.ros2-zenoh.yml" --profile "$DISTRO" -p "$PROJECT")

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
  *)
    echo "MISSING_CAPABILITY Dora fixture for $CASE" >&2
    exit 1
    ;;
esac

log="$DORA_ROS2_ZENOH_ARTIFACTS/${CASE}-rclpy.log"
timeout 60 "${compose[@]}" exec -T "$SERVICE" bash -lc \
  "source /opt/ros/$DISTRO/setup.bash && python3 /workspace/tests/ros2-zenoh/peers/$peer" \
  >"$log" 2>&1 &
peer_pid=$!
trap 'kill "$peer_pid" 2>/dev/null || true' EXIT
timeout 20 bash -c 'until grep -q '"'"'"event": "READY"'"'"' "$1"; do kill -0 "$2" 2>/dev/null || exit 1; sleep .2; done' _ "$log" "$peer_pid"

profile=rep2016
[[ "$DISTRO" == humble ]] && profile=humble
PYTHONPATH="$DORA_ROS2_ZENOH_PYTHON" AMENT_PREFIX_PATH="$DORA_ROS2_ZENOH_AMENT" \
  timeout 60 "$DORA_ROS2_ZENOH_PYTHON_EXECUTABLE" \
    "$ROOT/tests/ros2-zenoh/peers/dora_peer.py" "$CASE" \
    --profile "$profile" --ament "$DORA_ROS2_ZENOH_AMENT" \
    --config "$ROOT/tests/ros2-zenoh/fixtures/zenoh-client.json5"
wait "$peer_pid"
grep -q '"event": "PASS"' "$log"
trap - EXIT
