#!/usr/bin/env bash
# One Python-surface cell of the DDS interop matrix: an rclpy peer in the ROS
# container against `dora_peer.py --transport dds` on the host. Exits 0 when
# both sides pass. Called by scripts/ros2-dds-interop.sh.
set -euo pipefail

DISTRO=$1 RMW=$2 CASE=$3 PROJECT=$4 SERVICE=$5
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

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
  rosout) peer="rosout_peer.py" ;;
  *) echo "unknown Python case: $CASE" >&2; exit 2 ;;
esac

trap stop_peers EXIT
start_peer "$peer"
wait_for 30 peer_ready

RMW_IMPLEMENTATION="$RMW" ROS_DOMAIN_ID="$DORA_ROS2_DDS_DOMAIN" \
  PYTHONPATH="$DORA_ROS2_DDS_PYTHON" AMENT_PREFIX_PATH="$DORA_ROS2_DDS_AMENT" \
  timeout -k 30s 60 "$DORA_ROS2_DDS_PYTHON_EXECUTABLE" \
    "$ROOT/tests/ros2-zenoh/peers/dora_peer.py" "$CASE" \
    --transport dds --ament "$DORA_ROS2_DDS_AMENT" \
    >"$dora_log" 2>&1
wait "$peer_pid"
peer_said PASS
