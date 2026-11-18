#!/usr/bin/env bash
# One YAML-bridge cell of the DDS interop matrix: an examples/ros2-bridge
# dataflow (`dora run`) against an rclpy peer in the ROS container. Exits 0
# when both sides pass. Called by scripts/ros2-dds-interop.sh, which puts the
# per-distro `dora-ros2-bridge-node` first on PATH and exports DORA_ROS2_DDS_*.
set -euo pipefail

DISTRO=$1 RMW=$2 CASE=$3 PROJECT=$4 SERVICE=$5
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
EX="$ROOT/examples/ros2-bridge"

# <dataflow> <rclpy peer> <server|client> <endpoint kind> <endpoint> <dora-log check>
case "$CASE" in
  yaml-service-server) set -- yaml-bridge-service/dataflow-server.yml "service_peer.py client" server service /add_two_ints "" ;;
  yaml-service-client) set -- yaml-bridge-service/dataflow-client.yml "service_peer.py server" client service /add_two_ints "Received response: sum = 11" ;;
  yaml-action-server) set -- yaml-bridge-action-server/dataflow.yml "action_peer.py client --no-cancel" server action /fibonacci "" ;;
  yaml-action-server-cancel) set -- yaml-bridge-action-server/dataflow.yml "action_peer.py client" server action /fibonacci "" ;;
  yaml-action-client) set -- yaml-bridge-action/dataflow.yml "action_peer.py server" client action /fibonacci "Result: sequence=" ;;
  *) echo "unknown YAML case: $CASE" >&2; exit 2 ;;
esac
dataflow=$1 peer=$2 role=$3 kind=$4 endpoint=$5 dora_check=$6

dora_pid=
cleanup() {
  # SIGINT makes `dora run` stop the dataflow and its nodes cleanly; force
  # the process and its children down if that has not happened in 5 s.
  if [[ -n $dora_pid ]]; then
    kill -INT "$dora_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do kill -0 "$dora_pid" 2>/dev/null || break; sleep .25; done
    pkill -KILL -P "$dora_pid" 2>/dev/null || true
    kill -KILL "$dora_pid" 2>/dev/null || true
  fi
  stop_peers
}
trap cleanup EXIT

start_dora() {
  (cd "$EX/$(dirname "$dataflow")" &&
    RMW_IMPLEMENTATION="$RMW" ROS_DOMAIN_ID="$DORA_ROS2_DDS_DOMAIN" \
    AMENT_PREFIX_PATH="$DORA_ROS2_DDS_AMENT" \
    exec dora run "$(basename "$dataflow")") >"$dora_log" 2>&1 &
  dora_pid=$!
}
listed() {
  local daemon_flag=--no-daemon
  [[ $kind == action ]] && daemon_flag= # Humble's `ros2 action list` has no --no-daemon
  in_container "ros2 $kind list $daemon_flag" 2>/dev/null | grep -qx "$endpoint"
}
dora_ok() { [[ -z $dora_check ]] || grep -q "$dora_check" "$dora_log"; }
done_ok() { peer_said PASS && dora_ok; }

if [[ $role == server ]]; then
  start_dora
  wait_for 60 listed || { echo "$kind $endpoint never appeared in the ROS graph" >&2; exit 1; }
  start_peer "$peer"
  wait "$peer_pid" || true
  peer_pid=
  done_ok
else
  start_peer "$peer"
  wait_for 30 peer_ready || { echo "rclpy peer never became ready" >&2; exit 1; }
  start_dora
  wait_for 60 done_ok
fi
