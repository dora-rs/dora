#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

DOMAINS="default"
RUNS=10
STOP_AFTER=35
TIMEOUT=65
ACTION="/fibonacci"
ACTION_TYPE="example_interfaces/action/Fibonacci"
DATAFLOW="examples/ros2-bridge/yaml-bridge-action/dataflow.yml"
SERVER_PACKAGE="examples_rclcpp_minimal_action_server"
SERVER_EXECUTABLE="action_server_member_functions"
ARTIFACT_ROOT="$ROOT/target/ros2-bridge-action-loop/$(date -u +%Y%m%dT%H%M%SZ)"
DORA_BIN="${DORA_BIN:-dora}"
USE_EXISTING_SERVER=0

usage() {
  cat <<'USAGE'
Usage: scripts/ros2-bridge-action-loop.sh [OPTIONS]

Repeatedly validates the YAML ROS2 action-client bridge against a real
rclcpp Fibonacci action server. This is a local ROS2/Humble flake harness,
not part of the normal smoke suite.

Options:
  --domains LIST          Comma-separated domains. Use "default" to unset
                          ROS_DOMAIN_ID. Example: default,23,24 (default: default)
  --runs N                Runs per domain (default: 10)
  --stop-after SECS       dora run --stop-after value in seconds (default: 35)
  --timeout SECS          Outer timeout for each dora run (default: 65)
  --artifacts DIR         Directory for logs and summary files
                          (default: target/ros2-bridge-action-loop/<timestamp>)
  --use-existing-server   Reuse an already-running Fibonacci action server
                          instead of starting one. Use only when its
                          ROS_DOMAIN_ID matches the tested domain.
  -h, --help              Show this help

Examples:
  source /opt/ros/humble/setup.bash
  scripts/ros2-bridge-action-loop.sh --domains default,23 --runs 30
  scripts/ros2-bridge-action-loop.sh --domains 23 --runs 5 --use-existing-server

Environment:
  DORA_BIN                dora executable to run (default: dora from PATH)
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --domains)
      DOMAINS="${2:?--domains requires a value}"
      shift 2
      ;;
    --runs)
      RUNS="${2:?--runs requires a value}"
      shift 2
      ;;
    --stop-after)
      STOP_AFTER="${2:?--stop-after requires a value}"
      shift 2
      ;;
    --timeout)
      TIMEOUT="${2:?--timeout requires a value}"
      shift 2
      ;;
    --artifacts)
      ARTIFACT_ROOT="${2:?--artifacts requires a value}"
      shift 2
      ;;
    --use-existing-server)
      USE_EXISTING_SERVER=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing required command: $1" >&2
    exit 2
  }
}

require_positive_int() {
  local name="$1" value="$2"
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || {
    echo "$name must be a positive integer, got: $value" >&2
    exit 2
  }
}

require_cmd ros2
require_cmd timeout
require_cmd "$DORA_BIN"
require_positive_int "--runs" "$RUNS"
require_positive_int "--stop-after" "$STOP_AFTER"
require_positive_int "--timeout" "$TIMEOUT"

if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "AMENT_PREFIX_PATH is not set. Source ROS2 first, for example:" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  exit 2
fi

server_pattern="/${SERVER_PACKAGE}/${SERVER_EXECUTABLE}"
existing_servers="$(pgrep -af "$server_pattern" || true)"
if [[ "$USE_EXISTING_SERVER" != "1" && -n "$existing_servers" ]]; then
  echo "existing ROS2 Fibonacci action server process found:" >&2
  echo "$existing_servers" >&2
  echo >&2
  echo "Stop it before running this harness, or reuse it explicitly:" >&2
  echo "  scripts/ros2-bridge-action-loop.sh --use-existing-server --domains <matching-domain>" >&2
  echo >&2
  echo "Duplicate action servers can mask discovery or result-correlation flakes." >&2
  exit 2
fi

mkdir -p "$ARTIFACT_ROOT"
SUMMARY_JSONL="$ARTIFACT_ROOT/summary.jsonl"
SUMMARY_MD="$ARTIFACT_ROOT/summary.md"
: > "$SUMMARY_JSONL"

server_pids=()
cleanup() {
  for pid in "${server_pids[@]:-}"; do
    kill "$pid" >/dev/null 2>&1 || true
    wait "$pid" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

domain_label() {
  local domain="$1"
  if [[ "$domain" == "default" || "$domain" == "unset" ]]; then
    echo "default"
  else
    echo "domain${domain}"
  fi
}

domain_display() {
  local domain="$1"
  if [[ "$domain" == "default" || "$domain" == "unset" ]]; then
    echo "default(unset=>0)"
  else
    echo "$domain"
  fi
}

run_with_domain() {
  local domain="$1"
  shift
  if [[ "$domain" == "default" || "$domain" == "unset" ]]; then
    env -u ROS_DOMAIN_ID "$@"
  else
    env ROS_DOMAIN_ID="$domain" "$@"
  fi
}

wait_for_action() {
  local domain="$1" out_dir="$2"
  local action_list_log="$out_dir/ros2-action-list.txt"
  local deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    if run_with_domain "$domain" ros2 action list -t > "$action_list_log" 2>&1 \
      && grep -Fq "$ACTION [$ACTION_TYPE]" "$action_list_log"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

write_result() {
  local domain="$1" run="$2" status="$3" reason="$4" dir="$5"
  printf '{"domain":"%s","run":%s,"status":"%s","reason":"%s","dir":"%s"}\n' \
    "$(domain_display "$domain")" "$run" "$status" "$reason" "$dir" >> "$SUMMARY_JSONL"
}

classify_dora_log() {
  local dora_log="$1"
  local rc="$2"

  if [[ "$rc" -ne 0 ]]; then
    if grep -Fq "ROS2 action service not available" "$dora_log"; then
      echo "action_discovery_failed"
    elif grep -Fq "action goal send timed out" "$dora_log"; then
      echo "action_goal_send_timed_out"
    elif grep -Fq "failed to send action goal" "$dora_log"; then
      echo "action_goal_send_failed"
    elif grep -Fq "panicked at" "$dora_log"; then
      echo "action_node_panic"
    else
      echo "dora_exit_$rc"
    fi
    return
  fi

  if ! grep -Fq "ROS2 action service connected: ${ACTION}/_action/send_goal" "$dora_log"; then
    echo "missing_send_goal_connected_marker"
  elif ! grep -Fq "ROS2 action service connected: ${ACTION}/_action/get_result" "$dora_log"; then
    echo "missing_get_result_connected_marker"
  elif ! grep -Fq "ROS2 action service connected: ${ACTION}/_action/cancel_goal" "$dora_log"; then
    echo "missing_cancel_goal_connected_marker"
  elif ! grep -Fq "Sending Fibonacci goal:" "$dora_log"; then
    echo "missing_goal_send"
  elif ! grep -Fq "Feedback: sequence=" "$dora_log"; then
    echo "missing_feedback"
  elif ! grep -Fq "Result: sequence=" "$dora_log"; then
    echo "missing_result"
  elif grep -Fq "panicked at" "$dora_log"; then
    echo "action_node_panic"
  else
    echo "ok"
  fi
}

echo "# ROS2 Bridge Action Loop" > "$SUMMARY_MD"
{
  echo
  echo "- Artifacts: $ARTIFACT_ROOT"
  echo "- Domains: $DOMAINS"
  echo "- Runs per domain: $RUNS"
  echo "- Dora stop-after: ${STOP_AFTER}s"
  echo "- Per-run timeout: ${TIMEOUT}s"
  echo "- Dataflow: $DATAFLOW"
  echo "- Use existing server: $USE_EXISTING_SERVER"
  echo
} >> "$SUMMARY_MD"

total_pass=0
total_fail=0
IFS=',' read -ra DOMAIN_ITEMS <<< "$DOMAINS"

for raw_domain in "${DOMAIN_ITEMS[@]}"; do
  domain="$(echo "$raw_domain" | xargs)"
  [[ -n "$domain" ]] || continue
  label="$(domain_label "$domain")"
  domain_dir="$ARTIFACT_ROOT/$label"
  mkdir -p "$domain_dir"

  echo "== domain $(domain_display "$domain") =="
  echo "## Domain $(domain_display "$domain")" >> "$SUMMARY_MD"

  if [[ "$USE_EXISTING_SERVER" == "1" ]]; then
    echo "using existing action server" > "$domain_dir/server.log"
  else
    run_with_domain "$domain" ros2 run "$SERVER_PACKAGE" "$SERVER_EXECUTABLE" \
      > "$domain_dir/server.log" 2>&1 &
    server_pid=$!
    server_pids+=("$server_pid")
  fi

  if ! wait_for_action "$domain" "$domain_dir"; then
    echo "  FAIL: action did not appear"
    write_result "$domain" 0 "fail" "action_not_visible" "$domain_dir"
    total_fail=$((total_fail + RUNS))
    echo "- action preflight failed; see \`$domain_dir\`" >> "$SUMMARY_MD"
    continue
  fi

  if ! run_with_domain "$domain" timeout 20s ros2 action send_goal "$ACTION" "$ACTION_TYPE" "{order: 3}" \
      > "$domain_dir/ros2-action-send-goal.txt" 2>&1; then
    echo "  FAIL: ros2 action send_goal failed"
    write_result "$domain" 0 "fail" "ros2_action_send_goal_failed" "$domain_dir"
    total_fail=$((total_fail + RUNS))
    echo "- ros2 action send_goal preflight failed; see \`$domain_dir\`" >> "$SUMMARY_MD"
    continue
  fi

  if ! grep -Fq "Goal finished with status: SUCCEEDED" "$domain_dir/ros2-action-send-goal.txt"; then
    echo "  FAIL: ros2 action did not succeed"
    write_result "$domain" 0 "fail" "ros2_action_goal_not_succeeded" "$domain_dir"
    total_fail=$((total_fail + RUNS))
    echo "- ros2 action preflight did not succeed; see \`$domain_dir\`" >> "$SUMMARY_MD"
    continue
  fi

  domain_pass=0
  domain_fail=0
  for run in $(seq 1 "$RUNS"); do
    run_dir="$domain_dir/run-$run"
    mkdir -p "$run_dir"
    dora_log="$run_dir/dora.log"
    echo "  run $run/$RUNS"

    set +e
    run_with_domain "$domain" timeout "${TIMEOUT}s" "$DORA_BIN" run "$DATAFLOW" \
      --stop-after "${STOP_AFTER}s" > "$dora_log" 2>&1
    rc=$?
    set -e

    reason="$(classify_dora_log "$dora_log" "$rc")"
    if [[ "$reason" == "ok" ]]; then
      echo "    PASS"
      write_result "$domain" "$run" "pass" "ok" "$run_dir"
      domain_pass=$((domain_pass + 1))
    else
      echo "    FAIL: $reason"
      write_result "$domain" "$run" "fail" "$reason" "$run_dir"
      tail -n 80 "$dora_log" > "$run_dir/dora-tail.txt" || true
      domain_fail=$((domain_fail + 1))
    fi
  done

  total_pass=$((total_pass + domain_pass))
  total_fail=$((total_fail + domain_fail))
  echo "- pass=$domain_pass fail=$domain_fail" >> "$SUMMARY_MD"
done

{
  echo
  echo "## Summary"
  echo
  echo "- pass=$total_pass"
  echo "- fail=$total_fail"
  echo "- jsonl=$SUMMARY_JSONL"
} >> "$SUMMARY_MD"

echo
echo "summary: $SUMMARY_MD"
echo "jsonl: $SUMMARY_JSONL"

if [[ "$total_fail" -gt 0 ]]; then
  exit 1
fi
