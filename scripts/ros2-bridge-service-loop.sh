#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

DOMAINS="default"
RUNS=10
STOP_AFTER=25
TIMEOUT=55
SERVICE="/add_two_ints"
SERVICE_TYPE="example_interfaces/srv/AddTwoInts"
DATAFLOW="examples/ros2-bridge/yaml-bridge-service/dataflow-client.yml"
SERVER_PACKAGE="examples_rclcpp_minimal_service"
SERVER_EXECUTABLE="service_main"
ARTIFACT_ROOT="$ROOT/target/ros2-bridge-service-loop/$(date -u +%Y%m%dT%H%M%SZ)"
DORA_BIN="${DORA_BIN:-dora}"

usage() {
  cat <<'USAGE'
Usage: scripts/ros2-bridge-service-loop.sh [OPTIONS]

Repeatedly validates the YAML ROS2 service-client bridge against a real
rclcpp AddTwoInts service server. This is a local ROS2/Humble flake harness,
not part of the normal smoke suite.

Options:
  --domains LIST       Comma-separated domains. Use "default" to unset
                       ROS_DOMAIN_ID. Example: default,23,24 (default: default)
  --runs N             Runs per domain (default: 10)
  --stop-after SECS    dora run --stop-after value in seconds (default: 25)
  --timeout SECS       Outer timeout for each dora run (default: 55)
  --artifacts DIR      Directory for logs and summary files
                       (default: target/ros2-bridge-service-loop/<timestamp>)
  -h, --help           Show this help

Examples:
  source /opt/ros/humble/setup.bash
  scripts/ros2-bridge-service-loop.sh --domains default,23 --runs 30

Environment:
  DORA_BIN             dora executable to run (default: dora from PATH)
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

if [[ "${DORA_ROS2_SERVICE_LOOP_ALLOW_EXISTING_SERVER:-0}" != "1" ]]; then
  existing_servers="$(pgrep -af "/examples_rclcpp_minimal_service/service_main" || true)"
  if [[ -n "$existing_servers" ]]; then
    echo "existing ROS2 AddTwoInts service_main process found:" >&2
    echo "$existing_servers" >&2
    echo >&2
    echo "Stop it before running this harness, or set:" >&2
    echo "  DORA_ROS2_SERVICE_LOOP_ALLOW_EXISTING_SERVER=1" >&2
    echo >&2
    echo "The harness starts its own service server; duplicate servers can mask discovery flakes." >&2
    exit 2
  fi
fi

if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
  echo "AMENT_PREFIX_PATH is not set. Source ROS2 first, for example:" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
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

run_with_domain() {
  local domain="$1"
  shift
  if [[ "$domain" == "default" || "$domain" == "unset" ]]; then
    env -u ROS_DOMAIN_ID "$@"
  else
    env ROS_DOMAIN_ID="$domain" "$@"
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

wait_for_service() {
  local domain="$1" out_dir="$2"
  local service_list_log="$out_dir/ros2-service-list.txt"
  local deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    if run_with_domain "$domain" ros2 service list -t > "$service_list_log" 2>&1 \
      && grep -Fq "$SERVICE [$SERVICE_TYPE]" "$service_list_log"; then
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

echo "# ROS2 Bridge Service Loop" > "$SUMMARY_MD"
{
  echo
  echo "- Artifacts: $ARTIFACT_ROOT"
  echo "- Domains: $DOMAINS"
  echo "- Runs per domain: $RUNS"
  echo "- Dora stop-after: ${STOP_AFTER}s"
  echo "- Per-run timeout: ${TIMEOUT}s"
  echo "- Dataflow: $DATAFLOW"
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

  server_log="$domain_dir/server.log"
  run_with_domain "$domain" ros2 run "$SERVER_PACKAGE" "$SERVER_EXECUTABLE" \
    > "$server_log" 2>&1 &
  server_pid=$!
  server_pids+=("$server_pid")

  if ! wait_for_service "$domain" "$domain_dir"; then
    echo "  FAIL: service did not appear"
    write_result "$domain" 0 "fail" "service_not_visible" "$domain_dir"
    total_fail=$((total_fail + RUNS))
    kill "$server_pid" >/dev/null 2>&1 || true
    wait "$server_pid" >/dev/null 2>&1 || true
    echo "- service preflight failed; see \`$domain_dir\`" >> "$SUMMARY_MD"
    continue
  fi

  if ! run_with_domain "$domain" timeout 10s ros2 service call "$SERVICE" "$SERVICE_TYPE" "{a: 2, b: 40}" \
      > "$domain_dir/ros2-service-call.txt" 2>&1; then
    echo "  FAIL: ros2 service call failed"
    write_result "$domain" 0 "fail" "ros2_service_call_failed" "$domain_dir"
    total_fail=$((total_fail + RUNS))
    kill "$server_pid" >/dev/null 2>&1 || true
    wait "$server_pid" >/dev/null 2>&1 || true
    echo "- ros2 service call preflight failed; see \`$domain_dir\`" >> "$SUMMARY_MD"
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

    if [[ "$rc" -ne 0 ]]; then
      reason="dora_exit_$rc"
      if grep -Fq "ROS2 service not available" "$dora_log"; then
        reason="service_discovery_failed"
      elif grep -Fq "service response timed out" "$dora_log"; then
        reason="service_response_timed_out"
      fi
      echo "    FAIL: $reason"
      write_result "$domain" "$run" "fail" "$reason" "$run_dir"
      domain_fail=$((domain_fail + 1))
      total_fail=$((total_fail + 1))
      tail -80 "$dora_log" > "$run_dir/dora-tail.txt" || true
      continue
    fi

    if ! grep -Fq "ROS2 service connected:" "$dora_log"; then
      echo "    FAIL: missing_connected_marker"
      write_result "$domain" "$run" "fail" "missing_connected_marker" "$run_dir"
      domain_fail=$((domain_fail + 1))
      total_fail=$((total_fail + 1))
      continue
    fi

    if ! grep -Fq "Received response: sum =" "$dora_log"; then
      echo "    FAIL: missing_response"
      write_result "$domain" "$run" "fail" "missing_response" "$run_dir"
      domain_fail=$((domain_fail + 1))
      total_fail=$((total_fail + 1))
      continue
    fi

    if grep -Fq "sequence size exceeds remaining buffer" "$server_log"; then
      echo "    FAIL: server_sequence_buffer_error"
      write_result "$domain" "$run" "fail" "server_sequence_buffer_error" "$run_dir"
      domain_fail=$((domain_fail + 1))
      total_fail=$((total_fail + 1))
      continue
    fi

    echo "    PASS"
    write_result "$domain" "$run" "pass" "ok" "$run_dir"
    domain_pass=$((domain_pass + 1))
    total_pass=$((total_pass + 1))
  done

  kill "$server_pid" >/dev/null 2>&1 || true
  wait "$server_pid" >/dev/null 2>&1 || true
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

if [[ "$total_fail" -ne 0 ]]; then
  exit 1
fi
