# 2026-07-28 ROS2 Bridge 修复记录

## 本次实际修改

这次修复集中在 ROS2 Humble 下 YAML ROS2 bridge 的 service/action 互操作和诊断输出。

### 1. 修复 ROS_DOMAIN_ID 不生效

问题：

- `ros2 service call /add_two_ints ...` 在用户 shell 中可以访问 ROS2 service。
- `dora run examples/ros2-bridge/yaml-bridge-service/dataflow-client.yml --stop-after 45s` 之前会失败：
  - `Error: service not available after 10 retries`
- 设置 `ROS_DOMAIN_ID=23` 或 unset 后都能复现。

原因：

- `dora-ros2-bridge-node` 创建 `ros2_client::Context` 时固定使用默认 domain，没有读取进程环境里的 `ROS_DOMAIN_ID`。
- 这会让 Dora bridge 和外部 ROS2 节点不一定在同一个 DDS domain。

修改：

- `binaries/ros2-bridge-node/src/main.rs`
  - 新增 `parse_ros_domain_id`。
  - `create_ros_node` 改为从 `std::env::var("ROS_DOMAIN_ID")` 读取 domain。
  - 非法或未设置时回落到 `0`，保持 ROS2 默认行为。

验证：

- 新增单元测试 `ros_domain_id_parser_uses_env_value_and_defaults_to_zero`。
- 用户手工验证：
  - `ROS_DOMAIN_ID=23` 时 Dora service client 能持续收到 response。
  - unset `ROS_DOMAIN_ID` 时 Dora service client 也能持续收到 response。

### 2. 修复 service/action 默认 QoS 不匹配

问题：

- ROS2 CLI service call 能成功，但 Dora YAML service bridge 起不来，报 service unavailable。

原因：

- bridge 之前把 topic 风格的默认 QoS 用在 service/action discovery/request 路径上。
- service/action 在 ROS2 生态里通常需要 Reliable + KeepLast(10) 的默认语义；BestEffort 风格配置会导致 discovery 或匹配行为不稳定。

修改：

- `binaries/ros2-bridge-node/src/main.rs`
  - 新增 `default_service_qos_config`。
  - 新增 `service_or_action_qos`。
  - service mode 使用 service/action 专用默认 QoS。
  - action mode 使用 service/action 专用默认 QoS。
  - 保留显式 YAML QoS 配置的覆盖能力。

验证：

- 新增单元测试 `service_default_qos_is_reliable_without_changing_topic_default_detection`。
- 用户手工验证 service bridge 已经通过：
  - 请求 `1 + 10` 到 `20 + 200` 均收到正确 sum。

### 3. 改进 action client discovery 诊断

问题：

- action bridge 失败时，原始错误容易停在笼统的内部错误或等待超时，定位不到具体 ROS2 action endpoint。

修改：

- `binaries/ros2-bridge-node/src/main.rs`
  - 新增 `wait_for_action_goal_service`。
  - Dora action client 在发送 goal 前等待 `/<action>/_action/send_goal` 可发现。
  - 新增 `action_goal_service_unavailable_message`，错误信息明确提示 action name、goal service endpoint、ROS_DOMAIN_ID、RMW_IMPLEMENTATION 和可用 action 检查方向。

验证：

- 新增单元测试 `action_goal_service_unavailable_message_names_action_and_discovery_endpoint`。

### 4. 修复 Humble 下 `sequence size exceeds remaining buffer`

问题：

- service bridge 已经能正常工作后，ROS2 C++ service server 仍偶尔打印：

```text
sequence size exceeds remaining buffer
```

定位：

- 该字符串来自 ROS2 Humble 的 FastRTPS typesupport 模板：
  - `/opt/ros/humble/share/rosidl_typesupport_fastrtps_cpp/resource/msg__type_support.cpp.em`
- 触发点是 CDR 反序列化 sequence 时，读取到的 sequence size 超过剩余 buffer。
- 对照 Humble 消息定义：
  - `/opt/ros/humble/share/rmw_dds_common/msg/Gid.msg` 是 `char[24] data`。
- `ros2-client 0.8.1` 默认按 Iron 之后的 16-byte GID 编译；其 `pre-iron-gid` feature 才使用 Humble 需要的 24-byte GID。

原因：

- Dora ROS2 bridge 在 Humble 环境下没有启用 `ros2-client/pre-iron-gid`。
- bridge 发出的 ROS discovery 信息里 GID 长度与 Humble `rmw_dds_common` 类型定义不一致，导致 ROS2/FastRTPS 反序列化 discovery sample 时读错 sequence 长度。
- 这不一定破坏 `/add_two_ints` request/response 本身，但 discovery 数据格式确实不兼容，所以属于 bridge 的 Humble 兼容 bug。

修改：

- `libraries/extensions/ros2-bridge/Cargo.toml`
  - 默认 feature 加入 `pre-iron-gid`。
  - 新增 feature 映射：`pre-iron-gid = ["ros2-client/pre-iron-gid"]`。
- `binaries/ros2-bridge-node/Cargo.toml`
  - bridge node 关闭默认 feature 时，显式启用 `pre-iron-gid` 和 `rmw-zenoh`。
- `libraries/extensions/ros2-bridge/src/transport/dds.rs`
  - 新增回归测试 `default_ros2_bridge_build_uses_humble_compatible_gid_length`，防止默认构建退回 Iron 16-byte GID。

验证：

- 先写 failing test，未启用 feature 时测试失败。
- 启用 feature 后测试通过。
- 重新安装用户实际运行的二进制：

```bash
cargo install --path binaries/ros2-bridge-node --locked
```

### 5. 已运行验证命令

```bash
cargo test -p dora-ros2-bridge default_ros2_bridge_build_uses_humble_compatible_gid_length
cargo test -p dora-ros2-bridge-node peer_failure_tests
cargo build -p dora-ros2-bridge-node
cargo fmt --all -- --check
cargo install --path binaries/ros2-bridge-node --locked
```

结果：

- `dora-ros2-bridge` Humble GID 回归测试通过。
- `dora-ros2-bridge-node` 4 个 peer failure/domain/QoS/action diagnostic 测试通过。
- `dora-ros2-bridge-node` 构建通过。
- `cargo fmt --all -- --check` 通过。
- `/home/dora/.cargo/bin/dora-ros2-bridge-node` 已替换为本次修复后的版本。

### 6. 用户复测建议

终端 1：

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=23
ros2 run examples_rclcpp_minimal_service service_main
```

终端 2：

```bash
cd ~/Desktop/dora
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=23
dora run examples/ros2-bridge/yaml-bridge-service/dataflow-client.yml --stop-after 45s
```

预期：

- Dora requester 持续打印 `Received response: sum = ...`。
- ROS2 service server 只打印正常的 `request: ...`。
- 不再出现 `sequence size exceeds remaining buffer`。

---

# ROS2 Bridge Strict Diagnostics Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a strict, feedback-rich ROS2 bridge diagnostic harness that can reproduce and localize failures across topic, service, action, TurtleBot3, and Nav2 scenarios.

**Architecture:** Add a repo-local shell harness that runs each diagnostic case with timeouts, retries, ROS2 graph snapshots, Dora logs, and machine-readable result files. Add small Python probe nodes and YAML dataflows only where the existing examples do not provide enough observability, while reusing the existing `examples/ros2-bridge/yaml-bridge-*` service/action examples for baseline coverage. Use four modes: `preflight` for local build/package readiness, `smoke` for self-contained bridge cases with ROS2 example peers, `strict` for TurtleBot3 topic movement plus repeated service/action diagnostics, and `soak` for Nav2 integration.

**Tech Stack:** Bash, ROS2 CLI, Dora CLI, existing YAML ROS2 bridge, Python probe nodes using `dora-rs`/PyArrow, JSON Lines result records, Markdown summaries.

---

## File Structure

- Create: `scripts/ros2-bridge-strict.sh`
  - Main entry point. Runs preflight, selected cases, retries, timeouts, graph snapshots, log collection, and summary generation.
- Create: `tests/ros2-bridge-strict/README.md`
  - Operator-facing instructions for ROS2 Humble, TurtleBot3, Nav2, pass/fail criteria, and failure classification.
- Create: `tests/ros2-bridge-strict/dataflows/topic-odom-sub.yml`
  - Subscribes `/odom` from ROS2 into Dora and forwards it to a probe.
- Create: `tests/ros2-bridge-strict/dataflows/topic-cmd-vel-pub.yml`
  - Publishes `/cmd_vel` from Dora into ROS2 and records sent count.
- Create: `tests/ros2-bridge-strict/dataflows/nav2-navigate-to-pose.yml`
  - Sends `nav2_msgs/NavigateToPose` goals through the YAML action bridge.
- Create: `tests/ros2-bridge-strict/probes/odom_probe.py`
  - Counts `/odom` messages, records first-message latency and field presence.
- Create: `tests/ros2-bridge-strict/probes/cmd_vel_source.py`
  - Sends deterministic `geometry_msgs/Twist` Arrow structs at a fixed rate and records `/odom` motion evidence.
- Create: `tests/ros2-bridge-strict/probes/nav2_goal_sender.py`
  - Sends deterministic NavigateToPose goals and records feedback/result events.
- Create: `tests/ros2-bridge-strict/probes/result_utils.py`
  - Shared JSON Lines helper for probe events.

## Result Model

Every case writes one `result.json` with this shape:

```json
{
  "case": "service_dora_client_add_two_ints",
  "status": "pass",
  "classification": "PASS",
  "fail_at": null,
  "attempts": 3,
  "passed_attempts": 3,
  "duration_ms": 12450,
  "evidence_dir": "target/ros2-bridge-strict/2026-07-28T153000-domain23/service_dora_client_add_two_ints",
  "notes": []
}
```

Allowed classifications:

```text
PASS
FLAKY
TIMEOUT
ENV_FAIL
BRIDGE_FAIL
UNKNOWN
```

Allowed `fail_at` values:

```text
preflight
interface_lookup
ros2_peer_start
discovery
dora_start
serialization
send
receive
correlation
feedback_wait
result_wait
nav2_baseline
teardown
unknown
```

---

### Task 1: Add Harness Skeleton and Preflight Snapshots

**Files:**
- Create: `scripts/ros2-bridge-strict.sh`
- Create: `tests/ros2-bridge-strict/README.md`

- [ ] **Step 1: Create the executable script skeleton**

Create `scripts/ros2-bridge-strict.sh` with:

```bash
#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

MODE="${1:-preflight}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-23}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
RUN_ID="$(date -u +%Y-%m-%dT%H%M%SZ)-domain${ROS_DOMAIN_ID}"
ROOT="${PWD}/target/ros2-bridge-strict/${RUN_ID}"
SUMMARY_JSON="${ROOT}/summary.jsonl"
SUMMARY_MD="${ROOT}/summary.md"

export ROS_DOMAIN_ID
export RMW_IMPLEMENTATION
export RUST_BACKTRACE="${RUST_BACKTRACE:-1}"

mkdir -p "$ROOT"

log() {
  printf '[%s] %s\n' "$(date -u +%H:%M:%S)" "$*"
}

write_case_result() {
  local case="$1" status="$2" classification="$3" fail_at="$4" attempts="$5" passed="$6" duration_ms="$7" notes="$8"
  local dir="${ROOT}/${case}"
  mkdir -p "$dir"
  cat > "${dir}/result.json" <<JSON
{"case":"${case}","status":"${status}","classification":"${classification}","fail_at":${fail_at},"attempts":${attempts},"passed_attempts":${passed},"duration_ms":${duration_ms},"evidence_dir":"${dir}","notes":[${notes}]}
JSON
  cat "${dir}/result.json" >> "$SUMMARY_JSON"
}

snapshot_graph() {
  local out="$1"
  mkdir -p "$out"
  env | grep -E '^(ROS|RMW|AMENT|CYCLONEDDS|FASTRTPS|ZENOH|TURTLEBOT3)_' | sort > "${out}/env.txt" || true
  ros2 node list > "${out}/ros2-node-list.txt" 2> "${out}/ros2-node-list.err" || true
  ros2 topic list -t > "${out}/ros2-topic-list.txt" 2> "${out}/ros2-topic-list.err" || true
  ros2 service list -t > "${out}/ros2-service-list.txt" 2> "${out}/ros2-service-list.err" || true
  ros2 action list -t > "${out}/ros2-action-list.txt" 2> "${out}/ros2-action-list.err" || true
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "missing required command: $1" >&2
    exit 2
  }
}

preflight() {
  log "preflight: writing environment snapshot to ${ROOT}"
  require_cmd cargo
  require_cmd ros2
  require_cmd timeout
  require_cmd uv
  snapshot_graph "${ROOT}/preflight"
  {
    echo "# ROS2 Bridge Strict Diagnostics"
    echo
    echo "- Run ID: ${RUN_ID}"
    echo "- Mode: ${MODE}"
    echo "- ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
    echo "- RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION}"
    echo
  } > "$SUMMARY_MD"
}

generate_summary() {
  {
    echo
    echo "## Results"
    if [ -f "$SUMMARY_JSON" ]; then
      while IFS= read -r line; do
        echo "- \`${line}\`"
      done < "$SUMMARY_JSON"
    else
      echo "- No cases ran."
    fi
  } >> "$SUMMARY_MD"
  log "summary: ${SUMMARY_MD}"
}

main() {
  preflight
  case "$MODE" in
    preflight|smoke|strict|soak)
      log "mode ${MODE}: preflight-only skeleton is healthy"
      ;;
    *)
      echo "usage: $0 {preflight|smoke|strict|soak}" >&2
      exit 2
      ;;
  esac
  generate_summary
}

main "$@"
```

- [ ] **Step 2: Make the script executable**

Run:

```bash
chmod +x scripts/ros2-bridge-strict.sh
```

Expected: no output.

- [ ] **Step 3: Verify preflight output**

Run:

```bash
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh preflight
```

Expected:

```text
summary: target/ros2-bridge-strict/<run-id>/summary.md
```

- [ ] **Step 4: Document operator prerequisites**

Create `tests/ros2-bridge-strict/README.md` with:

```markdown
# ROS2 Bridge Strict Diagnostics

This directory contains manual strict diagnostics for the Dora ROS2 bridge.
The harness is intentionally evidence-heavy because service/action failures can
come from ROS2 discovery, message type lookup, Arrow conversion, request/goal
correlation, Nav2 lifecycle state, or upstream `ros2-client`/`rustdds`.

## Baseline Environment

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=23
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RUST_BACKTRACE=1
```

## Run Modes

```bash
scripts/ros2-bridge-strict.sh preflight
scripts/ros2-bridge-strict.sh smoke
scripts/ros2-bridge-strict.sh strict
scripts/ros2-bridge-strict.sh soak
```

`preflight` verifies local build and ROS2 package/interface readiness without
starting TurtleBot3 or Nav2. `smoke` runs one self-contained bridge service
client case and one action client case against ROS2 example peers. `strict`
adds TurtleBot3 topic diagnostics and repeats deterministic cases three times.
`soak` adds Nav2 `NavigateToPose` integration after a ROS2 CLI baseline passes.

## Result Classes

- `PASS`: all attempts passed.
- `FLAKY`: at least one attempt passed and at least one failed.
- `TIMEOUT`: the case exceeded its timeout.
- `ENV_FAIL`: ROS2, TurtleBot3, Nav2, or package prerequisites failed before Dora bridge behavior was tested.
- `BRIDGE_FAIL`: evidence shows the ROS2 peer was healthy and Dora bridge failed to send, receive, decode, or correlate.
- `UNKNOWN`: evidence is insufficient.
```

- [ ] **Step 5: Commit the skeleton**

Run:

```bash
git add scripts/ros2-bridge-strict.sh tests/ros2-bridge-strict/README.md
git commit -m "test: add ros2 bridge strict diagnostics skeleton"
```

Expected: commit succeeds.

---

### Task 2: Add Interface Availability and Cargo Baseline Cases

**Files:**
- Modify: `scripts/ros2-bridge-strict.sh`

- [ ] **Step 1: Add command runner helpers**

Insert after `require_cmd()`:

```bash
run_logged() {
  local case="$1" timeout_s="$2"
  shift 2
  local dir="${ROOT}/${case}"
  mkdir -p "$dir"
  printf '%q ' "$@" > "${dir}/command.txt"
  echo >> "${dir}/command.txt"
  snapshot_graph "${dir}/graph-before"
  local start end status
  start="$(date +%s%3N)"
  set +e
  timeout "${timeout_s}s" "$@" > "${dir}/stdout.log" 2> "${dir}/stderr.log"
  status="$?"
  set -e
  end="$(date +%s%3N)"
  snapshot_graph "${dir}/graph-after"
  echo "$status" > "${dir}/exit-code.txt"
  echo "$((end - start))" > "${dir}/duration-ms.txt"
  return "$status"
}

case_timeout_result() {
  local status="$1"
  if [ "$status" -eq 124 ]; then
    echo TIMEOUT
  else
    echo BRIDGE_FAIL
  fi
}
```

- [ ] **Step 2: Add baseline case function**

Insert before `generate_summary()`:

```bash
case_cargo_baseline() {
  local case="l0_cargo_ros2_bridge_baseline"
  local start end status duration classification
  start="$(date +%s%3N)"
  set +e
  run_logged "${case}" 900 bash -lc '
    set -euo pipefail
    cargo check -p dora-ros2-bridge --no-default-features
    cargo test -p dora-ros2-bridge-msg-gen
    basic_venv="$(mktemp -d -t dora-ros2-strict-venv-XXXXXX)"
    trap "rm -rf \"$basic_venv\"" EXIT
    uv venv --seed -p 3.12 "$basic_venv" >/dev/null
    source "$basic_venv/bin/activate"
    uv pip install --quiet pyarrow numpy
    cargo test -p dora-ros2-bridge-python
  '
  status="$?"
  set -e
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$status" -eq 0 ]; then
    write_case_result "$case" pass PASS null 1 1 "$duration" ""
  else
    classification="$(case_timeout_result "$status")"
    write_case_result "$case" fail "$classification" '"preflight"' 1 0 "$duration" '"cargo baseline failed"'
  fi
}
```

- [ ] **Step 3: Add ROS2 interface availability case function**

Insert after `case_cargo_baseline()`:

```bash
case_ros2_interface_availability() {
  local case="l1_ros2_interface_availability"
  local dir="${ROOT}/${case}"
  local start end status duration
  mkdir -p "$dir"
  start="$(date +%s%3N)"
  set +e
  run_logged "$case" 60 bash -lc '
    set -euo pipefail
    ros2 interface show nav_msgs/msg/Odometry >/dev/null
    ros2 interface show sensor_msgs/msg/LaserScan >/dev/null
    ros2 interface show geometry_msgs/msg/Twist >/dev/null
    ros2 interface show example_interfaces/srv/AddTwoInts >/dev/null
    ros2 interface show example_interfaces/action/Fibonacci >/dev/null
    ros2 interface show nav2_msgs/action/NavigateToPose >/dev/null
  '
  status="$?"
  set -e
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$status" -eq 0 ]; then
    write_case_result "$case" pass PASS null 1 1 "$duration" ""
  else
    write_case_result "$case" fail ENV_FAIL '"interface_lookup"' 1 0 "$duration" '"required ROS2 message/service/action interfaces are unavailable"'
  fi
}
```

- [ ] **Step 4: Wire baseline cases into `main()`**

Replace the `preflight|smoke|strict|soak)` branch body with:

```bash
      case_cargo_baseline
      case_ros2_interface_availability
      ;;
```

- [ ] **Step 5: Run the baseline cases**

Run:

```bash
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh preflight
```

Expected: `summary.jsonl` contains `l0_cargo_ros2_bridge_baseline` and `l1_ros2_interface_availability`.

- [ ] **Step 6: Commit baseline cases**

Run:

```bash
git add scripts/ros2-bridge-strict.sh
git commit -m "test: add ros2 bridge preflight diagnostics"
```

Expected: commit succeeds.

---

### Task 3: Add Topic Probe Dataflows

**Files:**
- Create: `tests/ros2-bridge-strict/probes/result_utils.py`
- Create: `tests/ros2-bridge-strict/probes/odom_probe.py`
- Create: `tests/ros2-bridge-strict/probes/cmd_vel_source.py`
- Create: `tests/ros2-bridge-strict/dataflows/topic-odom-sub.yml`
- Create: `tests/ros2-bridge-strict/dataflows/topic-cmd-vel-pub.yml`
- Modify: `scripts/ros2-bridge-strict.sh`

- [ ] **Step 1: Add probe result helper**

Create `tests/ros2-bridge-strict/probes/result_utils.py`:

```python
import json
import os
import time


def now_ms() -> int:
    return int(time.time() * 1000)


def result_path() -> str:
    path = os.environ.get("DORA_STRICT_RESULT_PATH")
    if not path:
        raise RuntimeError("DORA_STRICT_RESULT_PATH is not set")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    return path


def emit(event: dict) -> None:
    record = {"time_ms": now_ms(), **event}
    with open(result_path(), "a", encoding="utf-8") as f:
        f.write(json.dumps(record, sort_keys=True) + "\n")
```

- [ ] **Step 2: Add odom subscriber probe**

Create `tests/ros2-bridge-strict/probes/odom_probe.py`:

```python
import os
import time

from dora import Node

from result_utils import emit


node = Node()
start = time.time()
deadline = start + float(os.environ.get("DORA_STRICT_DURATION_S", "30"))
count = 0
first_latency_ms = None

emit({"event": "probe_start", "probe": "odom_probe"})

for event in node:
    if time.time() >= deadline:
        break
    if event["type"] == "INPUT" and event["id"] == "odom":
        count += 1
        if first_latency_ms is None:
            first_latency_ms = int((time.time() - start) * 1000)
        value = event["value"]
        fields = list(value.type)
        emit({
            "event": "odom_received",
            "count": count,
            "first_latency_ms": first_latency_ms,
            "fields": [field.name for field in fields],
        })
    elif event["type"] == "STOP":
        break

emit({
    "event": "probe_done",
    "probe": "odom_probe",
    "count": count,
    "first_latency_ms": first_latency_ms,
})
```

- [ ] **Step 3: Add cmd_vel publisher probe**

Create `tests/ros2-bridge-strict/probes/cmd_vel_source.py`:

```python
import os
import time
import math

import pyarrow as pa
from dora import Node

from result_utils import emit


def twist(linear_x: float, angular_z: float) -> pa.StructArray:
    vector_type = pa.struct([
        pa.field("x", pa.float64()),
        pa.field("y", pa.float64()),
        pa.field("z", pa.float64()),
    ])
    return pa.array([
        {
            "linear": {"x": linear_x, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": angular_z},
        }
    ], type=pa.struct([
        pa.field("linear", vector_type),
        pa.field("angular", vector_type),
    ]))


node = Node()
duration_s = float(os.environ.get("DORA_STRICT_DURATION_S", "10"))
deadline = time.time() + duration_s
sent = 0
start_xy = None
last_xy = None
motion_confirmed = False
emit({"event": "probe_start", "probe": "cmd_vel_source"})

while time.time() < deadline:
    event = node.try_recv()
    if event is not None:
        if event["type"] == "STOP":
            break
        if event["type"] == "INPUT" and event["id"] == "odom":
            odom = event["value"].to_pylist()[0]
            position = odom["pose"]["pose"]["position"]
            xy = (float(position["x"]), float(position["y"]))
            if start_xy is None:
                start_xy = xy
                emit({"event": "odom_start", "x": xy[0], "y": xy[1]})
            last_xy = xy
            distance = math.hypot(last_xy[0] - start_xy[0], last_xy[1] - start_xy[1])
            emit({"event": "odom_sample", "x": xy[0], "y": xy[1], "distance": distance})
            if distance >= 0.05 and not motion_confirmed:
                motion_confirmed = True
                emit({"event": "motion_confirmed", "distance": distance})
    node.send_output("cmd_vel", twist(0.15, 0.0))
    sent += 1
    emit({"event": "cmd_vel_sent", "count": sent})
    time.sleep(0.2)

emit({
    "event": "probe_done",
    "probe": "cmd_vel_source",
    "sent": sent,
    "motion_confirmed": motion_confirmed,
    "start_xy": start_xy,
    "last_xy": last_xy,
})
```

- [ ] **Step 4: Add odom dataflow**

Create `tests/ros2-bridge-strict/dataflows/topic-odom-sub.yml`:

```yaml
nodes:
  - id: odom_bridge
    ros2:
      topic: /odom
      message_type: nav_msgs/Odometry
      direction: subscribe
    outputs:
      - odom

  - id: odom_probe
    path: ../probes/odom_probe.py
    inputs:
      odom: odom_bridge/odom
```

- [ ] **Step 5: Add cmd_vel dataflow**

Create `tests/ros2-bridge-strict/dataflows/topic-cmd-vel-pub.yml`:

```yaml
nodes:
  - id: cmd_vel_source
    path: ../probes/cmd_vel_source.py
    inputs:
      odom: odom_bridge/odom
    outputs:
      - cmd_vel

  - id: odom_bridge
    ros2:
      topic: /odom
      message_type: nav_msgs/Odometry
      direction: subscribe
    outputs:
      - odom

  - id: cmd_bridge
    ros2:
      topic: /cmd_vel
      message_type: geometry_msgs/Twist
      direction: publish
    inputs:
      cmd_vel: cmd_vel_source/cmd_vel
```

- [ ] **Step 6: Add topic case functions**

Insert before `generate_summary()`:

```bash
case_topic_odom_sub() {
  local case="l2_topic_ros_to_dora_odom"
  local dir="${ROOT}/${case}"
  local start end status duration count attempts passed
  mkdir -p "$dir"
  start="$(date +%s%3N)"
  attempts=3
  passed=0
  for attempt in $(seq 1 "$attempts"); do
    mkdir -p "${dir}/attempt-${attempt}"
    set +e
    DORA_STRICT_RESULT_PATH="${dir}/attempt-${attempt}/probe-events.jsonl" DORA_STRICT_DURATION_S=30 \
      timeout 45s bash -lc 'cd tests/ros2-bridge-strict/dataflows && dora run topic-odom-sub.yml --uv --stop-after 35s' > "${dir}/attempt-${attempt}/dora.stdout.log" 2> "${dir}/attempt-${attempt}/dora.stderr.log"
    status="$?"
    set -e
    count="$(grep -c '"event": "odom_received"' "${dir}/attempt-${attempt}/probe-events.jsonl" 2>/dev/null || true)"
    if [ "$status" -eq 0 ] && [ "${count:-0}" -ge 30 ]; then
      passed=$((passed + 1))
    fi
  done
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$passed" -eq "$attempts" ]; then
    write_case_result "$case" pass PASS null "$attempts" "$passed" "$duration" ""
  elif [ "$passed" -gt 0 ]; then
    write_case_result "$case" fail FLAKY '"receive"' "$attempts" "$passed" "$duration" "\"${passed}/${attempts} odom subscribe attempts received at least 30 messages\""
  else
    write_case_result "$case" fail BRIDGE_FAIL '"receive"' "$attempts" 0 "$duration" '"no odom subscribe attempt received at least 30 messages"'
  fi
}

case_topic_cmd_vel_pub() {
  local case="l2_topic_dora_to_ros_cmd_vel"
  local dir="${ROOT}/${case}"
  local start end status duration sent observed moved attempts passed
  mkdir -p "$dir"
  start="$(date +%s%3N)"
  attempts=3
  passed=0
  for attempt in $(seq 1 "$attempts"); do
    mkdir -p "${dir}/attempt-${attempt}"
    set +e
    timeout 20s ros2 topic echo /cmd_vel > "${dir}/attempt-${attempt}/ros2-cmd-vel-echo.log" 2> "${dir}/attempt-${attempt}/ros2-cmd-vel-echo.err" &
    local echo_pid=$!
    DORA_STRICT_RESULT_PATH="${dir}/attempt-${attempt}/probe-events.jsonl" DORA_STRICT_DURATION_S=10 \
      timeout 30s bash -lc 'cd tests/ros2-bridge-strict/dataflows && dora run topic-cmd-vel-pub.yml --uv --stop-after 12s' > "${dir}/attempt-${attempt}/dora.stdout.log" 2> "${dir}/attempt-${attempt}/dora.stderr.log"
    status="$?"
    wait "$echo_pid" || true
    set -e
    sent="$(grep -c '"event": "cmd_vel_sent"' "${dir}/attempt-${attempt}/probe-events.jsonl" 2>/dev/null || true)"
    observed="$(grep -c 'linear:' "${dir}/attempt-${attempt}/ros2-cmd-vel-echo.log" 2>/dev/null || true)"
    moved="$(grep -c '"event": "motion_confirmed"' "${dir}/attempt-${attempt}/probe-events.jsonl" 2>/dev/null || true)"
    if [ "$status" -eq 0 ] && [ "${sent:-0}" -ge 20 ] && [ "${observed:-0}" -ge 5 ] && [ "${moved:-0}" -ge 1 ]; then
      passed=$((passed + 1))
    fi
  done
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$passed" -eq "$attempts" ]; then
    write_case_result "$case" pass PASS null "$attempts" "$passed" "$duration" ""
  elif [ "$passed" -gt 0 ]; then
    write_case_result "$case" fail FLAKY '"send"' "$attempts" "$passed" "$duration" "\"${passed}/${attempts} cmd_vel attempts produced ROS2 echo and odom motion\""
  else
    write_case_result "$case" fail BRIDGE_FAIL '"send"' "$attempts" 0 "$duration" '"no cmd_vel attempt produced ROS2 echo plus at least 0.05m odom motion"'
  fi
}
```

- [ ] **Step 7: Wire topic cases into strict and soak modes**

In `main()`, after baseline cases, add:

```bash
      if [ "$MODE" = "strict" ] || [ "$MODE" = "soak" ]; then
        case_topic_odom_sub
        case_topic_cmd_vel_pub
      fi
```

- [ ] **Step 8: Run topic cases against an already running TurtleBot3 Gazebo**

Run:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh strict
```

Expected: topic cases pass when `/odom` is publishing, ROS2 observes `/cmd_vel`, and TurtleBot3 odometry moves at least 0.05m while Dora publishes commands.

- [ ] **Step 9: Commit topic diagnostics**

Run:

```bash
git add scripts/ros2-bridge-strict.sh tests/ros2-bridge-strict
git commit -m "test: add ros2 bridge topic diagnostics"
```

Expected: commit succeeds.

---

### Task 4: Add Service Diagnostics with Repetition and Correlation Evidence

**Files:**
- Modify: `scripts/ros2-bridge-strict.sh`
- Modify: `tests/ros2-bridge-strict/README.md`

- [ ] **Step 1: Add service client case**

Insert before `generate_summary()`:

```bash
case_service_dora_client_add_two_ints() {
  local case="l3_service_dora_client_add_two_ints_10x"
  local dir="${ROOT}/${case}"
  local start end duration passed responses status
  mkdir -p "$dir"
  start="$(date +%s%3N)"
  passed=0
  for attempt in 1 2 3; do
    mkdir -p "${dir}/attempt-${attempt}"
    set +e
    timeout 90s ros2 run examples_rclcpp_minimal_service service_main > "${dir}/attempt-${attempt}/ros2-service-server.log" 2> "${dir}/attempt-${attempt}/ros2-service-server.err" &
    local peer_pid=$!
    sleep 3
    timeout 45s bash -lc 'cd examples/ros2-bridge/yaml-bridge-service && dora run dataflow-client.yml --stop-after 12s' > "${dir}/attempt-${attempt}/dora.stdout.log" 2> "${dir}/attempt-${attempt}/dora.stderr.log"
    status="$?"
    kill "$peer_pid" >/dev/null 2>&1 || true
    wait "$peer_pid" >/dev/null 2>&1 || true
    set -e
    responses="$(grep -c 'Received response: sum =' "${dir}/attempt-${attempt}/dora.stdout.log" 2>/dev/null || true)"
    if [ "$status" -eq 0 ] && [ "${responses:-0}" -ge 10 ]; then
      passed=$((passed + 1))
    fi
  done
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$passed" -eq 3 ]; then
    write_case_result "$case" pass PASS null 3 3 "$duration" ""
  elif [ "$passed" -gt 0 ]; then
    write_case_result "$case" fail FLAKY '"correlation"' 3 "$passed" "$duration" "\"${passed}/3 service client attempts produced at least 10 responses\""
  else
    write_case_result "$case" fail BRIDGE_FAIL '"correlation"' 3 0 "$duration" '"no service client attempt produced 10 AddTwoInts responses"'
  fi
}
```

- [ ] **Step 2: Add service server case**

Insert after `case_service_dora_client_add_two_ints()`:

```bash
case_service_dora_server_add_two_ints() {
  local case="l3_service_dora_server_add_two_ints_10x"
  local dir="${ROOT}/${case}"
  local start end duration passed success
  mkdir -p "$dir"
  start="$(date +%s%3N)"
  passed=0
  for attempt in 1 2 3; do
    mkdir -p "${dir}/attempt-${attempt}"
    set +e
    bash -lc 'cd examples/ros2-bridge/yaml-bridge-service && dora run dataflow-server.yml' > "${dir}/attempt-${attempt}/dora-server.stdout.log" 2> "${dir}/attempt-${attempt}/dora-server.stderr.log" &
    local dora_pid=$!
    sleep 5
    success=0
    for i in $(seq 1 10); do
      if timeout 20s ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: ${i}, b: $((i * 10))}" > "${dir}/attempt-${attempt}/ros2-call-${i}.log" 2> "${dir}/attempt-${attempt}/ros2-call-${i}.err"; then
        if grep -q "sum=$((i + i * 10))\\|sum: $((i + i * 10))" "${dir}/attempt-${attempt}/ros2-call-${i}.log"; then
          success=$((success + 1))
        fi
      fi
    done
    kill "$dora_pid" >/dev/null 2>&1 || true
    wait "$dora_pid" >/dev/null 2>&1 || true
    set -e
    if [ "$success" -eq 10 ]; then
      passed=$((passed + 1))
    fi
  done
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$passed" -eq 3 ]; then
    write_case_result "$case" pass PASS null 3 3 "$duration" ""
  elif [ "$passed" -gt 0 ]; then
    write_case_result "$case" fail FLAKY '"correlation"' 3 "$passed" "$duration" "\"${passed}/3 service server attempts handled 10 calls\""
  else
    write_case_result "$case" fail BRIDGE_FAIL '"discovery"' 3 0 "$duration" '"no service server attempt handled all 10 ROS2 service calls"'
  fi
}
```

- [ ] **Step 3: Wire service cases into smoke, strict, and soak modes**

In `main()`, after topic cases, add:

```bash
      if [ "$MODE" = "smoke" ] || [ "$MODE" = "strict" ] || [ "$MODE" = "soak" ]; then
        case_service_dora_client_add_two_ints
        case_service_dora_server_add_two_ints
      fi
```

- [ ] **Step 4: Document service failure attribution**

Append to `tests/ros2-bridge-strict/README.md`:

```markdown
## Service Failure Attribution

- If ROS2 cannot see `/add_two_ints`, classify as `ENV_FAIL` or `discovery`.
- If the Dora handler receives a request but ROS2 client times out, inspect
  metadata passthrough. The YAML service handler must return the same
  `request_id` metadata that came from the bridge.
- If sequential calls pass but repeated calls fail, classify as `correlation`
  and inspect pending request eviction and response matching.
```

- [ ] **Step 5: Run service diagnostics**

Run:

```bash
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh strict
```

Expected: service cases report `PASS` or produce evidence under their case directory.

- [ ] **Step 6: Commit service diagnostics**

Run:

```bash
git add scripts/ros2-bridge-strict.sh tests/ros2-bridge-strict/README.md
git commit -m "test: add ros2 bridge service diagnostics"
```

Expected: commit succeeds.

---

### Task 5: Add Action Diagnostics with Feedback/Result Attribution

**Files:**
- Modify: `scripts/ros2-bridge-strict.sh`
- Modify: `tests/ros2-bridge-strict/README.md`

- [ ] **Step 1: Add Dora action client case**

Insert before `generate_summary()`:

```bash
case_action_dora_client_fibonacci() {
  local case="l4_action_dora_client_fibonacci_3x"
  local dir="${ROOT}/${case}"
  local start end duration passed feedback result
  mkdir -p "$dir"
  start="$(date +%s%3N)"
  passed=0
  for attempt in 1 2 3; do
    mkdir -p "${dir}/attempt-${attempt}"
    set +e
    timeout 120s ros2 run examples_rclcpp_action_server fibonacci_action_server > "${dir}/attempt-${attempt}/ros2-action-server.log" 2> "${dir}/attempt-${attempt}/ros2-action-server.err" &
    local peer_pid=$!
    sleep 5
    timeout 90s bash -lc 'cd examples/ros2-bridge/yaml-bridge-action && dora run dataflow.yml --stop-after 20s' > "${dir}/attempt-${attempt}/dora.stdout.log" 2> "${dir}/attempt-${attempt}/dora.stderr.log"
    local status="$?"
    kill "$peer_pid" >/dev/null 2>&1 || true
    wait "$peer_pid" >/dev/null 2>&1 || true
    set -e
    feedback="$(grep -c 'Feedback: partial_sequence=' "${dir}/attempt-${attempt}/dora.stdout.log" 2>/dev/null || true)"
    result="$(grep -c 'Result: sequence=' "${dir}/attempt-${attempt}/dora.stdout.log" 2>/dev/null || true)"
    if [ "$status" -eq 0 ] && [ "${feedback:-0}" -ge 1 ] && [ "${result:-0}" -ge 1 ]; then
      passed=$((passed + 1))
    fi
  done
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$passed" -eq 3 ]; then
    write_case_result "$case" pass PASS null 3 3 "$duration" ""
  elif [ "$passed" -gt 0 ]; then
    write_case_result "$case" fail FLAKY '"result_wait"' 3 "$passed" "$duration" "\"${passed}/3 action client attempts passed\""
  else
    write_case_result "$case" fail BRIDGE_FAIL '"result_wait"' 3 0 "$duration" '"no action client attempts produced feedback and result"'
  fi
}
```

- [ ] **Step 2: Add Dora action server case**

Insert after `case_action_dora_client_fibonacci()`:

```bash
case_action_dora_server_fibonacci() {
  local case="l4_action_dora_server_fibonacci_3x"
  local dir="${ROOT}/${case}"
  local start end duration passed
  mkdir -p "$dir"
  start="$(date +%s%3N)"
  passed=0
  for attempt in 1 2 3; do
    mkdir -p "${dir}/attempt-${attempt}"
    set +e
    bash -lc 'cd examples/ros2-bridge/yaml-bridge-action-server && dora run dataflow.yml' > "${dir}/attempt-${attempt}/dora.stdout.log" 2> "${dir}/attempt-${attempt}/dora.stderr.log" &
    local dora_pid=$!
    sleep 5
    timeout 40s ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 10}" --feedback > "${dir}/attempt-${attempt}/ros2-action-client.log" 2> "${dir}/attempt-${attempt}/ros2-action-client.err"
    local status="$?"
    kill "$dora_pid" >/dev/null 2>&1 || true
    wait "$dora_pid" >/dev/null 2>&1 || true
    set -e
    if [ "$status" -eq 0 ] && grep -q 'sequence' "${dir}/attempt-${attempt}/ros2-action-client.log"; then
      passed=$((passed + 1))
    fi
  done
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$passed" -eq 3 ]; then
    write_case_result "$case" pass PASS null 3 3 "$duration" ""
  elif [ "$passed" -gt 0 ]; then
    write_case_result "$case" fail FLAKY '"result_wait"' 3 "$passed" "$duration" "\"${passed}/3 action server attempts passed\""
  else
    write_case_result "$case" fail BRIDGE_FAIL '"discovery"' 3 0 "$duration" '"no ROS2 action client attempts completed"'
  fi
}
```

- [ ] **Step 3: Wire action cases into smoke, strict, and soak modes**

In `main()`, after service cases, add:

```bash
      if [ "$MODE" = "smoke" ] || [ "$MODE" = "strict" ] || [ "$MODE" = "soak" ]; then
        case_action_dora_client_fibonacci
        case_action_dora_server_fibonacci
      fi
```

- [ ] **Step 4: Document action failure attribution**

Append to `tests/ros2-bridge-strict/README.md`:

```markdown
## Action Failure Attribution

- `goal timeout`: action server was not ready, discovery failed, or the goal
  request channel failed.
- `feedback observed, result missing`: likely result channel or deferred
  `get_result` behavior. This is a known unstable area in upstream
  `ros2-client`/`rustdds`; preserve logs and classify separately from topic and
  service failures.
- `Dora action server not discoverable by ROS2 CLI`: keep this separate from
  action client failures. The repository documents upstream discovery
  limitations for Dora-hosted action servers.
- `goal_id` metadata must be preserved on feedback and result messages.
```

- [ ] **Step 5: Run action diagnostics**

Run:

```bash
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh strict
```

Expected: action cases either pass or classify failure at `discovery`, `feedback_wait`, or `result_wait`.

- [ ] **Step 6: Commit action diagnostics**

Run:

```bash
git add scripts/ros2-bridge-strict.sh tests/ros2-bridge-strict/README.md
git commit -m "test: add ros2 bridge action diagnostics"
```

Expected: commit succeeds.

---

### Task 6: Add Nav2 NavigateToPose Diagnostics

**Files:**
- Create: `tests/ros2-bridge-strict/probes/nav2_goal_sender.py`
- Create: `tests/ros2-bridge-strict/dataflows/nav2-navigate-to-pose.yml`
- Modify: `scripts/ros2-bridge-strict.sh`
- Modify: `tests/ros2-bridge-strict/README.md`

- [ ] **Step 1: Add Nav2 goal sender probe**

Create `tests/ros2-bridge-strict/probes/nav2_goal_sender.py`:

```python
import os
import time
import math

import pyarrow as pa
from dora import Node

from result_utils import emit


def pose_stamped_goal(x: float, y: float) -> pa.StructArray:
    stamp_type = pa.struct([
        pa.field("sec", pa.int32()),
        pa.field("nanosec", pa.uint32()),
    ])
    header_type = pa.struct([
        pa.field("stamp", stamp_type),
        pa.field("frame_id", pa.string()),
    ])
    point_type = pa.struct([
        pa.field("x", pa.float64()),
        pa.field("y", pa.float64()),
        pa.field("z", pa.float64()),
    ])
    quat_type = pa.struct([
        pa.field("x", pa.float64()),
        pa.field("y", pa.float64()),
        pa.field("z", pa.float64()),
        pa.field("w", pa.float64()),
    ])
    pose_type = pa.struct([
        pa.field("position", point_type),
        pa.field("orientation", quat_type),
    ])
    pose_stamped_type = pa.struct([
        pa.field("header", header_type),
        pa.field("pose", pose_type),
    ])
    goal_type = pa.struct([
        pa.field("pose", pose_stamped_type),
        pa.field("behavior_tree", pa.string()),
    ])
    return pa.array([
        {
            "pose": {
                "header": {
                    "stamp": {"sec": 0, "nanosec": 0},
                    "frame_id": "map",
                },
                "pose": {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            },
            "behavior_tree": "",
        }
    ], type=goal_type)


node = Node()
sent = False
start = time.time()
deadline = start + float(os.environ.get("DORA_STRICT_DURATION_S", "120"))
target_x = float(os.environ.get("DORA_STRICT_NAV2_X", "1.0"))
target_y = float(os.environ.get("DORA_STRICT_NAV2_Y", "0.0"))
last_xy = None
result_seen = False
goal_reached = False

emit({"event": "probe_start", "probe": "nav2_goal_sender", "target_x": target_x, "target_y": target_y})

while time.time() < deadline:
    event = node.try_recv()
    if not sent:
        node.send_output("goal", pose_stamped_goal(target_x, target_y))
        sent = True
        emit({"event": "nav2_goal_sent", "target_x": target_x, "target_y": target_y})
    if event is not None and event["type"] == "INPUT":
        if event["id"] == "odom":
            odom = event["value"].to_pylist()[0]
            position = odom["pose"]["pose"]["position"]
            last_xy = (float(position["x"]), float(position["y"]))
            distance = math.hypot(last_xy[0] - target_x, last_xy[1] - target_y)
            emit({"event": "nav2_odom_sample", "x": last_xy[0], "y": last_xy[1], "distance_to_target": distance})
            if result_seen and distance <= 0.5 and not goal_reached:
                goal_reached = True
                emit({"event": "nav2_goal_reached", "distance_to_target": distance})
                break
            continue
        emit({"event": f"nav2_{event['id']}_received", "value": event["value"].to_pylist()})
        if event["id"] == "result":
            result_seen = True
            if last_xy is not None:
                distance = math.hypot(last_xy[0] - target_x, last_xy[1] - target_y)
                if distance <= 0.5:
                    goal_reached = True
                    emit({"event": "nav2_goal_reached", "distance_to_target": distance})
                    break
    elif event is not None and event["type"] == "STOP":
        break
    time.sleep(0.1)

emit({
    "event": "probe_done",
    "probe": "nav2_goal_sender",
    "sent": sent,
    "result_seen": result_seen,
    "goal_reached": goal_reached,
    "last_xy": last_xy,
})
```

- [ ] **Step 2: Add Nav2 dataflow**

Create `tests/ros2-bridge-strict/dataflows/nav2-navigate-to-pose.yml`:

```yaml
nodes:
  - id: nav2_goal_sender
    path: ../probes/nav2_goal_sender.py
    inputs:
      feedback: nav2_client/feedback
      result: nav2_client/result
      odom: odom_bridge/odom
    outputs:
      - goal

  - id: odom_bridge
    ros2:
      topic: /odom
      message_type: nav_msgs/Odometry
      direction: subscribe
    outputs:
      - odom

  - id: nav2_client
    ros2:
      action: /navigate_to_pose
      action_type: nav2_msgs/NavigateToPose
      role: client
    inputs:
      goal: nav2_goal_sender/goal
    outputs:
      - feedback
      - result
```

- [ ] **Step 3: Add Nav2 preflight helper**

Insert before `generate_summary()`:

```bash
nav2_preflight() {
  local dir="$1"
  mkdir -p "$dir"
  ros2 action list -t > "${dir}/action-list.txt" 2> "${dir}/action-list.err" || return 1
  grep -q '/navigate_to_pose.*nav2_msgs/action/NavigateToPose' "${dir}/action-list.txt" || return 1
  ros2 lifecycle get /bt_navigator > "${dir}/bt-navigator.lifecycle" 2>&1 || return 1
  ros2 lifecycle get /controller_server > "${dir}/controller-server.lifecycle" 2>&1 || return 1
  ros2 lifecycle get /planner_server > "${dir}/planner-server.lifecycle" 2>&1 || return 1
  grep -q active "${dir}/bt-navigator.lifecycle" || return 1
  grep -q active "${dir}/controller-server.lifecycle" || return 1
  grep -q active "${dir}/planner-server.lifecycle" || return 1
}
```

- [ ] **Step 4: Add Nav2 CLI baseline helper**

Insert after `nav2_preflight()`:

```bash
nav2_cli_baseline() {
  local dir="$1"
  mkdir -p "$dir"
  timeout 120s ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
    --feedback > "${dir}/ros2-cli-navigate.log" 2> "${dir}/ros2-cli-navigate.err"
}
```

- [ ] **Step 5: Add Nav2 case**

Insert after `nav2_cli_baseline()`:

```bash
case_nav2_navigate_to_pose() {
  local case="l5_nav2_navigate_to_pose_3x"
  local dir="${ROOT}/${case}"
  local start end duration passed feedback result
  mkdir -p "$dir"
  start="$(date +%s%3N)"
  if ! nav2_preflight "${dir}/nav2-preflight"; then
    end="$(date +%s%3N)"
    duration="$((end - start))"
    write_case_result "$case" fail ENV_FAIL '"nav2_baseline"' 3 0 "$duration" '"Nav2 action or lifecycle preflight failed"'
    return 0
  fi
  if ! nav2_cli_baseline "${dir}/nav2-cli-baseline"; then
    end="$(date +%s%3N)"
    duration="$((end - start))"
    write_case_result "$case" fail ENV_FAIL '"nav2_baseline"' 3 0 "$duration" '"ROS2 CLI NavigateToPose baseline failed before Dora bridge was tested"'
    return 0
  fi
  passed=0
  for attempt in 1 2 3; do
    mkdir -p "${dir}/attempt-${attempt}"
    set +e
    DORA_STRICT_RESULT_PATH="${dir}/attempt-${attempt}/probe-events.jsonl" DORA_STRICT_DURATION_S=120 \
      timeout 150s bash -lc 'cd tests/ros2-bridge-strict/dataflows && dora run nav2-navigate-to-pose.yml --uv --stop-after 130s' > "${dir}/attempt-${attempt}/dora.stdout.log" 2> "${dir}/attempt-${attempt}/dora.stderr.log"
    local status="$?"
    set -e
    feedback="$(grep -c '"event": "nav2_feedback_received"' "${dir}/attempt-${attempt}/probe-events.jsonl" 2>/dev/null || true)"
    result="$(grep -c '"event": "nav2_result_received"' "${dir}/attempt-${attempt}/probe-events.jsonl" 2>/dev/null || true)"
    reached="$(grep -c '"event": "nav2_goal_reached"' "${dir}/attempt-${attempt}/probe-events.jsonl" 2>/dev/null || true)"
    if [ "$status" -eq 0 ] && [ "${feedback:-0}" -ge 1 ] && [ "${result:-0}" -ge 1 ] && [ "${reached:-0}" -ge 1 ]; then
      passed=$((passed + 1))
    fi
  done
  end="$(date +%s%3N)"
  duration="$((end - start))"
  if [ "$passed" -eq 3 ]; then
    write_case_result "$case" pass PASS null 3 3 "$duration" ""
  elif [ "$passed" -gt 0 ]; then
    write_case_result "$case" fail FLAKY '"result_wait"' 3 "$passed" "$duration" "\"${passed}/3 Nav2 goals passed\""
  else
    write_case_result "$case" fail BRIDGE_FAIL '"result_wait"' 3 0 "$duration" '"no Nav2 NavigateToPose attempts produced feedback, result, and odom within 0.5m of target through Dora bridge"'
  fi
}
```

- [ ] **Step 6: Wire Nav2 into soak mode only**

In `main()`, after action cases, add:

```bash
      if [ "$MODE" = "soak" ]; then
        case_nav2_navigate_to_pose
      fi
```

- [ ] **Step 7: Document Nav2 prerequisites**

Append to `tests/ros2-bridge-strict/README.md`:

```markdown
## Nav2 Requirements

Before running `soak`, verify Nav2 with the ROS2 CLI:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

If this CLI baseline fails, the harness classifies Nav2 as `ENV_FAIL` and does
not attribute the failure to Dora bridge.
```

- [ ] **Step 8: Run Nav2 diagnostics**

Run TurtleBot3 Gazebo and Nav2 in separate terminals, then:

```bash
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh soak
```

Expected: Nav2 case either passes three Dora goals with feedback, result, and final odom within 0.5m of the target, or classifies the failure at `nav2_baseline` or `result_wait`.

- [ ] **Step 9: Commit Nav2 diagnostics**

Run:

```bash
git add scripts/ros2-bridge-strict.sh tests/ros2-bridge-strict
git commit -m "test: add nav2 action diagnostics"
```

Expected: commit succeeds.

---

### Task 7: Improve Summary Reporting and Exit Codes

**Files:**
- Modify: `scripts/ros2-bridge-strict.sh`

- [ ] **Step 1: Make summary human-readable**

Replace `generate_summary()` with:

```bash
generate_summary() {
  {
    echo
    echo "## Results"
    if [ -f "$SUMMARY_JSON" ]; then
      while IFS= read -r line; do
        case_name="$(printf '%s' "$line" | sed -n 's/.*"case":"\([^"]*\)".*/\1/p')"
        classification="$(printf '%s' "$line" | sed -n 's/.*"classification":"\([^"]*\)".*/\1/p')"
        fail_at="$(printf '%s' "$line" | sed -n 's/.*"fail_at":\("[^"]*"\|null\).*/\1/p')"
        echo "- ${classification}: \`${case_name}\` fail_at=${fail_at}"
      done < "$SUMMARY_JSON"
    else
      echo "- No cases ran."
    fi
  } >> "$SUMMARY_MD"
  log "summary: ${SUMMARY_MD}"
}
```

- [ ] **Step 2: Add failing exit code only for strict evidence**

Insert after `generate_summary` call in `main()`:

```bash
  if [ -f "$SUMMARY_JSON" ] && grep -q '"classification":"BRIDGE_FAIL"\|"classification":"TIMEOUT"' "$SUMMARY_JSON"; then
    exit 1
  fi
```

- [ ] **Step 3: Verify summary behavior**

Run:

```bash
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh preflight
```

Expected: exit 0 when preflight cases pass. If a bridge timeout or bridge failure occurs in `smoke`, `strict`, or `soak`, exit 1 and leave evidence in `target/ros2-bridge-strict/<run-id>/`.

- [ ] **Step 4: Commit summary improvements**

Run:

```bash
git add scripts/ros2-bridge-strict.sh
git commit -m "test: summarize ros2 bridge strict diagnostics"
```

Expected: commit succeeds.

---

## Verification Commands

Run these after all tasks are implemented:

```bash
cargo fmt --all -- --check
cargo check -p dora-ros2-bridge --no-default-features
cargo test -p dora-ros2-bridge-msg-gen
basic_venv="$(mktemp -d -t dora-ros2-strict-verify-XXXXXX)"
trap 'rm -rf "$basic_venv"' EXIT
uv venv --seed -p 3.12 "$basic_venv"
source "$basic_venv/bin/activate"
uv pip install pyarrow numpy
cargo test -p dora-ros2-bridge-python
deactivate
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh preflight
scripts/ros2-bridge-strict.sh smoke
scripts/ros2-bridge-strict.sh strict
```

Run this only when TurtleBot3 Gazebo and Nav2 are active and already pass ROS2 CLI baselines:

```bash
source /opt/ros/humble/setup.bash
scripts/ros2-bridge-strict.sh soak
```

## Self-Review

- Spec coverage: The plan covers environment snapshots, ROS2 interface availability, bridge startup through real service/action/topic cases, topic subscribe/publish with TurtleBot3 motion evidence, service client/server, action client/server, and Nav2 action integration.
- Placeholder scan: No task uses placeholder language; each created file has concrete content.
- Type consistency: Result classes, `fail_at` names, dataflow paths, and probe event names are defined before use and reused consistently.
- Scope check: The plan is intentionally focused on a diagnostic harness. It does not attempt to fix bridge bugs; it produces evidence for targeted follow-up fixes.
