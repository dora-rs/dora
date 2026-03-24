# Adora CLI Reference

Adora (AI-Dora, Dataflow-Oriented Robotic Architecture) is a 100% Rust framework for building real-time robotics and AI applications. This document covers the `adora` CLI from both an end-user and developer perspective.

## Table of Contents

- [Quick Start](#quick-start)
- [Installation](#installation)
- [Core Concepts](#core-concepts)
- [Dataflow Descriptor](#dataflow-descriptor)
- [Command Reference](#command-reference)
  - [Lifecycle](#lifecycle-commands)
  - [Monitoring](#monitoring-commands)
  - [Debugging](#debugging-commands)
  - [Setup](#setup-commands)
  - [Utility](#utility-commands)
  - [Self-Management](#self-management-commands)
- [Environment Variables](#environment-variables)
- [Architecture Guide](#architecture-guide)
- [Writing Nodes](#writing-nodes)
- [Writing Operators](#writing-operators)
- [Distributed Deployments](#distributed-deployments) -- see also [Distributed Deployment Guide](distributed-deployment.md) for cluster management, scheduling, and operations
- [Troubleshooting](#troubleshooting)
- [Debugging and Observability](debugging.md) -- standalone guide covering record/replay, topic inspection, log analysis, and resource monitoring
- **API References:** [Rust](api-rust.md) | [Python](api-python.md) | [C](api-c.md) | [C++](api-cxx.md)

---

## Quick Start

```bash
# Create a new project
adora new my-robot --kind dataflow --lang rust

# Run locally (no coordinator/daemon needed)
adora run dataflow.yml

# Or use coordinator/daemon for production
adora up
adora start dataflow.yml --attach
# Ctrl-C to stop
adora down
```

## Installation

### From crates.io (recommended)

```bash
cargo install adora-cli
```

### From source

```bash
cargo install --path binaries/cli --locked
```

### Verify

```bash
adora --version
adora status
```

---

## Core Concepts

### Dataflow

A **dataflow** is a directed graph of nodes connected by typed data channels. Nodes produce **outputs** that other nodes consume as **inputs**. The framework handles data routing, serialization (Apache Arrow), and lifecycle management.

### Execution Modes

| Mode | Command | Infrastructure | Use case |
|------|---------|---------------|----------|
| **Local** | `adora run` | None | Development, testing, single-machine |
| **Distributed** | `adora up` + `adora start` | Coordinator + Daemon(s) | Production, multi-machine |

### Component Roles

```
CLI  -->  Coordinator  -->  Daemon(s)  -->  Nodes / Operators
              (control plane)  (per machine)    (user code)
```

- **CLI**: User interface. Sends commands, displays logs.
- **Coordinator**: Orchestrates dataflow lifecycle across machines.
- **Daemon**: Spawns node processes, manages IPC, collects metrics.
- **Node**: A standalone process that produces and consumes Arrow data.
- **Operator**: In-process code running inside a shared runtime (lower latency than nodes).

### Data Format

All data flows through the system as **Apache Arrow** columnar arrays. This enables zero-copy shared memory transfer between co-located nodes and zero-serialization overhead.

---

## Dataflow Descriptor

Dataflows are defined in YAML files. Here is the complete schema:

### Minimal Example

```yaml
nodes:
  - id: sender
    path: sender.py
    outputs:
      - message

  - id: receiver
    path: receiver.py
    inputs:
      message: sender/message
```

### Full Schema

```yaml
# Dataflow-level settings
health_check_interval: 5.0    # health check sweep interval in seconds (default: 5.0)

nodes:
  - id: my-node                 # unique identifier (required)
    name: "My Node"             # human-readable name (optional)
    description: "..."          # description (optional)

    # --- Source (pick one) ---
    path: ./target/debug/my-node          # local executable
    # path: https://example.com/node.zip  # download from URL
    # git: https://github.com/org/repo.git  # build from git
    #   branch: main            # git branch (mutually exclusive with tag/rev)
    #   tag: v1.0               # git tag
    #   rev: abc123             # git commit hash

    # --- Build ---
    build: cargo build -p my-node   # shell command to build (optional)

    # --- Inputs ---
    inputs:
      # Short form: source_node/output_id
      tick: adora/timer/millis/100
      data: other-node/output

      # Long form with options
      sensor_data:
        source: sensor/frames
        queue_size: 10            # input buffer size (default: 10)
        queue_policy: drop_oldest # or "backpressure" (buffers up to 10x queue_size)
        input_timeout: 5.0        # circuit breaker timeout in seconds

    # --- Outputs ---
    outputs:
      - processed
      - status

    # --- Environment ---
    env:
      MY_VAR: "value"
      FROM_ENV:
        __adora_env: HOST_VAR     # read from host environment
    args: "--verbose"             # command-line arguments

    # --- Fault tolerance ---
    restart_policy: on-failure    # never (default) | on-failure | always
    max_restarts: 5               # 0 = unlimited
    restart_delay: 1.0            # initial backoff in seconds
    max_restart_delay: 30.0       # backoff cap in seconds
    restart_window: 300.0         # reset counter after N seconds
    health_check_timeout: 30.0    # kill if no activity for N seconds

    # --- Logging ---
    min_log_level: info           # source-level filter (daemon-side)
    send_stdout_as: raw_output    # route raw stdout as data output
    send_logs_as: log_entries     # route structured logs as data output
    max_log_size: "50MB"          # rotate log files at this size
    max_rotated_files: 5          # number of rotated files to keep (1-100)

    # --- Deployment ---
    _unstable_deploy:
      machine: A                  # target machine/daemon ID

# Debug settings
_unstable_debug:
  publish_all_messages_to_zenoh: true   # required for topic echo/hz/info
```

### Built-in Timer Nodes

Timers are virtual nodes that emit ticks at fixed intervals:

```yaml
inputs:
  tick: adora/timer/millis/100   # every 100ms
  slow: adora/timer/millis/1000  # every 1s
  fast: adora/timer/hz/30        # 30 Hz (~33ms)
```

### Operator Nodes

Operators run in-process inside a shared runtime (no separate process):

```yaml
nodes:
  # Single operator (shorthand)
  - id: detector
    operator:
      python: detect.py
      build: pip install -r requirements.txt
      inputs:
        image: camera/frames
      outputs:
        - bbox

  # Multiple operators sharing a runtime
  - id: runtime-node
    operators:
      - id: preprocessor
        shared-library: ../../target/debug/libpreprocess
        inputs:
          raw: sensor/data
        outputs:
          - processed
      - id: analyzer
        shared-library: ../../target/debug/libanalyze
        inputs:
          data: runtime-node/preprocessor/processed
        outputs:
          - result
```

### Distributed Deployment

Assign nodes to specific machines using `_unstable_deploy`:

```yaml
nodes:
  - id: camera-driver
    _unstable_deploy:
      machine: robot-arm
    path: ./target/debug/camera
    outputs:
      - frames

  - id: ml-inference
    _unstable_deploy:
      machine: gpu-server
    path: ./target/debug/inference
    inputs:
      frames: camera-driver/frames
    outputs:
      - predictions
```

When nodes are on different machines, communication automatically switches from shared memory to Zenoh pub/sub.

---

## Command Reference

### Lifecycle Commands

#### `adora run`

Run a dataflow locally without coordinator or daemon. Best for development and testing.

```
adora run <PATH> [OPTIONS]
```

| Argument/Flag | Default | Description |
|---------------|---------|-------------|
| `<PATH>` | required | Path to dataflow descriptor YAML |
| `--stop-after <DURATION>` | | Auto-stop after duration (e.g., `30s`, `5m`) |
| `--uv` | false | Use `uv` for Python node management |
| `--debug` | false | Enable debug topics (equivalent to `publish_all_messages_to_zenoh: true`) |
| `--allow-shell-nodes` | false | Enable shell-based node execution |
| `--log-level <LEVEL>` | `stdout` | Min display level: `error\|warn\|info\|debug\|trace\|stdout` |
| `--log-format <FORMAT>` | `pretty` | Output format: `pretty\|json\|compact` |
| `--log-filter <FILTER>` | | Per-node level overrides: `"node1=debug,node2=warn"` |

**Examples:**

```bash
# Basic run
adora run dataflow.yml

# Stop after 10 seconds, only show warnings
adora run dataflow.yml --stop-after 10s --log-level warn

# Python dataflow with uv
adora run dataflow.yml --uv

# Debug one node, silence others
adora run dataflow.yml --log-level warn --log-filter "sensor=debug"

# JSON output for CI pipelines
adora run dataflow.yml --log-format json --stop-after 30s 2>test.json
```

#### `adora up`

Start coordinator and daemon in local mode.

```
adora up
```

Spawns `adora coordinator` and `adora daemon` as background processes. Waits for both to be ready before returning. Idempotent: if already running, does nothing.

When running the daemon directly (e.g., for distributed deployments), additional flags are available:

| Flag | Default | Description |
|------|---------|-------------|
| `--worker-threads <N>` | num_cpus | Tokio worker thread count |
| `--rt` | false | Real-time profile: mlockall + SCHED_FIFO (Linux). See [tuning guide](realtime-tuning.md). |
| `--machine-id <ID>` | | Unique machine identifier for distributed mode |
| `--allow-shell-nodes` | false | Enable shell-based node execution |

#### `adora down` (alias: `adora destroy`)

Tear down coordinator and daemon. Stops all running dataflows first.

```
adora down [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

#### `adora build`

Run build commands defined in the dataflow descriptor.

```
adora build <PATH> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<PATH>` | required | Dataflow descriptor path |
| `--uv` | false | Use `uv` for Python builds |
| `--local` | false | Force local build (skip coordinator) |
| `--strict-types` | false | Treat type warnings as errors (non-zero exit code) |

**Type checking:** After expanding modules, `build` runs the same type checks as `validate`. Warnings are printed by default; use `--strict-types` (or set `strict_types: true` in the YAML) to fail the build on type mismatches. User-defined types in a `types/` directory next to the dataflow are loaded automatically.

**Build strategy:** If nodes have `_unstable_deploy` sections and a coordinator is reachable, builds are distributed to target machines. Otherwise, builds run locally.

**Git sources:** Nodes with a `git:` field are cloned/updated before building. The build command runs from the git repository root.

#### `adora start`

Start a dataflow on a running coordinator.

```
adora start <PATH> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<PATH>` | required | Dataflow descriptor path |
| `--name <NAME>`, `-n` | | Assign a name to the dataflow |
| `--attach` | auto | Attach to log stream and wait for completion |
| `--detach` | auto | Return immediately after spawn |
| `--debug` | false | Enable debug topics (equivalent to `publish_all_messages_to_zenoh: true`) |
| `--hot-reload` | false | Watch Python files and reload on change |
| `--uv` | false | Use `uv` for Python nodes |
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

If neither `--attach` nor `--detach` is specified: attaches if running in a TTY, detaches otherwise.

**Attach mode:** Streams logs, handles Ctrl-C gracefully (first = stop, second = force kill).

**Hot reload:** Watches Python operator source files. On change, sends a reload request to the coordinator which propagates to the daemon.

#### `adora stop`

Stop a running dataflow.

```
adora stop [UUID_OR_NAME] [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `[UUID_OR_NAME]` | interactive | Dataflow UUID or name |
| `--name <NAME>`, `-n` | | Alternative name specification |
| `--grace-duration <DURATION>` | | Graceful shutdown timeout |
| `--force`, `-f` | false | Immediate termination |
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

If no identifier is given and running in a TTY, presents an interactive picker.

**Stop sequence:** Send Event::Stop -> wait grace duration -> SIGTERM -> hard kill.

#### `adora restart`

Restart a running dataflow (stop + re-start with stored descriptor). No YAML path needed -- the coordinator retains the original descriptor.

```
adora restart [UUID] [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `[UUID]` | | Dataflow UUID |
| `--name <NAME>`, `-n` | | Restart by name instead of UUID |
| `--grace-duration <DURATION>` | | Graceful shutdown timeout for the stop phase |
| `--force`, `-f` | false | Force kill before restart |
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

**Examples:**

```bash
# Restart by name
adora restart --name my-app

# Restart by UUID with forced stop
adora restart a1b2c3d4-... --force
```

#### `adora record`

Record dataflow messages to an `.adorec` file for offline replay. See [Debugging Guide](debugging.md#record-and-replay) for full workflows.

```
adora record <DATAFLOW_YAML> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<DATAFLOW_YAML>` | required | Path to dataflow descriptor |
| `-o, --output <PATH>` | `recording_{timestamp}.adorec` | Output file path |
| `--topics <TOPICS>` | all | Comma-separated `node/output` topics to record |
| `--proxy` | false | Stream via WebSocket instead of recording on target |
| `--output-yaml <PATH>` | | Write modified YAML without running (dry run) |

Default mode injects a record node into the dataflow. `--proxy` mode requires a running dataflow and `publish_all_messages_to_zenoh: true`.

#### `adora replay`

Replay a recorded `.adorec` file by replacing source nodes with replay nodes. See [Debugging Guide](debugging.md#replaying-a-recording) for full workflows.

```
adora replay <FILE> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<FILE>` | required | Path to `.adorec` recording |
| `--speed <FLOAT>` | `1.0` | Playback speed (0 = max speed) |
| `--loop` | false | Loop the recording |
| `--replace <NODE_IDS>` | all recorded | Comma-separated nodes to replace |
| `--output-yaml <PATH>` | | Write modified YAML without running (dry run) |

---

### Monitoring Commands

#### `adora list` (alias: `adora ps`)

List running dataflows with metrics.

```
adora list [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--format <FMT>`, `-f` | `table` | Output format: `table\|json` |
| `--status <STATUS>` | | Filter: `running\|finished\|failed` |
| `--name <PATTERN>` | | Filter by name (case-insensitive substring) |
| `--sort-by <FIELD>` | | Sort by: `cpu\|memory` |
| `--quiet`, `-q` | false | Print only UUIDs |
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

**Output columns:** UUID, Name, Status, Nodes, CPU, Memory

#### `adora logs`

Show and follow logs of a dataflow and node.

```
adora logs [UUID_OR_NAME] [NODE] [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `[UUID_OR_NAME]` | | Dataflow UUID or name |
| `[NODE]` | | Node name (required unless `--all-nodes`) |
| `--all-nodes` | false | Merge logs from all nodes by timestamp |
| `--tail <N>` | all | Show last N lines |
| `--follow`, `-f` | false | Stream new log entries |
| `--local` | false | Read from local `out/` directory |
| `--since <DURATION>` | | Show logs newer than duration ago |
| `--until <DURATION>` | | Show logs older than duration ago |
| `--level <LEVEL>` | `stdout` | Min log level |
| `--log-format <FORMAT>` | `pretty` | Output format |
| `--log-filter <FILTER>` | | Per-node level overrides |
| `--grep <PATTERN>` | | Case-insensitive text search |
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

**Filter pipeline:** Read/Parse -> Time filters -> Grep -> Tail -> Display

**Examples:**

```bash
# Follow all nodes live
adora logs my-dataflow --all-nodes --follow

# Last 50 errors from a specific node
adora logs my-dataflow sensor --level error --tail 50

# Search logs from last 5 minutes
adora logs my-dataflow --all-nodes --since 5m --grep "timeout"

# Read local files (no coordinator needed)
adora logs --local --all-nodes --tail 100

# Post-mortem analysis: errors in time window
adora logs --local sensor --since 1h --until 30m --level error
```

**Duration formats:** `30` (seconds), `30s`, `5m`, `1h`, `2d`

#### `adora inspect top` (alias: `adora top`)

Real-time TUI monitor for node resource usage (like `top`).

```
adora inspect top [OPTIONS]
adora top [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--refresh-interval <SECONDS>` | `2` | Update interval (min: 1) |
| `--once` | false | Print a single JSON snapshot and exit (for scripting/CI) |
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

**Requires an interactive terminal** (unless `--once` is used).

| Key | Action |
|-----|--------|
| `q` / `Esc` | Quit |
| `Up` / `k` | Select previous node |
| `Down` / `j` | Select next node |
| `n` | Sort by node name |
| `c` | Sort by CPU |
| `m` | Sort by memory |
| `r` | Force refresh |

**Columns:** NODE, STATUS, DATAFLOW, PID, CPU%, MEMORY (MB), RESTARTS, QUEUE, NET TX, NET RX, I/O READ (MB/s), I/O WRITE (MB/s)

- **STATUS**: Running, Restarting, Degraded (broken inputs), or Failed
- **RESTARTS**: Current restart count per node
- **QUEUE**: Pending messages in the node's input queue
- **NET TX/RX**: Cumulative cross-daemon network bytes sent/received via Zenoh

CPU values are per-core (can exceed 100% with multiple cores). Metrics come from daemons, so this works for distributed deployments.

**Scripting example:**

```bash
# JSON snapshot for CI/monitoring pipelines
adora top --once | jq '.[].cpu_usage'
```

#### `adora topic list`

List all topics (outputs) in a running dataflow.

```
adora topic list [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |
| `--format <FMT>` | `table` | Output format: `table\|json` |

#### `adora topic echo`

Subscribe to topics and display messages in real-time.

```
adora topic echo [OPTIONS] [DATA...]
```

| Flag | Default | Description |
|------|---------|-------------|
| `-d <DATAFLOW>`, `--dataflow` | required | Dataflow UUID or name |
| `[DATA...]` | all outputs | Topics to echo (e.g., `node1/output`) |
| `--format <FMT>` | `table` | Output format: `table\|json` |

Requires `_unstable_debug.publish_all_messages_to_zenoh: true` in the descriptor.

#### `adora topic hz`

Measure topic publish frequency with a TUI dashboard.

```
adora topic hz [OPTIONS] [DATA...]
```

| Flag | Default | Description |
|------|---------|-------------|
| `-d <DATAFLOW>`, `--dataflow` | required | Dataflow UUID or name |
| `[DATA...]` | all outputs | Topics to measure |
| `--window <SECONDS>` | `10` | Sliding window (min: 1) |

**Requires an interactive terminal.** Displays: Avg (ms), Avg (Hz), Min (ms), Max (ms), Std (ms), plus a rate sparkline and histogram for the selected topic.

#### `adora topic info`

Show detailed metadata of a single topic.

```
adora topic info [OPTIONS] DATA
```

| Flag | Default | Description |
|------|---------|-------------|
| `-d <DATAFLOW>`, `--dataflow` | required | Dataflow UUID or name |
| `DATA` | required | Single topic (e.g., `camera/image`) |
| `--duration <SECONDS>` | `5` | Collection duration (min: 1) |

Subscribes to the topic for the specified duration and reports: type (Arrow schema), publisher, subscribers, message count, bandwidth.

#### `adora node`

Manage and inspect dataflow nodes.

##### `adora node list`

```
adora node list [OPTIONS]
```

Lists nodes in a running dataflow with their status, CPU, memory, and restart count.

**Columns:** NODE, STATUS, PID, CPU%, MEMORY (MB), RESTARTS, DATAFLOW

##### `adora node info`

Show detailed information about a specific node including status, inputs, outputs, and metrics.

```
adora node info <NODE> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NODE>` | required | Node ID to inspect |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |
| `-f <FORMAT>`, `--format` | `table` | Output format: `table\|json` |

##### `adora node restart`

Restart a single node within a running dataflow. The daemon stops the node process and respawns it.

```
adora node restart <NODE> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NODE>` | required | Node ID to restart |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |
| `--grace <DURATION>` | | Grace period before force-killing the node |

##### `adora node stop`

Stop a single node within a running dataflow without stopping the entire dataflow.

```
adora node stop <NODE> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NODE>` | required | Node ID to stop |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |
| `--grace <DURATION>` | | Grace period before force-killing the node |

##### `adora node add`

Add a node to a running dataflow dynamically.

```
adora node add --from-yaml <FILE> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--from-yaml <FILE>` | required | YAML file containing a single node definition |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |

##### `adora node remove`

Remove a node from a running dataflow (stops the node and cleans up mappings).

```
adora node remove <NODE> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NODE>` | required | Node ID to remove |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |
| `--grace <SECS>` | | Grace period in seconds before force-killing |

##### `adora node connect`

Add a live mapping between two nodes in a running dataflow.

```
adora node connect <SOURCE/OUTPUT> <TARGET/INPUT> [OPTIONS]
```

##### `adora node disconnect`

Remove a live mapping between two nodes in a running dataflow.

```
adora node disconnect <SOURCE/OUTPUT> <TARGET/INPUT> [OPTIONS]
```

#### `adora topic pub`

Publish JSON data to a topic in a running dataflow. Requires `publish_all_messages_to_zenoh: true`.

```
adora topic pub <TOPIC> [DATA] [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<TOPIC>` | required | Topic to publish to (format: `node_id/output_id`) |
| `[DATA]` | | JSON data to publish (required unless `--file`) |
| `--file <PATH>` | | Read data from a JSON file instead of command line |
| `--count <N>` | `1` | Number of messages to publish |
| `-d <DATAFLOW>`, `--dataflow` | required | Dataflow UUID or name |

**Examples:**

```bash
# Publish a single value
adora topic pub -d my-app sensor/threshold '[42]'

# Publish from file, 10 times
adora topic pub -d my-app sensor/config --file config.json --count 10
```

#### `adora param`

Manage runtime parameters for nodes. Parameters are persisted in the coordinator store and optionally forwarded to running nodes.

##### `adora param list`

List all runtime parameters for a node.

```
adora param list <NODE> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NODE>` | required | Node ID |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |
| `--format <FMT>` | `table` | Output format: `table\|json` |

##### `adora param get`

Get a single runtime parameter value.

```
adora param get <NODE> <KEY> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NODE>` | required | Node ID |
| `<KEY>` | required | Parameter key |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |

##### `adora param set`

Set a runtime parameter. The value is JSON. The parameter is stored in the coordinator and forwarded to the node if it is running.

```
adora param set <NODE> <KEY> <VALUE> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NODE>` | required | Node ID |
| `<KEY>` | required | Parameter key (max 256 bytes) |
| `<VALUE>` | required | Parameter value as JSON (max 64KB serialized) |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |

**Examples:**

```bash
# Set a numeric parameter
adora param set -d my-app sensor threshold 42

# Set a string parameter
adora param set -d my-app camera resolution '"1080p"'

# Set a complex parameter
adora param set -d my-app detector config '{"confidence": 0.8, "nms": 0.5}'
```

##### `adora param delete`

Delete a runtime parameter.

```
adora param delete <NODE> <KEY> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NODE>` | required | Node ID |
| `<KEY>` | required | Parameter key |
| `-d <DATAFLOW>`, `--dataflow` | interactive | Dataflow UUID or name |

#### `adora doctor`

Diagnose environment, coordinator/daemon connectivity, and optionally validate a dataflow YAML.

```
adora doctor [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--dataflow <PATH>` | | Path to a dataflow YAML to validate |

Checks performed:
1. Coordinator reachability
2. Daemon connectivity
3. Active dataflow status
4. Dataflow YAML validation (if `--dataflow` provided)

**Examples:**

```bash
# Basic health check
adora doctor

# Check environment + validate a dataflow
adora doctor --dataflow dataflow.yml
```

#### `adora trace list`

List recent traces captured by the coordinator. The coordinator captures spans from `adora_coordinator` and `adora_core` crates in-memory (up to 4096 spans). No external tracing infrastructure required.

```
adora trace list [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

**Output columns:** TRACE ID (first 12 chars), ROOT SPAN, SPANS, STARTED, DURATION

**Example:**

```bash
adora trace list
```

```
TRACE ID      ROOT SPAN          SPANS  STARTED              DURATION
a1b2c3d4e5f6  spawn_dataflow     12     2026-03-01 10:30:05  1.234s
f8e7d6c5b4a3  build_dataflow     5      2026-03-01 10:29:58  0.500s
```

#### `adora trace view`

View spans for a specific trace as an indented tree. Supports prefix matching on trace IDs.

```
adora trace view <TRACE_ID> [OPTIONS]
```

| Argument/Flag | Default | Description |
|---------------|---------|-------------|
| `<TRACE_ID>` | required | Full trace ID or unique prefix |
| `--coordinator-addr <IP>` | `127.0.0.1` | Coordinator address |
| `--coordinator-port <PORT>` | `6013` | Coordinator port |

**Example:**

```bash
adora trace view a1b2c3d4
```

```
spawn_dataflow [INFO 1.234s] {build_id="abc", session_id="def"}
  build_dataflow [INFO 0.500s]
    download_node [DEBUG 0.200s] {url="..."}
  start_inner [INFO 0.734s]
    spawn_node [INFO 0.100s] {node_id="camera"}
    spawn_node [INFO 0.080s] {node_id="detector"}
```

Trace IDs are prefix-matched: if the prefix uniquely identifies a trace, it resolves automatically. If ambiguous, you'll be prompted to use a longer prefix.

---

### Setup Commands

#### `adora status` (alias: `adora check`)

Check system health and connectivity.

```
adora status [OPTIONS]
```

Reports coordinator connectivity, daemon status, and active dataflow count.

#### `adora new`

Generate a new project or node from templates.

```
adora new <NAME> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<NAME>` | required | Project or node name |
| `--kind <KIND>` | `dataflow` | `dataflow\|node` |
| `--lang <LANG>` | `rust` | `rust\|python\|c\|cxx` |

#### `adora expand`

Expand module references in a dataflow and print the resulting flat YAML. Useful for debugging module composition.

```
adora expand <PATH> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<PATH>` | required | Dataflow descriptor (or module file with `--module`) |
| `--module` | false | Validate a standalone module file instead of a full dataflow |

**Examples:**

```bash
# Expand a dataflow with modules
adora expand dataflow.yml

# Validate a module file
adora expand --module modules/navigation.module.yml
```

See the [Modules Guide](modules.md) for full documentation on module composition.

#### `adora graph`

Visualize a dataflow as a graph.

```
adora graph <PATH> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<PATH>` | required | Dataflow descriptor path |
| `--mermaid` | false | Output Mermaid diagram text |
| `--open` | false | Open HTML in browser |

Without `--mermaid`, generates an interactive HTML file using mermaid.js. When outputs have type annotations, edge labels include the type name (e.g. `image [Image]`).

```bash
# Generate HTML
adora graph dataflow.yml --open

# Generate Mermaid for GitHub markdown
adora graph dataflow.yml --mermaid
```

#### `adora validate`

Validate a dataflow YAML file and check type annotations.

```
adora validate <PATH> [OPTIONS]
```

| Flag | Default | Description |
|------|---------|-------------|
| `<PATH>` | required | Dataflow descriptor path |
| `--strict-types` | false | Treat warnings as errors (non-zero exit code for CI) |

Checks:
1. **Key existence**: `output_types`/`input_types` keys exist in the corresponding `outputs`/`inputs` lists
2. **URN resolution**: All type URNs resolve in the standard or user-defined type library
3. **Edge compatibility**: Connected edges have compatible types (exact match, widening, or user-defined rules)
4. **Parameterized types**: Parameter mismatches (e.g. `AudioFrame[sample_type=f32]` vs `AudioFrame[sample_type=i16]`)
5. **Timer auto-typing**: Timer inputs are automatically typed as `std/core/v1/UInt64`
6. **Type inference**: When only upstream annotates a type, it is inferred on the downstream input
7. **Metadata patterns**: `output_metadata` keys and `pattern` shorthands are validated
8. **Schema compatibility**: Struct types are checked at the field level (missing/wrong fields)

User-defined types in a `types/` directory next to the dataflow are loaded automatically.

```bash
# Validate with warnings
adora validate dataflow.yml

# Strict mode for CI (exit 1 on warnings)
adora validate --strict-types dataflow.yml
```

See the [Type Annotations Guide](types.md) for the full type library and usage details.

---

### Utility Commands

#### `adora completion`

Generate shell completion scripts.

```
adora completion [SHELL]
```

Shell is auto-detected if omitted. Supported: bash, zsh, fish, elvish, powershell.

```bash
# Bash
eval "$(adora completion bash)"
echo 'eval "$(adora completion bash)"' >> ~/.bashrc

# Zsh
eval "$(adora completion zsh)"
echo 'eval "$(adora completion zsh)"' >> ~/.zshrc

# Fish
adora completion fish > ~/.config/fish/completions/adora.fish
```

#### `adora system`

System management commands.

```
adora system status [OPTIONS]
```

Currently provides `status` as a subcommand (equivalent to `adora status`).

---

### Self-Management Commands

#### `adora self update`

Check for and install CLI updates.

```
adora self update [--check-only]
```

Downloads from GitHub releases (`dora-rs/adora`).

#### `adora self uninstall`

Remove the CLI from the system.

```
adora self uninstall [--force]
```

Without `--force`, prompts for confirmation (requires a TTY). Tries `uv pip uninstall` first, then `pip uninstall`, then binary self-delete.

---

## Environment Variables

All environment variables serve as fallbacks. CLI flags always take precedence.

| Variable | Default | Commands | Description |
|----------|---------|----------|-------------|
| `ADORA_COORDINATOR_ADDR` | `127.0.0.1` | All coordinator commands | Coordinator IP address |
| `ADORA_COORDINATOR_PORT` | `6013` | All coordinator commands | Coordinator WebSocket port |
| `ADORA_LOG_LEVEL` | `stdout` | `run`, `logs` | Default minimum log level |
| `ADORA_LOG_FORMAT` | `pretty` | `run`, `logs` | Default output format |
| `ADORA_LOG_FILTER` | | `run`, `logs` | Default per-node level overrides |
| `ADORA_ALLOW_SHELL_NODES` | | `run` | Enable shell node execution |
| `ADORA_RUNTIME_TYPE_CHECK` | | `run`, `start` | Runtime type checking: `warn` (log mismatches) or `error` (fail on mismatch). See [Type Annotations](types.md#runtime-type-checking) |

```bash
# Set defaults for a development session
export ADORA_COORDINATOR_ADDR=192.168.1.10
export ADORA_LOG_LEVEL=info
export ADORA_LOG_FORMAT=compact
```

---

## Architecture Guide

This section is for developers who want to understand the framework internals, extend it, or debug issues.

### Communication Stack

```
                    ┌─────────────────────────────────────┐
                    │           CLI (adora)                │
                    │   WebSocket (JSON request/reply)     │
                    └─────────────┬───────────────────────┘
                                  │
                    ┌─────────────▼───────────────────────┐
                    │        Coordinator                   │
                    │   WebSocket control + daemon mgmt    │
                    │   State: InMemoryStore | RedbStore   │
                    └──┬──────────────────────────────┬───┘
                       │                              │
          ┌────────────▼──────────┐     ┌─────────────▼──────────┐
          │     Daemon A          │     │     Daemon B           │
          │  (machine: robot)     │     │  (machine: gpu-server) │
          │                       │     │                        │
          │  ┌─────┐  ┌─────┐    │     │  ┌──────┐  ┌───────┐  │
          │  │Node1│  │Node2│    │     │  │Node3 │  │Node4  │  │
          │  └──┬──┘  └──┬──┘    │     │  └──┬───┘  └───┬───┘  │
          │     │shmem    │shmem  │     │     │shmem      │shmem │
          │     └────┬────┘       │     │     └─────┬─────┘      │
          └──────────┼────────────┘     └───────────┼────────────┘
                     │                              │
                     └──────── Zenoh pub/sub ────────┘
                              (cross-machine)
```

### Protocol Layers

| Layer | Transport | Format | Use |
|-------|-----------|--------|-----|
| CLI <-> Coordinator | WebSocket | JSON (ControlRequest/Reply) | Commands, log streaming |
| Coordinator <-> Daemon | WebSocket | JSON (DaemonCoordinatorEvent) | Node lifecycle, metrics |
| Daemon <-> Node (small) | TCP / Unix socket | Custom binary | Control messages, small data |
| Daemon <-> Node (large) | Shared memory | Zero-copy Arrow | Data messages > 4KB |
| Daemon <-> Daemon | Zenoh pub/sub | Arrow + metadata | Cross-machine data routing |

### Coordinator Internals

The coordinator is an event-driven async server:

```
Event Sources:
  - CLI WebSocket connections (ControlRequest)
  - Daemon WebSocket connections (DaemonEvent)
  - Heartbeat timer (3s interval)
  - External events (for embedding)

Event Loop:
  merge_all(cli_events, daemon_events, heartbeat, external)
    -> handle_event()
    -> update state
    -> persist to store (if redb)
    -> send replies
```

**Key types:**

```rust
// State
RunningDataflow { uuid, name, descriptor, daemons, node_metrics, ... }
RunningBuild    { build_id, errors, log_subscribers, pending_results, ... }
DaemonConnection { sender, pending_replies, last_heartbeat }

// Store trait
trait CoordinatorStore: Send + Sync {
    fn put_dataflow(&self, record: &DataflowRecord) -> Result<()>;
    fn get_dataflow(&self, uuid: &Uuid) -> Result<Option<DataflowRecord>>;
    fn list_dataflows(&self) -> Result<Vec<DataflowRecord>>;
    // ... daemon and build methods
}
```

**Store backends:**
- `memory` (default): In-memory, lost on restart.
- `redb`: Persistent to disk (`~/.adora/coordinator.redb`). Survives crashes. Requires `redb-backend` feature.

```bash
adora coordinator --store redb
adora coordinator --store redb:/custom/path.redb
```

### Daemon Internals

The daemon manages node processes on a single machine:

```
Per Node:
  1. Build (if build command specified)
  2. Spawn process with ADORA_NODE_CONFIG env var
  3. Node registers via TCP/shmem handshake
  4. Route inputs/outputs between nodes
  5. Collect metrics (CPU, memory, I/O)
  6. Handle restart policy on exit
  7. Forward logs to coordinator

Communication:
  - Shared memory for messages > 4KB (zero-copy)
  - TCP for control messages and small data
  - flume channels for internal event routing
```

**Metrics collection:**

```rust
struct NodeMetrics {
    pid: u32,
    cpu_usage: f32,      // per-core percentage
    memory_mb: f64,
    disk_read_mb_s: Option<f64>,
    disk_write_mb_s: Option<f64>,
    status: NodeStatus,  // Running | Restarting | Degraded | Failed
    restart_count: u32,
    pending_messages: u64,
}
```

### Message Types

All inter-component messages are defined in `libraries/message/`:

```rust
// Node identification
struct NodeId(String);      // [a-zA-Z0-9_.-]
struct DataId(String);      // same validation
type DataflowId = uuid::Uuid;

// Data metadata
struct Metadata {
    timestamp: uhlc::Timestamp,    // hybrid logical clock
    type_info: ArrowTypeInfo,      // Arrow schema
    parameters: MetadataParameters, // custom key-value pairs
}

// Node events (daemon -> node)
enum NodeEvent {
    Stop,
    Reload { operator_id },
    Input { id, metadata, data },
    InputClosed { id },
    InputRecovered { id },
    NodeRestarted { id },
    AllInputsClosed,
}
```

### Timestamping

Adora uses a **Unified Hybrid Logical Clock** (UHLC) for distributed causality. Every message carries a `uhlc::Timestamp` that preserves causal ordering across machines without synchronized clocks.

### Zero-Copy Shared Memory

For large messages (> 4KB), the daemon uses shared memory regions:

1. Sender node requests a shared memory slot from daemon
2. Daemon allocates a region and returns the ID
3. Sender writes Arrow data directly into shared memory
4. Daemon notifies receiver node of the region ID
5. Receiver reads directly from shared memory (zero-copy)
6. Receiver sends a drop token when done

This achieves 10-17x lower latency than ROS2 for large payloads.

---

## Writing Nodes

### Rust Node

```rust
use adora_node_api::{AdoraNode, Event, IntoArrow};
use adora_core::config::DataId;

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;

    let output = DataId::from("result".to_owned());

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => {
                // Process input data (Arrow array)
                let result: u64 = 42;
                node.send_output(
                    output.clone(),
                    metadata.parameters,
                    result.into_arrow(),
                )?;
            }
            Event::Stop(_) => break,
            Event::InputClosed { id } => {
                eprintln!("input {id} closed");
            }
            Event::InputRecovered { id } => {
                eprintln!("input {id} recovered");
            }
            _ => {}
        }
    }
    Ok(())
}
```

**Cargo.toml:**

```toml
[dependencies]
adora-node-api = { workspace = true }
eyre = "0.6"
```

### Python Node

```python
import pyarrow as pa
from adora import Node

node = Node()

for event in node:
    if event["type"] == "INPUT":
        # event["value"] is a PyArrow array
        values = event["value"].to_pylist()
        result = pa.array([sum(values)])
        node.send_output("result", result)
    elif event["type"] == "STOP":
        break
```

### C Node

```c
#include "node_api.h"

int main() {
    void *ctx = init_adora_context_from_env();
    // ... event loop using adora_next_event / adora_send_output
    free_adora_context(ctx);
    return 0;
}
```

### Node Logging

Nodes can emit structured logs:

**Rust:**

```rust
// Via tracing (recommended)
tracing::info!("processing frame {}", frame_id);

// Via node API
node.log_info("processing complete");
node.log_with_fields("info", "reading", None, Some(&fields));
```

**Python:**

```python
import logging
logging.info("processing frame %d", frame_id)

# Or via node API
node.log("info", "processing complete")
```

---

## Writing Operators

Operators run in-process inside a shared runtime, avoiding process spawn overhead.

### Rust Operator

```rust
use adora_operator_api::{register_operator, AdoraOperator, AdoraOutputSender, AdoraStatus, Event};

#[register_operator]
#[derive(Default)]
pub struct MyOperator {
    counter: u32,
}

impl AdoraOperator for MyOperator {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut AdoraOutputSender,
    ) -> Result<AdoraStatus, String> {
        match event {
            Event::Input { id, data } => {
                self.counter += 1;
                output_sender.send(
                    "count".to_string(),
                    arrow::array::UInt32Array::from(vec![self.counter]),
                )?;
                Ok(AdoraStatus::Continue)
            }
            Event::Stop => Ok(AdoraStatus::Stop),
            _ => Ok(AdoraStatus::Continue),
        }
    }
}
```

**Cargo.toml:**

```toml
[lib]
crate-type = ["cdylib"]

[dependencies]
adora-operator-api = { workspace = true }
arrow = "53"
```

### Python Operator

```yaml
nodes:
  - id: my-node
    operator:
      python: my_operator.py
      inputs:
        data: source/output
      outputs:
        - result
```

```python
# my_operator.py
class Operator:
    def __init__(self):
        self.counter = 0

    def on_event(self, event, send_output):
        if event["type"] == "INPUT":
            self.counter += 1
            send_output("result", pa.array([self.counter]))
```

---

## Distributed Deployments

### Setup

```bash
# Machine A (coordinator + daemon)
adora up

# Machine B (daemon only, pointing to coordinator on Machine A)
adora daemon --interface 0.0.0.0 --coordinator-addr 192.168.1.10 --machine-id B

# Machine C (same)
adora daemon --interface 0.0.0.0 --coordinator-addr 192.168.1.10 --machine-id C
```

### Dataflow with Machine Assignment

```yaml
nodes:
  - id: camera
    _unstable_deploy:
      machine: robot
    path: ./camera-driver
    outputs:
      - frames

  - id: inference
    _unstable_deploy:
      machine: gpu-server
    path: ./ml-model
    inputs:
      frames: camera/frames
    outputs:
      - predictions

  - id: actuator
    _unstable_deploy:
      machine: robot
    path: ./actuator-driver
    inputs:
      commands: inference/predictions
```

### Build and Start

```bash
# From any machine with coordinator access
adora build dataflow.yml       # distributed build on target machines
adora start dataflow.yml --name my-robot --attach
```

### Monitor

```bash
# Resource usage across all machines
adora top

# Logs from any node regardless of machine
adora logs my-robot inference --follow

# List all dataflows
adora list
```

### Coordinator Persistence

For production, use the redb store backend so the coordinator survives restarts:

```bash
adora coordinator --store redb
```

State is persisted to `~/.adora/coordinator.redb`. On restart, stale dataflows are marked as failed and the coordinator resumes normal operation.

> For managed cluster deployments (cluster.yml, SSH-based lifecycle, label scheduling, systemd services, rolling upgrades), see the [Distributed Deployment Guide](distributed-deployment.md).

---

## Troubleshooting

> For a comprehensive debugging guide covering record/replay workflows, topic inspection, resource monitoring, and end-to-end debugging scenarios, see [Debugging and Observability Guide](debugging.md).

### Common Issues

**"Could not connect to adora-coordinator"**
- Run `adora up` first, or check `ADORA_COORDINATOR_ADDR`/`ADORA_COORDINATOR_PORT`
- Verify with `adora status`

**"publish_all_messages_to_zenoh not enabled"**
- Use `--debug` flag: `adora start dataflow.yml --debug` or `adora run dataflow.yml --debug`
- Or add to your dataflow YAML:
  ```yaml
  _unstable_debug:
    publish_all_messages_to_zenoh: true
  ```
- Required for `topic echo`, `topic hz`, `topic info`

**"`adora top` requires an interactive terminal"**
- These TUI commands need a real terminal (not piped output)
- Same applies to `topic hz`

**Node not receiving inputs**
- Check that output names match: `source_node/output_id`
- Verify the source node lists the output in its `outputs:` array
- Check `adora topic list` for available topics

**Logs not appearing**
- Check `--log-level` setting (default `stdout` shows everything)
- Check `min_log_level` in YAML (filters at source)
- For distributed: verify coordinator/daemon connectivity

**Build fails with git source**
- Verify `git:` URL is accessible
- Check that `branch`, `tag`, or `rev` exists
- Build command runs from the git repo root, not the dataflow directory

### Debug Workflow

```bash
# 1. Full environment diagnosis
adora doctor --dataflow dataflow.yml

# 2. Start with verbose logging and debug topics
adora run dataflow.yml --log-level trace --debug

# 3. Inspect a specific node
adora node info -d my-dataflow problem-node

# 4. Monitor specific node logs
adora logs my-dataflow problem-node --follow --level debug

# 5. Check resource usage
adora top

# 6. Inspect topic data
adora topic echo -d my-dataflow problem-node/output

# 7. Publish test data to a topic
adora topic pub -d my-dataflow problem-node/input '[1, 2, 3]'

# 8. Measure frequencies
adora topic hz -d my-dataflow --window 5

# 9. View/modify runtime parameters
adora param list -d my-dataflow problem-node
adora param set -d my-dataflow problem-node threshold 42

# 10. Restart a misbehaving node without stopping the dataflow
adora node restart -d my-dataflow problem-node

# 11. View coordinator traces (no external infra needed)
adora trace list
adora trace view <trace-id-prefix>

# 12. Visualize dataflow graph
adora graph dataflow.yml --open
```

### Log File Locations

```
out/
  <dataflow-uuid>/
    log_<node-id>.jsonl          # current log
    log_<node-id>.1.jsonl        # rotated (previous)
    log_<node-id>.2.jsonl        # rotated (older)
```

Read directly with:

```bash
adora logs --local --all-nodes
adora logs --local <node-name> --tail 50
```
