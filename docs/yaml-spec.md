# Dataflow YAML Specification

Dataflows are defined in YAML files. Each file describes a graph of nodes, their inputs/outputs, and execution parameters.

A JSON Schema is available at the repo root (`dora-schema.json`) for editor autocompletion and validation.

## Quick Start

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

Run with `dora run dataflow.yml` (local mode) or `dora up && dora start dataflow.yml` (networked mode).

## Editor Setup

Add a schema comment at the top of your YAML file for VS Code autocompletion (requires the [YAML extension](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml)):

```yaml
# yaml-language-server: $schema=https://raw.githubusercontent.com/dora-rs/dora/main/dora-schema.json
nodes:
  - id: my-node
    # ... autocompletion works here
```

## Root-Level Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `nodes` | list | **required** | List of node configurations |
| `strict_types` | bool | `false` | Treat type warnings as errors in `validate` and `build` |
| `type_rules` | list | `[]` | User-defined type compatibility rules (see [Type Annotations](types.md#user-defined-compatibility-rules)) |
| `health_check_interval` | float | `5.0` | Seconds between daemon health check sweeps. For each node with `health_check_timeout` set, the daemon checks whether the node has communicated within its timeout; if not, the node is killed and its `restart_policy` is evaluated |
| `_unstable_deploy` | object | -- | Root-level deployment config (see [Deployment](#deployment)) |
| `_unstable_debug` | object | -- | Debug options (see [Debug](#debug)) |

## Node Configuration

Every node requires an `id`. All other fields are optional (though most nodes need at least `path` or `operator`/`operators`).

### Identity

| Field | Type | Description |
|-------|------|-------------|
| `id` | string | **Required.** Unique identifier. Must not contain `/`. Whitespace is discouraged |
| `name` | string | Human-readable display name (metadata only, used in tooling and logs) |
| `description` | string | Documentation string (metadata only, not used at runtime) |

### Source

A node's executable comes from a local path, a git repository, a module reference, or is implicit (operator/ROS2 nodes).

| Field | Type | Description |
|-------|------|-------------|
| `path` | string | Path to executable or script. Can also be a URL (legacy) |
| `module` | string | Path to a module definition file (mutually exclusive with `path`). See [Modules Guide](modules.md) |
| `git` | string | Git repo URL. `dora build` clones it and uses the clone dir as working directory |
| `branch` | string | Branch to checkout (requires `git`, mutually exclusive with `tag`/`rev`) |
| `tag` | string | Tag to checkout (requires `git`, mutually exclusive with `branch`/`rev`) |
| `rev` | string | Commit hash to checkout (requires `git`, mutually exclusive with `branch`/`tag`) |
| `build` | string | Build commands run during `dora build`. Each line runs separately. `pip`/`pip3` lines use `uv` when `--uv` is passed |
| `args` | string | Command-line arguments (space-separated) |

Example with git source:

```yaml
- id: rust-node
  git: https://github.com/dora-rs/adora.git
  branch: main
  build: cargo build -p example-node --release
  path: target/release/example-node
```

### Data I/O

#### Inputs

Inputs subscribe to another node's output using the format `<node-id>/<output-id>`:

```yaml
inputs:
  # Short form
  image: camera/frames
  tick: dora/timer/millis/100

  # Long form with options
  sensor_data:
    source: sensor/frames
    queue_size: 10
    queue_policy: drop_oldest
    input_timeout: 5.0

  # Lossless input (blocks sender when full)
  commands:
    source: controller/cmd
    queue_size: 100
    queue_policy: backpressure
```

| Input option | Type | Default | Description |
|-------------|------|---------|-------------|
| `source` | string | **required** | `<node-id>/<output-id>` or timer path |
| `queue_size` | integer | `10` | Input buffer size |
| `queue_policy` | string | `drop_oldest` | `drop_oldest`: drops oldest message when full. `backpressure`: buffers up to 10x `queue_size` without dropping (drops with ERROR log at hard cap) |
| `input_timeout` | float | -- | Circuit breaker timeout in seconds. If no message arrives within this period, the daemon closes the input and the node receives an `InputClosed` event for graceful degradation |

#### Built-in Timers

Timers are virtual nodes that emit ticks at fixed intervals:

```yaml
inputs:
  tick: dora/timer/millis/100   # every 100ms
  slow: dora/timer/millis/1000  # every 1s
  fast: dora/timer/hz/30        # 30 Hz (~33ms)
```

#### Built-in Log Aggregation

Subscribe to structured log messages from all (or filtered) nodes:

```yaml
inputs:
  all_logs: dora/logs               # all nodes, all levels
  errors:   dora/logs/error         # error+ from all nodes
  sensor:   dora/logs/info/sensor   # info+ from specific node
```

Each message arrives as a JSON-encoded `LogMessage` string. See [Logging](logging.md#doralogs----automatic-log-aggregation) for details.

#### Outputs

A list of output identifiers the node produces:

```yaml
outputs:
  - processed_image
  - metadata
```

### Type Annotations

Optional type annotations for inputs and outputs. Types are never required -- unannotated ports remain fully dynamic.

```yaml
- id: camera
  path: camera.py
  outputs:
    - image
    - depth
  output_types:
    image: std/media/v1/Image
    depth: std/media/v1/Image

- id: detector
  path: detect.py
  inputs:
    image: camera/image
  input_types:
    image: std/media/v1/Image
  outputs:
    - bbox
  output_types:
    bbox: std/vision/v1/BoundingBox
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `output_types` | object | `{}` | Maps output IDs to type URNs. Keys must match entries in `outputs` |
| `input_types` | object | `{}` | Maps input IDs to expected type URNs. Keys must match entries in `inputs` |
| `output_metadata` | object | `{}` | Maps output IDs to lists of required metadata keys |
| `pattern` | string | -- | Communication pattern shorthand: `service-server`, `service-client`, `action-server`, `action-client` |

Type URNs use the format `std/<category>/v<version>/<TypeName>` and support parameters (e.g. `std/media/v1/AudioFrame[sample_type=f32]`). See the [Type Annotations Guide](types.md) for the full standard type library, parameterized types, compatibility rules, and user-defined types.

Run `dora validate <file>` to check type annotations statically. For runtime checking, set `DORA_RUNTIME_TYPE_CHECK=warn` or `error`:

```bash
dora validate dataflow.yml
DORA_RUNTIME_TYPE_CHECK=warn dora run dataflow.yml
```

Types also appear on `dora graph` edge labels when annotated.

### Module Parameters

When using `module:`, pass configuration values via `params:`:

```yaml
- id: fast_pipeline
  module: modules/transform.module.yml
  inputs:
    data: sender/value
  params:
    speed: "2.0"
    mode: turbo
```

Inside the module, params are available as `$PARAM_<UPPERCASE_KEY>` in `args:` and as environment variables. See the [Modules Guide](modules.md) for full documentation.

### Environment

```yaml
env:
  MY_VAR: "value"          # string
  DEBUG: true               # boolean
  PORT: 8080                # integer
  RATE: 1.5                 # float
  FROM_HOST:
    __dora_env: HOST_VAR   # read from host environment at runtime
```

Environment variables apply to both `build` commands and node execution. Values support `$VAR` expansion syntax.

### Logging

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `send_stdout_as` | string | -- | Route raw stdout/stderr lines as a data output. Each line is sent as a separate Arrow message |
| `send_logs_as` | string | -- | Route structured log entries as a data output. Each entry is a JSON string with fields: `timestamp`, `level`, `node_id`, `message`, `target`, `fields` |
| `min_log_level` | string | -- | Suppress logs below this level from file output, coordinator forwarding, and `send_logs_as`. Levels from most to least verbose: `stdout` (all output including raw stdout), `trace`, `debug`, `info`, `warn`, `error` |
| `max_log_size` | string | -- | Rotate log file at this size (e.g. `"50MB"`, `"1GB"`) |
| `max_rotated_files` | integer | `5` | Number of rotated log files to keep |

Example:

```yaml
- id: sensor
  path: ./sensor
  min_log_level: info
  send_stdout_as: raw_output
  send_logs_as: log_entries
  max_log_size: "100MB"
  max_rotated_files: 3
  outputs:
    - data
    - raw_output
    - log_entries
```

When using `send_stdout_as` or `send_logs_as`, include the output name in the `outputs` list so downstream nodes can subscribe to it.

For a complete guide to all logging features, see [Logging](logging.md).

### Fault Tolerance

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `restart_policy` | string | `never` | `never`, `on-failure`, or `always` |
| `max_restarts` | integer | `0` | Max restart attempts. 0 = unlimited |
| `restart_delay` | float | -- | Initial backoff in seconds. Doubles each attempt |
| `max_restart_delay` | float | -- | Cap for exponential backoff |
| `restart_window` | float | -- | Time window for counting restarts. The counter resets after this many seconds since the first restart in the current window. Enables "N restarts per M seconds" semantics with `max_restarts` |
| `health_check_timeout` | float | -- | If the node does not communicate with the daemon (send outputs, subscribe, etc.) for this many seconds, the daemon kills the process and evaluates the `restart_policy` |

Restart policies:

- **`never`** (default): no automatic restart
- **`on-failure`**: restart only on non-zero exit code
- **`always`**: restart on any exit, except when stopped by user or all inputs closed with success

Example with exponential backoff:

```yaml
- id: sensor
  path: ./sensor
  restart_policy: on-failure
  max_restarts: 5
  restart_delay: 1.0         # 1s, 2s, 4s, 8s, 16s
  max_restart_delay: 30.0    # capped at 30s
  restart_window: 300.0      # 5 restarts per 5 minutes
  health_check_timeout: 30.0
```

### Arrow IPC Framing

By default, outputs use a raw Arrow buffer layout (zero-copy, no schema overhead). For self-describing wire format with full schema metadata, enable Arrow IPC framing per output:

```yaml
- id: sensor
  path: ./sensor
  outputs:
    - image
  output_framing:
    image: arrow-ipc
```

When `arrow-ipc` framing is set, data is serialized as Arrow IPC stream format (schema + record batches). Receivers automatically detect the framing mode via a `_framing` metadata key and decode accordingly. This is useful when consumers need schema introspection or when interoperating with external Arrow tools.

Values: `raw` (default) or `arrow-ipc`.

### CPU Affinity

Pin a node's process to specific CPU cores (Linux only, ignored on other platforms):

```yaml
- id: realtime-controller
  path: ./controller
  cpu_affinity: [0, 1]
  inputs:
    sensor: sensor/data
```

The daemon applies `sched_setaffinity` before exec. Core indices must be less than the system's `CPU_SETSIZE` (typically 1024). Out-of-range cores are skipped with a warning.

### Deployment

Assign nodes to specific machines using `_unstable_deploy`:

```yaml
- id: camera-driver
  _unstable_deploy:
    machine: robot-arm
  path: ./target/debug/camera
  outputs:
    - frames

- id: ml-inference
  _unstable_deploy:
    machine: gpu-server
    labels:
      gpu: "true"
    distribute: scp
  path: ./target/debug/inference
  inputs:
    frames: camera-driver/frames
```

| Deploy field | Type | Default | Description |
|-------------|------|---------|-------------|
| `machine` | string | -- | Target machine/daemon ID. The coordinator routes the node to the daemon registered with this ID |
| `working_dir` | string | -- | Working directory on the target machine |
| `labels` | object | -- | Key-value labels for scheduling. The coordinator matches these against labels reported by each daemon at registration |
| `distribute` | string | `local` | How built binaries reach the target daemon: **`local`** -- each daemon builds from source independently; **`scp`** -- CLI pushes the built binary via SSH/SCP before spawn; **`http`** -- daemon pulls the binary from the coordinator's HTTP artifact store |

When nodes are on different machines, communication automatically switches from shared memory to Zenoh pub/sub.

## Operator Nodes

Operators run in-process inside a shared runtime (no separate process). Use `operator` for a single operator or `operators` for multiple.

### Single Operator

The `id` field is optional for single operators (defaults to the node `id`):

```yaml
- id: detector
  operator:
    python: detect.py
    build: pip install -r requirements.txt
    inputs:
      image: camera/frames
    outputs:
      - bbox
```

### Multiple Operators

Each operator in `operators` **requires** a unique `id`:


```yaml
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

### Operator Source Types

| Field | Description |
|-------|-------------|
| `python` | Python script path, or `{source: "script.py", conda_env: "myenv"}` |
| `shared-library` | Path to a shared library (`.so`/`.dylib`/`.dll`) |

Operators also support `inputs`, `outputs`, `build`, `send_stdout_as`, `send_logs_as`, `min_log_level`, `max_log_size`, and `max_rotated_files` with the same semantics as node-level fields.

## ROS2 Bridge

Declare a node as a ROS2 bridge to automatically convert between ROS2 DDS messages and Dora's Arrow format. No custom code needed.

### Single Topic

```yaml
- id: camera_bridge
  ros2:
    topic: /camera/image_raw
    message_type: sensor_msgs/Image
    direction: subscribe
  outputs:
    - image
```

### Multiple Topics

```yaml
- id: robot_bridge
  ros2:
    topics:
      - topic: /camera/image_raw
        message_type: sensor_msgs/Image
        direction: subscribe
        output: image
      - topic: /cmd_vel
        message_type: geometry_msgs/Twist
        direction: publish
        input: velocity
    qos:
      reliable: true
  inputs:
    velocity: planner/cmd_vel
  outputs:
    - image
```

### Service Bridge

```yaml
- id: add_service
  ros2:
    service: /add_two_ints
    service_type: example_interfaces/AddTwoInts
    role: server
  inputs:
    request: client_node/request
  outputs:
    - response
```

### Action Bridge

```yaml
- id: nav_action
  ros2:
    action: /navigate
    action_type: nav2_msgs/NavigateToPose
    role: client
  inputs:
    goal: planner/goal
  outputs:
    - feedback
    - result
```

### QoS Configuration

QoS can be set at the bridge level (applies to all topics) or per-topic:

| QoS field | Type | Default | Description |
|-----------|------|---------|-------------|
| `reliable` | bool | `false` | Reliable vs best-effort transport |
| `durability` | string | `volatile` | `volatile` or `transient_local` |
| `liveliness` | string | `automatic` | `automatic`, `manual_by_participant`, `manual_by_topic` |
| `lease_duration` | float | infinity | Lease duration in seconds |
| `max_blocking_time` | float | -- | Max blocking time for reliable transport |
| `keep_last` | integer | `1` | History depth (KeepLast policy) |
| `keep_all` | bool | `false` | Use KeepAll history instead of KeepLast |

### Other ROS2 Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `namespace` | string | `/` | ROS2 namespace |
| `node_name` | string | node `id` | ROS2 node name |

## Debug

```yaml
_unstable_debug:
  publish_all_messages_to_zenoh: true
```

Required for `dora topic echo`, `dora topic hz`, and `dora topic info` commands.

## Communication Patterns

Dora supports four communication patterns built on top of the dataflow:

- **Topic** (default): pub/sub dataflow
- **Service**: request/reply via `request_id` metadata
- **Action**: goal/feedback/result via `goal_id`/`goal_status` metadata, with cancellation support
- **Streaming**: session/segment/chunk via `session_id`/`segment_id`/`seq`/`fin`/`flush` metadata, with queue flush for interruption

See [Communication Patterns](../../../docs/patterns.md) for details and examples.

## Full Example

```yaml
health_check_interval: 10.0

_unstable_debug:
  publish_all_messages_to_zenoh: true

nodes:
  - id: webcam
    operator:
      python: webcam.py
      inputs:
        tick: dora/timer/millis/100
      outputs:
        - image

  - id: detector
    operator:
      python: detect.py
      build: pip install ultralytics
      inputs:
        image: webcam/image
      outputs:
        - bbox

  - id: plotter
    operator:
      python: plot.py
      inputs:
        image: webcam/image
        bbox: detector/bbox

  - id: logger
    path: ./logger
    inputs:
      bbox: detector/bbox
    send_stdout_as: logs
    min_log_level: info
    restart_policy: on-failure
    max_restarts: 3
    outputs:
      - logs
```
