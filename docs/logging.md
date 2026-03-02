# Logging

Adora provides a structured logging system for real-time robotics and AI dataflows. Logs are captured per-node as structured JSONL files, forwarded to the coordinator for live streaming, and optionally routed through the dataflow graph as data messages.

## Features at a Glance

| Feature | Scope | Config |
|---------|-------|--------|
| Log level filtering | CLI display | `--log-level`, `ADORA_LOG_LEVEL` |
| Output formats | CLI display | `--log-format`, `ADORA_LOG_FORMAT` |
| Per-node level overrides | CLI display | `--log-filter`, `ADORA_LOG_FILTER` |
| Source-level filtering | Per-node YAML | `min_log_level` |
| Stdout-as-data routing | Per-node YAML | `send_stdout_as` |
| Structured log routing | Per-node YAML | `send_logs_as` |
| Log file rotation | Per-node YAML | `max_log_size` |
| Rotation file limit | Per-node YAML | `max_rotated_files` |
| Node log API | Rust/Python/C/C++ node | `node.log()`, `adora_log()`, etc. |
| Log utilities library | Rust crate | `adora-log-utils` |
| Time-range filtering | `adora logs` | `--since`, `--until` |
| Live log streaming | `adora logs` | `--follow` |
| Text search | `adora logs` | `--grep` |
| Local log reading | `adora logs` | `--local`, `--all-nodes` |

---

## Log File Format

Each node produces a JSONL file (one JSON object per line) at:

```
<working_dir>/out/<dataflow_uuid>/log_<node_id>.jsonl
```

Each line has this structure:

```json
{
  "timestamp": "2024-01-15T10:30:00.123Z",
  "level": "info",
  "node_id": "sensor",
  "message": "Starting sensor...",
  "target": "sensor::module",
  "fields": { "key": "value" }
}
```

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | string | RFC3339 timestamp with millisecond precision |
| `level` | string | `"error"`, `"warn"`, `"info"`, `"debug"`, `"trace"`, or `"stdout"` |
| `node_id` | string | Node ID |
| `message` | string | The log message text |
| `target` | string? | Rust module target (e.g. `"sensor::module"`), null if absent |
| `fields` | object? | Structured key-value fields from the logging framework |

### How Node Output Becomes Log Entries

The daemon captures each line of stdout/stderr from a node process and attempts to parse it as a structured log message (JSON with `level`, `message`, `timestamp`, and optional `fields`). If parsing succeeds, the structured fields are preserved. If parsing fails, the raw line becomes a `"stdout"`-level entry.

This means nodes using Rust's `tracing` or `log` crate with JSON output get full structured logging automatically. Nodes that simply `println!` produce `"stdout"`-level entries.

---

## Viewing Logs: `adora run`

When running a dataflow with `adora run`, logs from all nodes are displayed in real-time on the terminal.

### Flags

```
adora run dataflow.yml [OPTIONS]
```

| Flag | Default | Env Var | Description |
|------|---------|---------|-------------|
| `--log-level LEVEL` | `stdout` | `ADORA_LOG_LEVEL` | Minimum level to display |
| `--log-format FORMAT` | `pretty` | `ADORA_LOG_FORMAT` | Output format: `pretty`, `json`, `compact` |
| `--log-filter FILTER` | none | `ADORA_LOG_FILTER` | Per-node level overrides |

### Log Levels

From most to least verbose:

| Level | Description |
|-------|-------------|
| `stdout` | Everything including raw stdout from nodes (default) |
| `trace` | Fine-grained diagnostic messages |
| `debug` | Developer-level diagnostic messages |
| `info` | General informational messages |
| `warn` | Warning conditions |
| `error` | Error conditions only |

Setting `--log-level info` hides `stdout`, `trace`, and `debug` messages. The `stdout` level is a special catch-all that passes everything.

### Level Filtering Logic

The level filter uses `LogLevelOrStdout::passes()`:

```
Message level    Filter level    Displayed?
─────────────    ────────────    ──────────
stdout           stdout          yes
stdout           info            no       (stdout only passes stdout filter)
info             stdout          yes      (any log level passes stdout filter)
debug            info            no       (debug is more verbose than info)
error            info            yes      (error is less verbose than info)
```

### Per-Node Overrides

The `--log-filter` flag lets you set different levels for different nodes:

```bash
adora run dataflow.yml --log-level info --log-filter "sensor=debug,planner=warn"
```

This shows `info` and above for all nodes, except `sensor` (shows `debug` and above) and `planner` (shows `warn` and above).

Format: `"node1=level,node2=level"` (comma-separated `name=level` pairs).

### Output Formats

**Pretty** (default) -- colored, human-readable:
```
10:30:00 INFO   sensor: Starting sensor...

10:30:01 INFO   [adora]: spawning node processor

10:30:01 stdout sensor: raw output line
```

- Timestamp in local timezone (`HH:MM:SS`)
- Level colored: ERROR (red), WARN (yellow), INFO (green), DEBUG (blue), TRACE (dimmed), stdout (italic dimmed blue)
- Node name in bold with a unique color based on the name
- System messages prefixed with `[adora]`
- Lifecycle messages (`spawning`, `node finished`, `stopping`) get visual separation with blank lines

**Json** -- full `LogMessage` struct as JSON, one per line:
```json
{"build_id":null,"dataflow_id":"abc-123","node_id":"sensor","level":"INFO","message":"Starting...","timestamp":"2024-01-15T10:30:00Z",...}
```

Useful for piping to `jq` or ingesting into log aggregation systems.

**Compact** -- minimal, no color:
```
10:30:00 INFO sensor: Starting sensor...
```

Useful for CI/CD environments and log files.

---

## Viewing Logs: `adora logs`

Read historical logs or stream live logs from a running dataflow.

### Basic Usage

```bash
# Read logs for a specific node (via coordinator)
adora logs <dataflow_uuid> <node_name>

# Read local log files directly
adora logs --local <node_name>
adora logs --local --all-nodes

# Stream live logs
adora logs <dataflow_uuid> <node_name> --follow
adora logs --local <node_name> --follow
```

### Flags

| Flag | Short | Default | Description |
|------|-------|---------|-------------|
| `--local` | | false | Read from local `out/` directory instead of coordinator |
| `--all-nodes` | | false | Merge logs from all nodes, sorted by timestamp |
| `--tail N` | `-n` | all | Show only the last N lines |
| `--follow` | `-f` | false | Stream new log entries as they arrive |
| `--since DURATION` | | none | Only show logs newer than this duration ago |
| `--until DURATION` | | none | Only show logs older than this duration ago |
| `--level LEVEL` | | `stdout` | Minimum log level (env: `ADORA_LOG_LEVEL`) |
| `--grep PATTERN` | | none | Case-insensitive text search |
| `--coordinator-addr IP` | | `127.0.0.1` | Coordinator address |
| `--coordinator-port PORT` | | default | Coordinator control port |

### Time Filters

`--since` and `--until` accept duration strings relative to now:

```bash
# Logs from the last 5 minutes
adora logs --local sensor --since 5m

# Logs from 1 hour ago to 30 minutes ago
adora logs --local sensor --since 1h --until 30m

# Last 10 errors from the past hour
adora logs --local sensor --since 1h --level error --tail 10
```

Supported duration formats: `30` (seconds), `30s`, `5m`, `1h`, `2d`.

### Text Search

`--grep` performs case-insensitive substring matching against:
- The log message text
- The node ID
- The module target

```bash
# Find all timeout-related messages
adora logs --local --all-nodes --grep "timeout"

# Find errors from a specific module
adora logs --local sensor --grep "camera::driver" --level error
```

### Filter Pipeline

All filters are applied in this order:

```
Read/Parse -> Time Filters -> Grep -> Tail -> Display
```

When `--since`, `--until`, or `--grep` are used in coordinator mode, the CLI fetches all logs from the server (ignoring `--tail` server-side) and applies all filters client-side. This ensures correct results when combining filters.

### Local vs Coordinator Mode

**Local mode** (`--local`) reads JSONL files directly from the `out/` directory in the current working directory. No coordinator or daemon needs to be running. If `--all-nodes` is used or no node name is given, all log files are merged and sorted by timestamp.

**Coordinator mode** (default) connects to a running coordinator via WebSocket. The coordinator reads log files from the daemon's working directory and streams them back. This works for both local and distributed deployments.

### Follow Mode

**Local follow** (`--local --follow`): Polls log files every 200ms for new content. New lines are parsed, filtered by `--grep`, and printed. Time/tail filters only apply to the initial historical output.

**Coordinator follow** (`--follow`): Opens a WebSocket subscription to the coordinator. The coordinator forwards log messages from the daemon in real-time. Level filtering is applied server-side for efficiency. `--grep` and `--since` are applied client-side on the stream.

---

## Environment Variables

All environment variables serve as fallbacks -- CLI flags always take precedence.

| Variable | Used By | Values | Description |
|----------|---------|--------|-------------|
| `ADORA_LOG_LEVEL` | `adora run`, `adora logs` | `error`, `warn`, `info`, `debug`, `trace`, `stdout` | Default minimum log level |
| `ADORA_LOG_FORMAT` | `adora run` | `pretty`, `json`, `compact` | Default output format |
| `ADORA_LOG_FILTER` | `adora run` | `"node1=level,node2=level"` | Default per-node overrides |
| `ADORA_QUIET` | daemon | any value | Suppress log forwarding to display (file writing continues) |

Example:

```bash
# Set defaults for a development session
export ADORA_LOG_LEVEL=info
export ADORA_LOG_FORMAT=pretty
export ADORA_LOG_FILTER="sensor=debug"

# These are equivalent:
adora run dataflow.yml
adora run dataflow.yml --log-level info --log-format pretty --log-filter "sensor=debug"

# CLI flag overrides env var:
adora run dataflow.yml --log-level debug   # overrides ADORA_LOG_LEVEL=info
```

---

## YAML Configuration

### `min_log_level`

Filter logs at the source (daemon-side) before they reach log files, the coordinator, or `send_logs_as` routing.

```yaml
nodes:
  - id: noisy-sensor
    path: ./target/debug/sensor
    min_log_level: info    # suppress debug/trace/stdout from this node
```

Valid values: `error`, `warn`, `info`, `debug`, `trace`, `stdout`.

When set, the daemon drops log messages below this level immediately after parsing. This reduces disk I/O, network traffic, and log file size. The filtering uses the same `passes()` logic as the CLI display filter.

### `send_stdout_as`

Route raw stdout/stderr lines as dataflow output messages.

```yaml
nodes:
  - id: legacy-node
    path: ./legacy-script.py
    send_stdout_as: raw_output
    outputs:
      - raw_output
      - data

  - id: log-consumer
    inputs:
      logs: legacy-node/raw_output
```

Each stdout/stderr line is sent as an Arrow-encoded string. This is useful for integrating legacy nodes that output data on stdout (e.g., Python scripts using `print()`).

Both `send_stdout_as` and normal log file writing happen -- stdout routing does not suppress log files.

### `send_logs_as`

Route parsed structured log entries as dataflow output messages.

```yaml
nodes:
  - id: sensor
    path: ./target/debug/sensor
    send_logs_as: log_entries
    outputs:
      - data
      - log_entries

  - id: log-aggregator
    inputs:
      sensor_logs: sensor/log_entries
```

Unlike `send_stdout_as`, this only sends lines that were successfully parsed as structured logs (not raw stdout). Each entry is serialized as a full JSON `LogMessage` string. The `min_log_level` filter applies before routing -- suppressed messages are not sent.

Use this to build log aggregation, alerting, or monitoring nodes within the dataflow itself.

### `max_log_size`

Enable size-based log file rotation.

```yaml
nodes:
  - id: sensor
    path: ./target/debug/sensor
    max_log_size: "50MB"
```

| Value | Bytes |
|-------|-------|
| `"1KB"` or `"1K"` | 1,024 |
| `"50MB"` or `"50M"` | 52,428,800 |
| `"1GB"` or `"1G"` | 1,073,741,824 |
| `"1000"` | 1,000 (plain number = bytes) |

When the active log file exceeds the configured size, the daemon:

1. Flushes and closes the current file
2. Renames existing rotated files: `.4.jsonl` -> `.5.jsonl`, `.3.jsonl` -> `.4.jsonl`, etc.
3. Renames the current file: `log_sensor.jsonl` -> `log_sensor.1.jsonl`
4. Creates a fresh `log_sensor.jsonl`
5. Deletes any file beyond the rotation limit (default 5, configurable via `max_rotated_files`)

**Naming convention:**

```
log_sensor.jsonl       # current (active)
log_sensor.1.jsonl     # previous
log_sensor.2.jsonl     # older
log_sensor.3.jsonl
log_sensor.4.jsonl
log_sensor.5.jsonl     # oldest (deleted on next rotation)
```

Maximum disk usage per node: `max_log_size * (1 + max_rotated_files)` (1 active + N rotated).

Without `max_log_size`, log files grow unbounded. For long-running dataflows, always set this.

The `adora logs --local` command automatically reads all rotated files for a node and merges them in chronological order (oldest rotated file first, current file last).

### `max_rotated_files`

Control how many rotated log files to keep (default: 5, range: 1-100).

```yaml
nodes:
  - id: sensor
    path: ./target/debug/sensor
    max_log_size: "50MB"
    max_rotated_files: 10    # keep 10 rotated files instead of 5
```

With `max_rotated_files: 10` and `max_log_size: "50MB"`, maximum disk usage is `50MB * 11` = 550MB per node. Lower values save disk space; higher values preserve more history.

### Runtime Node Restrictions

For runtime nodes (operators), only one of each logging field is allowed per runtime:

```yaml
# OK -- single operator
nodes:
  - id: runtime-node
    operator:
      python: process.py
      send_logs_as: logs
      min_log_level: info
      max_log_size: "100MB"

# ERROR -- multiple operators with conflicting configs
nodes:
  - id: runtime-node
    operators:
      - id: op1
        python: a.py
        send_logs_as: logs1
      - id: op2
        python: b.py
        send_logs_as: logs2    # Error: multiple send_logs_as
```

When a single operator in a runtime sets these fields, the output name is prefixed with the operator ID (e.g., `op1/logs`).

---

## Node Log API

Nodes can emit structured log messages programmatically using the node API. These are equivalent to writing JSON-formatted log lines to stdout -- the daemon parses them identically.

### Rust

```rust
use adora_node_api::AdoraNode;
use std::collections::BTreeMap;

let (node, mut events) = AdoraNode::init_from_env()?;

// General log with level string and optional target
node.log("info", "sensor initialized", Some("sensor::init"));

// Convenience methods (no target parameter)
node.log_error("connection failed");
node.log_warn("temperature elevated");
node.log_info("reading acquired");
node.log_debug("raw bytes received");
node.log_trace("entering loop iteration");

// Structured fields (key-value context preserved through send_logs_as)
let mut fields = BTreeMap::new();
fields.insert("sensor_id".to_string(), "temp-01".to_string());
fields.insert("reading".to_string(), "42.5".to_string());
node.log_with_fields("info", "reading acquired", None, Some(&fields));
```

The `level` parameter accepts `"error"`, `"warn"` (or `"warning"`), `"info"`, `"debug"`, `"trace"`. Unknown levels default to `"info"`. Fields are capped at 60 KB total to match the downstream 64 KB parse limit.

### Python

```python
from adora import Node

node = Node()

# General log with level string and optional target
node.log("info", "sensor initialized", target="sensor.init")

# Structured fields
node.log("info", "reading acquired", fields={"sensor_id": "temp-01", "reading": "42.5"})

# For most cases, use Python's built-in logging module instead:
import logging
logging.info("sensor initialized")
```

The Python `node.log()` method has the same level normalization as Rust. However, Python nodes typically use the standard `logging` module, which the daemon parses into structured log entries automatically.

### C

```c
#include "node_api.h"

void *ctx = init_adora_context_from_env();
const char *level = "info";
const char *msg = "sensor initialized";
adora_log(ctx, level, strlen(level), msg, strlen(msg));
```

### C++

```cpp
// Via the cxx bridge
auto node = init_adora_node();
log_message(node.send_output, "info", "sensor initialized");
```

---

## Log Utilities Library (`adora-log-utils`)

The `adora-log-utils` crate provides parsing, merging, filtering, and formatting utilities for working with `LogMessage` entries in custom sink nodes. Use it when building nodes that consume log data via `send_logs_as`.

### API

```rust
use adora_log_utils;

// Parse a LogMessage from JSON (as received from send_logs_as)
let log = adora_log_utils::parse_log(json_str)?;

// Parse directly from Arrow input data (convenience for event handlers)
let log = adora_log_utils::parse_log_from_arrow(&data)?;

// Merge multiple log streams into a single timeline
let merged = adora_log_utils::merge_by_timestamp(vec![stream_a, stream_b]);

// Filter by minimum level
let errors = adora_log_utils::filter_by_level(&logs, &min_level);

// Format as JSON (one line, no trailing newline)
let json = adora_log_utils::format_json(&log);

// Format as compact single-line: "<timestamp> <node> <LEVEL>: <message>"
let compact = adora_log_utils::format_compact(&log);

// Format as pretty: "[<timestamp>][<LEVEL>][<node>] <message>"
let pretty = adora_log_utils::format_pretty(&log);
```

### Dependency

Add to your sink node's `Cargo.toml`:

```toml
[dependencies]
adora-log-utils = { workspace = true }
```

---

## Log Sink Examples

Three example sink nodes demonstrate how to consume logs routed via `send_logs_as` and forward them to external destinations.

### File Sink (`examples/log-sink-file/`)

Merges log streams from multiple nodes into a single JSONL file. Useful for unified log collection.

```yaml
nodes:
  - id: sensor
    path: sensor.py
    send_logs_as: log_entries
    inputs:
      tick: adora/timer/millis/200
    outputs:
      - reading
      - log_entries

  - id: processor
    path: processor.py
    send_logs_as: log_entries
    inputs:
      reading: sensor/reading
    outputs:
      - result
      - log_entries

  - id: file_sink
    path: log-sink-file
    inputs:
      sensor_logs: sensor/log_entries
      processor_logs: processor/log_entries
    env:
      LOG_FILE: "./combined.jsonl"
```

The file sink reads `LOG_FILE` from the environment (default `./combined.jsonl`), parses each incoming Arrow message with `adora_log_utils::parse_log_from_arrow()`, formats it as JSON, and appends it to the file.

### TCP Sink (`examples/log-sink-tcp/`)

Forwards log entries over a TCP socket to a remote log collector. Useful for embedded systems that lack local filesystems and need to stream logs off-device.

```yaml
nodes:
  - id: source
    path: source.py
    send_logs_as: log_entries
    inputs:
      tick: adora/timer/millis/500
    outputs:
      - data
      - log_entries

  - id: tcp_sink
    path: log-sink-tcp
    inputs:
      logs: source/log_entries
    env:
      SINK_ADDR: "127.0.0.1:9876"
```

The TCP sink reads `SINK_ADDR` from the environment (default `127.0.0.1:9876`), connects to the server on startup, and sends each log entry as a JSON line. It reconnects automatically on write failure.

### Alert Router (`examples/log-sink-alert/`)

Splits incoming log entries by severity. All logs are forwarded to the `all_logs` output; only error and warn logs are forwarded to the `alerts` output. This enables downstream nodes to handle alerts differently (e.g., trigger notifications, write to a dedicated file).

```yaml
nodes:
  - id: source
    path: my_node.py
    send_stdout_as: log_entries
    inputs:
      tick: adora/timer/millis/200
    outputs:
      - log_entries

  - id: alert_router
    path: log-sink-alert
    inputs:
      logs: source/log_entries
    outputs:
      - all_logs
      - alerts
```

The source node uses `send_stdout_as` to route its stdout lines as Arrow string data. The router parses each log entry with `adora_log_utils::parse_log_from_arrow()`, checks the level, and uses `node.send_output()` to forward data to the appropriate outputs. Nodes using the node API can alternatively use `send_logs_as` to route structured logs from `node.log()`.

### Building a Custom Sink

To build your own sink node, follow this pattern:

```rust
use adora_node_api::{AdoraNode, Event};

fn main() -> eyre::Result<()> {
    let (_node, mut events) = AdoraNode::init_from_env()?;

    while let Some(event) = events.recv() {
        match event {
            Event::Input { data, .. } => {
                let log = adora_log_utils::parse_log_from_arrow(&data)?;
                // Process the log entry: write to file, send over network, etc.
                let json = adora_log_utils::format_json(&log);
                println!("{json}");
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }
    Ok(())
}
```

---

## How the Daemon Processes Logs

Understanding the internal pipeline helps with debugging and tuning. For each node, the daemon runs a dedicated async task that processes log lines in order:

```
Node Process (stdout/stderr)
    |
    v
[1] Capture: lines buffered in mpsc channel (capacity 10)
    |
    v
[2] send_stdout_as: raw line -> Arrow data -> dataflow output
    |
    v
[3] Parse: try JSON structured log, fall back to Stdout-level
    |
    v
[4] min_log_level filter: drop messages below threshold
    |
    v
[5] send_logs_as: LogMessage -> JSON -> Arrow data -> dataflow output
    |
    v
[6] Write JSONL: compact format to log file, track bytes written
    |
    v
[7] Rotation check: if bytes_written >= max_log_size, rotate files
    |
    v
[8] Forward: send LogMessage to display channel (unless ADORA_QUIET)
    |
    v
[9] Sync: fsync log file to disk
```

Key details:

- **Step 2** happens before parsing, so `send_stdout_as` captures every line including non-structured output
- **Step 4** happens before Steps 5-8, so `min_log_level` suppresses messages from all downstream processing
- **Step 5** only fires for successfully parsed structured logs (Step 3 success path)
- **Step 8** sends to either a flume channel (`adora run` direct mode) or the coordinator (distributed mode)
- **Step 9** calls `sync_all()` after every write, ensuring durability at the cost of some I/O overhead

### Structured Log Parsing

When a node emits JSON-formatted log output (e.g., from `tracing-subscriber` with JSON formatting), the daemon extracts:

- `level`: log severity
- `message`: the log text
- `target`: module path
- `timestamp`: when the log was emitted
- `fields`: arbitrary key-value pairs
- `build_id`, `dataflow_id`, `node_id`, `daemon_id`: extracted from fields as fallback

The daemon also sets `dataflow_id`, `node_id`, and `daemon_id` on all messages to ensure they are always present in the log file.

---

## Coordinator Log Streaming Protocol

When a daemon runs under a coordinator (distributed mode), log forwarding works via WebSocket:

1. **Daemon -> Coordinator**: Each `LogMessage` is wrapped in `DaemonEvent::Log(message)` and sent over the daemon's WebSocket connection
2. **Coordinator storage**: The coordinator stores/forwards logs
3. **CLI subscription**: The CLI sends `ControlRequest::LogSubscribe { dataflow_id, level }` over its WebSocket connection
4. **Server-side filtering**: The coordinator only forwards messages where `msg_level <= subscription_level`. This reduces network traffic for filtered subscriptions
5. **CLI receive**: Messages arrive as serialized `LogMessage` structs

The `--level` flag maps to `log::LevelFilter`:
- `stdout` -> `LevelFilter::Trace` (most permissive, receives everything)
- `info` -> `LevelFilter::Info` (receives Error, Warn, Info)
- etc.

---

## Complete YAML Reference

```yaml
nodes:
  - id: sensor
    path: ./target/debug/sensor
    outputs:
      - data
      - raw_output       # for send_stdout_as
      - log_entries       # for send_logs_as

    # Source-level log filtering (daemon-side)
    min_log_level: info          # suppress debug/trace/stdout

    # Route stdout to dataflow
    send_stdout_as: raw_output   # every stdout line becomes a data message

    # Route structured logs to dataflow
    send_logs_as: log_entries    # parsed log entries become data messages

    # Log file rotation
    max_log_size: "50MB"         # rotate when file exceeds 50MB
    max_rotated_files: 5         # keep 5 rotated files (default, range 1-100)

    inputs:
      tick: adora/timer/millis/100
```

---

## Complete Example

The `examples/python-logging/` directory contains a runnable three-node pipeline that exercises every logging feature:

```
sensor (noisy, high-volume) --> processor (structured logs) --> monitor (log aggregator)
```

**Dataflow configuration highlights:**

```yaml
nodes:
  - id: sensor
    path: sensor.py
    min_log_level: info       # suppress debug noise at source
    max_log_size: "1KB"       # small for demo (triggers rotation quickly)
    inputs:
      tick: adora/timer/millis/50
    outputs:
      - reading

  - id: processor
    path: processor.py
    send_logs_as: log_entries  # route structured logs as data
    inputs:
      reading: sensor/reading
    outputs:
      - result
      - log_entries

  - id: monitor
    path: monitor.py
    inputs:
      logs: processor/log_entries
      reading: sensor/reading
```

**What each node demonstrates:**

- **sensor** -- Mixes `print()` (raw stdout), `logging.info()`, `logging.debug()`, and `logging.warning()`. With `min_log_level: info`, debug messages are dropped by the daemon before reaching log files. With `max_log_size: "1KB"`, log rotation kicks in after a few seconds.
- **processor** -- Uses `send_logs_as: log_entries` to route its structured log entries as dataflow data. Raw `print()` output is *not* routed (only parsed structured entries are).
- **monitor** -- Subscribes to `processor/log_entries` and counts warnings/errors, demonstrating in-dataflow log aggregation.

**Direct mode** (`adora run` -- single process, good for quick testing):

```bash
# Basic run
adora run examples/python-logging/dataflow.yml --stop-after 5s

# Only warnings and above
adora run examples/python-logging/dataflow.yml --log-level warn --stop-after 5s

# Per-node overrides
adora run examples/python-logging/dataflow.yml --log-filter "monitor=debug,sensor=warn" --stop-after 5s

# JSON output for machine parsing
adora run examples/python-logging/dataflow.yml --log-format json --stop-after 3s

# Environment variable control
ADORA_LOG_LEVEL=warn adora run examples/python-logging/dataflow.yml --stop-after 5s
```

**Distributed mode** (`adora up` + `adora start` -- coordinator/daemon architecture, required for multi-machine deployments):

```bash
# Start infrastructure
adora up

# Start attached (live log stream)
adora start examples/python-logging/dataflow.yml --attach

# Or start detached and query logs separately
adora start examples/python-logging/dataflow.yml
adora logs <dataflow-id> sensor --follow                    # stream one node
adora logs <dataflow-id> sensor --follow --level warn       # only warnings
adora logs <dataflow-id> --all-nodes --tail 20              # last 20 lines
adora logs <dataflow-id> processor --grep "error" --since 5m  # targeted search
```

In distributed mode, logs flow Node -> Daemon -> Coordinator -> CLI over WebSocket. The coordinator buffers log messages until a subscriber connects, so you won't miss logs even if you attach late. YAML-level settings (`min_log_level`, `send_logs_as`, `max_log_size`) work identically since they are applied at the daemon.

| | `adora run` | `adora start` |
|---|---|---|
| Display filtering | `--log-level`, `--log-format`, `--log-filter` | `--level` on `adora logs` |
| Per-node overrides | `--log-filter "sensor=debug"` | Separate `adora logs` per node |
| Remote nodes | No | Yes |
| Live streaming | Always attached | `--attach` or `adora logs --follow` |

**Post-run log analysis** (works the same for both modes):

```bash
# Read all local logs
adora logs --local --all-nodes --tail 20

# Search for warnings in sensor logs
adora logs --local sensor --grep "high temp"

# Check that rotation created multiple files
ls -la out/*/log_sensor*.jsonl
```

---

## Use Case Scenarios

### 1. Debugging a Noisy Sensor Pipeline

A camera sensor node floods the logs with debug messages, making it hard to see errors from other nodes.

```yaml
nodes:
  - id: camera
    path: ./target/debug/camera
    min_log_level: warn          # suppress info/debug/trace at the source
    max_log_size: "10MB"         # limit disk usage

  - id: detector
    path: ./target/debug/detector

  - id: planner
    path: ./target/debug/planner
```

```bash
# During development: see everything from detector, only warnings from camera
adora run dataflow.yml --log-level debug --log-filter "camera=warn,detector=debug"

# In production: only errors
export ADORA_LOG_LEVEL=error
adora run dataflow.yml
```

**What happens:**
- Camera node's debug/info messages are dropped by the daemon before reaching the log file (`min_log_level: warn`)
- The CLI further filters display based on `--log-filter`
- Log files rotate at 10MB, keeping at most 60MB on disk for the camera node

### 2. Log Aggregation Within the Dataflow

Build an in-dataflow log monitoring node that watches for errors across multiple nodes and sends alerts.

```yaml
nodes:
  - id: camera
    path: ./target/debug/camera
    send_logs_as: logs
    outputs:
      - frames
      - logs

  - id: detector
    path: ./target/debug/detector
    send_logs_as: logs
    outputs:
      - detections
      - logs

  - id: log-monitor
    path: ./target/debug/log-monitor
    inputs:
      camera_logs: camera/logs
      detector_logs: detector/logs
    outputs:
      - alerts
```

**Node-side handling in the log monitor (using `adora-log-utils`):**

```rust
use adora_node_api::{AdoraNode, Event};
use adora_message::common::{LogLevel, LogLevelOrStdout};

let (mut node, mut events) = AdoraNode::init_from_env()?;
while let Some(event) = events.recv() {
    match event {
        Event::Input { data, .. } => {
            let log = adora_log_utils::parse_log_from_arrow(&data)?;

            let is_error = matches!(log.level,
                LogLevelOrStdout::LogLevel(LogLevel::Error));

            if is_error || log.message.contains("timeout") {
                // Send alert downstream
                node.send_output("alerts", /* ... */)?;
            }
        }
        Event::Stop(_) => break,
        _ => {}
    }
}
```

See also the [Log Sink Examples](#log-sink-examples) section for complete runnable examples.

### 3. Post-Mortem Debugging of a Crash

After a dataflow crashes, investigate what happened in the last few minutes.

```bash
# Find available dataflows
ls out/

# Read the last 50 lines from all nodes around the crash
adora logs --local --all-nodes --tail 50

# Focus on errors in the last 5 minutes
adora logs --local --all-nodes --since 5m --level error

# Search for a specific error pattern
adora logs --local --all-nodes --grep "out of memory"

# Drill into a specific node
adora logs --local detector --since 2m

# Export as JSON for external analysis
adora run dataflow.yml --log-format json 2>logs.json
```

### 4. Long-Running Production Dataflow

A dataflow runs for days or weeks. Without log rotation, disk space fills up.

```yaml
nodes:
  - id: ingest
    path: ./target/debug/ingest
    min_log_level: info        # no debug noise in production
    max_log_size: "100MB"      # ~600MB max per node (100MB * 6)
    restart_policy: always
    inputs:
      tick: adora/timer/millis/1000
    outputs:
      - data

  - id: processor
    path: ./target/debug/processor
    min_log_level: warn        # only warnings and errors
    max_log_size: "50MB"
    restart_policy: on-failure
    inputs:
      data: ingest/data
    outputs:
      - results

  - id: writer
    path: ./target/debug/writer
    min_log_level: error       # minimal logging
    max_log_size: "20MB"
    inputs:
      results: processor/results
```

**Disk budget:**
- `ingest`: up to 600MB (100MB x 6 files)
- `processor`: up to 300MB (50MB x 6 files)
- `writer`: up to 120MB (20MB x 6 files)
- **Total**: ~1GB maximum disk usage for all logs

### 5. Live Monitoring of a Distributed Deployment

Multiple daemons running on different machines, monitored from a central workstation.

```bash
# Start infrastructure (coordinator + local daemon)
adora up

# On remote machines, start a daemon pointing to the coordinator:
#   adora daemon --coordinator-addr 192.168.1.10

# Start the dataflow (detached)
adora start dataflow.yml

# Open targeted log streams in separate terminals:

# Terminal 1: all sensor warnings
adora logs <dataflow-id> sensor --follow --level warn

# Terminal 2: processor errors with text search
adora logs <dataflow-id> processor --follow --level error --grep "timeout"

# Terminal 3: all nodes merged
adora logs <dataflow-id> --all-nodes --follow

# Terminal 4: historical + live (errors from the last hour, then stream)
adora logs <dataflow-id> processor --since 1h --level error --follow

# Monitor a remote coordinator from another machine:
adora logs <dataflow-id> sensor --follow --coordinator-addr 192.168.1.10
```

**How it works internally:**
1. CLI connects to the coordinator (default `localhost:6013`, or `--coordinator-addr`)
2. For historical logs: request-reply with filters applied client-side (`--since`, `--grep`, `--tail`)
3. For `--follow`: opens a WebSocket subscription to the coordinator
4. Coordinator filters by `--level` server-side before forwarding (reduces network traffic)
5. CLI applies `--grep` and `--since` client-side on the live stream
6. Coordinator buffers log messages until a subscriber connects, so late-joining subscribers see recent history

### 6. CI/CD Pipeline with Structured Logging

In CI, use JSON format for machine-parseable output and compact format for readable logs.

```bash
# Machine-parseable logs for CI tooling
adora run dataflow.yml --log-format json --stop-after 30s 2>test-logs.json

# Compact logs for CI console output
adora run dataflow.yml --log-format compact --log-level info --stop-after 30s

# Post-run analysis: count errors per node
adora logs --local --all-nodes --level error | wc -l
```

With JSON format, each line is a complete `LogMessage` that can be processed by `jq`, log aggregators, or custom scripts:

```bash
# Extract error messages with jq
cat test-logs.json | jq -r 'select(.level == "ERROR") | "\(.node_id): \(.message)"'
```

---

## Best Practices

**Set `min_log_level` in production.** Source-level filtering at the daemon prevents debug noise from reaching log files and the network. This is the most effective way to reduce log volume since it filters before any I/O.

**Always set `max_log_size` for long-running dataflows.** Without rotation, a single noisy node can fill the disk. Start with `"50MB"` (300MB total per node with rotation) and adjust based on your storage budget. Use `max_rotated_files` to tune how much history to keep (default 5, range 1-100).

**Use environment variables for team defaults.** Set `ADORA_LOG_LEVEL` and `ADORA_LOG_FORMAT` in your shell profile or CI configuration. Individual developers can override with CLI flags.

**Use `--log-filter` during development.** Instead of changing YAML config, use per-node display overrides to focus on the node you're debugging: `--log-filter "my-node=debug"`.

**Use `send_logs_as` for operational monitoring.** Build monitoring nodes that watch for error patterns, compute error rates, or forward alerts. This keeps monitoring logic within the dataflow graph. Use `adora-log-utils` to parse and format log entries in custom sink nodes (see `examples/log-sink-file/` and `examples/log-sink-tcp/`).

**Prefer `send_logs_as` over `send_stdout_as` for structured data.** `send_stdout_as` captures every stdout line (including raw prints), while `send_logs_as` only captures parsed structured log entries with full metadata.

**Use `--local` for post-mortem debugging.** After a crash, `adora logs --local --all-nodes` works without a running coordinator and merges all node logs chronologically.

**Combine `--since` with `--grep` for targeted debugging.** Instead of scrolling through thousands of lines, narrow the window: `adora logs --local sensor --since 5m --grep "error"`.

**Use JSON format for log pipelines.** When feeding logs to external systems (ELK, Grafana Loki, Datadog), use `--log-format json` for structured ingestion.
