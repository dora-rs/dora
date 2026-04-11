# Log Aggregator Example

Demonstrates `dora/logs`, a virtual input that automatically aggregates structured log messages from all nodes in the dataflow -- no manual wiring needed.

## Architecture

```
sensor (readings + logs) ----> processor (transforms + logs)
                                        |
                            [daemon collects all logs]
                                        |
                                   log-viewer
                              (dora/logs virtual input)
```

## Nodes

| Node | Role | Logging |
|------|------|---------|
| `sensor` | Emits temperature readings on a timer | `node.log_info()`, `node.log_warn()` |
| `processor` | Converts Celsius to Fahrenheit | `node.log_debug()` |
| `log-viewer` | Receives **all** logs via `dora/logs` | Parses and displays JSON log messages |

## Key Concepts

### `dora/logs` Virtual Input

Works like `dora/timer` -- the daemon handles subscription internally. Each message is a JSON-encoded `LogMessage` string in an Arrow array.

```yaml
inputs:
  logs: dora/logs              # all nodes, all levels
  errors: dora/logs/error      # error+ from all nodes
  sensor: dora/logs/info/sensor  # info+ from a specific node
```

A node **never receives its own logs** (prevents infinite loops).

### vs `send_logs_as`

| | `dora/logs` | `send_logs_as` |
|--|-------------|---------------|
| Scope | All nodes at once | One node at a time |
| YAML changes | Only the consumer | Each source node |
| Adding a node | Zero wiring changes | Must update consumer |
| Use case | Dashboard, monitoring | Per-node log processing |

## Running

```bash
# Local mode (quick test)
dora run dataflow.yml --stop-after 5s

# With level filtering
dora run dataflow.yml --log-level info --stop-after 5s

# Distributed mode
dora up
dora start dataflow.yml --attach
```

## Filter Syntax

| Input | Description |
|-------|-------------|
| `dora/logs` | All logs from all nodes |
| `dora/logs/<level>` | Logs at `<level>` or above from all nodes |
| `dora/logs/<level>/<node-id>` | Logs at `<level>` or above from one node |

Levels (most to least verbose): `stdout`, `trace`, `debug`, `info`, `warn`, `error`.

## Consuming Logs in Python

```python
import json
from dora import Node

node = Node()
for event in node:
    if event["type"] == "INPUT" and event["id"] == "logs":
        raw = bytes(event["value"]).decode("utf-8")
        log = json.loads(raw)
        print(f"[{log['level']}][{log['node_id']}] {log['message']}")
```

## Consuming Logs in Rust

```rust
use dora_log_utils;

// In your event loop:
let log = dora_log_utils::parse_log_from_arrow(&data)?;
println!("{}", dora_log_utils::format_pretty(&log));
```

Add `dora-log-utils = { workspace = true }` to your `Cargo.toml`.
