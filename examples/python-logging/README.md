# Python Logging Example

Demonstrates all dora logging features using a three-node pipeline:

```
sensor (noisy, high-volume) --> processor (structured logs) --> monitor (log aggregator)
```

## Nodes

**sensor.py** -- Simulates a temperature sensor. Emits `print()`, `logging.info()`,
`logging.debug()`, and `logging.warning()`. YAML sets `min_log_level: info` (suppresses
debug at source) and `max_log_size: "1KB"` (triggers log rotation quickly).

**processor.py** -- Computes a running average. Uses `send_logs_as: log_entries` to route
structured logs as dataflow data. Raw `print()` lines are NOT routed by `send_logs_as`.

**monitor.py** -- Consumes log entries from processor and sensor readings. Counts
warnings and errors, demonstrating in-dataflow log aggregation.

## Running (Direct Mode)

`dora run` executes the dataflow in a single process -- good for quick testing.

```bash
# Basic run (5 seconds)
dora run dataflow.yml --stop-after 5s

# Show only warnings and above
dora run dataflow.yml --log-level warn --stop-after 5s

# Per-node filtering: debug for monitor, warn for sensor
dora run dataflow.yml --log-filter "monitor=debug,sensor=warn" --stop-after 5s

# JSON output for machine parsing
dora run dataflow.yml --log-format json --stop-after 3s

# Using environment variables
DORA_LOG_LEVEL=warn dora run dataflow.yml --stop-after 5s
```

## Running (Distributed Mode)

`dora up` + `dora start` runs through the coordinator/daemon architecture --
required for multi-machine deployments.

```bash
# 1. Start coordinator + daemon in background
dora up

# 2a. Start attached (live log stream, Ctrl-C to detach)
dora start dataflow.yml --attach

# 2b. Or start detached and query logs separately
dora start dataflow.yml
```

Once running, use `dora logs` to query any node:

```bash
# Stream live logs from sensor
dora logs <dataflow-id> sensor --follow

# Stream only warnings and above from sensor
dora logs <dataflow-id> sensor --follow --level warn

# Last 20 lines from all nodes
dora logs <dataflow-id> --all-nodes --tail 20

# Search for specific patterns
dora logs <dataflow-id> processor --grep "error" --since 5m

# Multiple terminals for targeted monitoring:
#   Terminal 1: dora logs <id> sensor --follow --level warn
#   Terminal 2: dora logs <id> processor --follow --level error
#   Terminal 3: dora logs <id> monitor --follow
```

When done:

```bash
dora stop
dora down
```

## Post-Run Analysis

Works the same for both direct and distributed mode:

```bash
# Read local log files
dora logs --local --all-nodes --tail 20

# Search sensor warnings
dora logs --local sensor --grep "high temp"

# Time-filtered errors
dora logs --local --all-nodes --since 10s --level error

# Check log rotation (1KB limit triggers rotation quickly)
ls -la out/*/log_sensor*.jsonl
```

## Features Demonstrated

| Feature | Where |
|---------|-------|
| `min_log_level` | sensor: suppresses debug at daemon |
| `max_log_size` | sensor: 1KB triggers rotation quickly |
| `send_logs_as` | processor: routes structured logs as data |
| `print()` vs `logging` | sensor/processor: shows how each is handled |
| Log aggregation | monitor: counts errors/warnings from log stream |
| `--log-level` | CLI-level display filtering |
| `--log-filter` | Per-node display overrides |
| `--log-format` | pretty, json, compact output |
| `dora logs --local` | Post-run log file reading |
| `--grep`, `--since` | Targeted log search |
