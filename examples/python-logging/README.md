# Python Logging Example

Demonstrates all adora logging features using a three-node pipeline:

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

`adora run` executes the dataflow in a single process -- good for quick testing.

```bash
# Basic run (5 seconds)
adora run dataflow.yml --stop-after 5s

# Show only warnings and above
adora run dataflow.yml --log-level warn --stop-after 5s

# Per-node filtering: debug for monitor, warn for sensor
adora run dataflow.yml --log-filter "monitor=debug,sensor=warn" --stop-after 5s

# JSON output for machine parsing
adora run dataflow.yml --log-format json --stop-after 3s

# Using environment variables
ADORA_LOG_LEVEL=warn adora run dataflow.yml --stop-after 5s
```

## Running (Distributed Mode)

`adora up` + `adora start` runs through the coordinator/daemon architecture --
required for multi-machine deployments.

```bash
# 1. Start coordinator + daemon in background
adora up

# 2a. Start attached (live log stream, Ctrl-C to detach)
adora start dataflow.yml --attach

# 2b. Or start detached and query logs separately
adora start dataflow.yml
```

Once running, use `adora logs` to query any node:

```bash
# Stream live logs from sensor
adora logs <dataflow-id> sensor --follow

# Stream only warnings and above from sensor
adora logs <dataflow-id> sensor --follow --level warn

# Last 20 lines from all nodes
adora logs <dataflow-id> --all-nodes --tail 20

# Search for specific patterns
adora logs <dataflow-id> processor --grep "error" --since 5m

# Multiple terminals for targeted monitoring:
#   Terminal 1: adora logs <id> sensor --follow --level warn
#   Terminal 2: adora logs <id> processor --follow --level error
#   Terminal 3: adora logs <id> monitor --follow
```

When done:

```bash
adora stop
adora destroy
```

## Post-Run Analysis

Works the same for both direct and distributed mode:

```bash
# Read local log files
adora logs --local --all-nodes --tail 20

# Search sensor warnings
adora logs --local sensor --grep "high temp"

# Time-filtered errors
adora logs --local --all-nodes --since 10s --level error

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
| `adora logs --local` | Post-run log file reading |
| `--grep`, `--since` | Targeted log search |
