# Log Sink: File Aggregation

Aggregates structured logs from multiple Python nodes into a single JSONL file.

## Architecture

```
timer (200ms) --> sensor (Python)    --> log_entries --> file_sink (Rust) --> combined.jsonl
                  processor (Python) --> log_entries -->
                  sensor             --> reading     --> processor
```

## Nodes

**sensor** (`sensor.py`) -- Simulates a temperature sensor producing random readings. Uses `send_logs_as: log_entries` to capture Python `logging.*` calls as Arrow data. Logs warnings when temperature is above 28C or below 15C.

**processor** (`processor.py`) -- Computes a rolling average (20-sample window) of sensor readings. Also uses `send_logs_as: log_entries` to route its own structured logs as dataflow data.

**file_sink** (Rust, `log-sink-file`) -- Receives log streams from both sensor and processor. Parses Arrow log entries and writes each as a JSON line to the output file. Flushes every 100 entries.

## Configuration

| Env var | Default | Description |
|---------|---------|-------------|
| `LOG_FILE` | `./combined.jsonl` | Output file path (must be relative, no `..`) |

## Run

```bash
cargo build -p log-sink-file
dora run dataflow.yml

# Check output
cat combined.jsonl | head
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `send_logs_as` to route logging.* as data | YAML (sensor, processor) |
| Multi-source log aggregation | File sink merges two streams |
| Arrow log parsing to JSONL | Rust sink |
| Path validation (no traversal) | Rust sink |
