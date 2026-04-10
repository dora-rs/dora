# Log Sink: TCP Streaming

Streams structured log entries over TCP to a remote receiver with automatic reconnection.

## Architecture

```
timer (500ms) --> source (Python) --> log_entries --> tcp_sink (Rust) --> TCP 127.0.0.1:9876
```

## Nodes

**source** (`source.py`) -- Generates random readings and logs at info, warn, or error level based on value thresholds. Uses `send_logs_as: log_entries` to capture Python logging output as Arrow data.

**tcp_sink** (Rust, `log-sink-tcp`) -- Connects to a TCP endpoint, parses Arrow log entries, and writes each as a JSON line over the socket. On write failure, reconnects if at least 2 seconds have elapsed since the last attempt (throttle prevents SYN-flood), then retries the write. Drops the entry only if the retry also fails.

## Configuration

| Env var | Default | Description |
|---------|---------|-------------|
| `SINK_ADDR` | `127.0.0.1:9876` | TCP endpoint to stream logs to |

## Run

```bash
cargo build -p log-sink-tcp

# Terminal 1: start a TCP listener (e.g., netcat)
nc -l 9876

# Terminal 2: run the dataflow
dora run dataflow.yml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `send_logs_as` to route logging.* as data | YAML (source) |
| TCP log streaming with JSONL format | Rust sink |
| Throttled reconnection (2s backoff) | Rust sink |
| `env:` for runtime configuration | YAML (SINK_ADDR) |
