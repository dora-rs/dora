# Log Sink: Alert Routing

Routes log entries by severity -- errors and warnings go to a dedicated `alerts` output while all entries go to `all_logs`.

## Architecture

```
timer (200ms) --> source (Python) --> log_entries --> alert_router (Rust) --> all_logs
                                                                          --> alerts (error/warn only)
```

## Nodes

**source** (`source.py`) -- Generates 20 JSON log entries cycling through error, warn, info, and debug levels. Uses `send_stdout_as: log_entries` so that `print(json.dumps(...))` output is captured as Arrow data on the `log_entries` output.

**alert_router** (Rust, `log-sink-alert`) -- Parses incoming Arrow log data via `adora_log_utils::parse_log_from_arrow()`. Forwards every entry to `all_logs`. Entries at `Error` or `Warn` level are additionally sent to the `alerts` output for targeted monitoring.

## Run

```bash
cargo build -p log-sink-alert
adora run dataflow.yml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `send_stdout_as` to capture print() as node output | YAML (source) |
| Arrow-based log parsing | Rust router |
| Severity-based log routing | Router filters error/warn |
| Fan-out: one input to multiple outputs | Router (all_logs + alerts) |
