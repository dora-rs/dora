# Rust Dataflow

Basic three-node Rust pipeline with three variant configurations demonstrating different transport and node loading modes.

## Architecture

```
timer (10ms)  --> rust-node --> random --> rust-status-node --> status --> rust-sink
timer (100ms) ---------------------------------->
```

## Nodes

**rust-node** -- Generates random values on each 10ms timer tick and sends them on the `random` output.

**rust-status-node** -- Receives random values and its own 100ms timer tick. Combines them into a status message sent on the `status` output.

**rust-sink** -- Consumes and prints status messages.

## Dataflow Variants

| File | What's Different |
|------|-----------------|
| `dataflow.yml` | Standard: shared memory IPC, pre-built local binaries |
| `dataflow_dynamic.yml` | Dynamic sink: uses `path: dynamic` for runtime-discovered nodes, 100ms timer on source |

## Run

```bash
# Standard
cargo run --example rust-dataflow

# Or manually:
cargo build -p rust-dataflow-example-node -p rust-dataflow-example-status-node -p rust-dataflow-example-sink
adora run dataflow.yml

# Dynamic variant
cargo build -p rust-dataflow-example-sink-dynamic
adora run dataflow_dynamic.yml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| Multi-stage Rust pipeline | All three nodes |
| Timer inputs (`millis/10`, `millis/100`) | YAML |
| Shared memory IPC (default) | `dataflow.yml` |
| Dynamic node loading | `dataflow_dynamic.yml` |
| `build:` for pre-run compilation | YAML |
