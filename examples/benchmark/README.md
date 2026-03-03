# Benchmark

Measures **latency** and **throughput** across 10 payload sizes from 0 bytes to ~4 MB.

## Architecture

```
rust-node --> latency    --> rust-sink (records timestamps, computes stats)
           --> throughput --> rust-sink (queue_size: 1000)
```

`rust-node` has no external inputs -- it drives itself internally with `std::thread::sleep`.

## Nodes

**rust-node** (Rust, release build) -- For each of 10 payload sizes, sends one latency message (with 10ms sleep) then 100 throughput messages in rapid succession. Uses `send_output_raw()` for lowest-overhead binary transmission. Sends a single-byte sentinel `[1]` to mark end of each throughput batch. Sleeps 2s between size brackets.

**rust-sink** (Rust, release build) -- Records arrival timestamps via `metadata.timestamp()`. Segments results by `data.len()` to detect payload size changes. Computes average latency per size and messages/sec for throughput.

## Run

```bash
cargo build --release -p benchmark-example-node -p benchmark-example-sink
adora run dataflow.yml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `send_output_raw()` for zero-overhead binary | Node |
| `metadata.timestamp()` for arrival timing | Sink |
| `queue_size: 1000` for burst buffering | YAML (throughput input) |
| Release-mode builds for accurate measurement | Both nodes |
| Variable payload sizes (0B to ~4MB) | Node |
