# Benchmark

Measures **latency** (with percentile analysis) and **throughput** across 10 payload sizes from 0 bytes to ~4 MB.

## Architecture

```
rust-node --> latency    --> rust-sink (records timestamps, computes percentiles)
           --> throughput --> rust-sink (queue_size: 1000)
```

`rust-node` has no external inputs -- it drives itself internally with `std::thread::sleep`.

## Nodes

**rust-node** (Rust, release build) -- For each of 10 payload sizes, sends 100 latency messages (with 5ms sleep between sends) then 100 throughput messages in rapid succession. Uses `send_output_raw()` for lowest-overhead binary transmission. Sends a single-byte sentinel `[1]` to mark end of each throughput batch. Sleeps 2s between size brackets.

**rust-sink** (Rust, release build) -- Records arrival timestamps via `metadata.timestamp()`. Segments results by `data.len()` to detect payload size changes. Computes latency percentiles (p50/p95/p99/p99.9/min/max) per size and messages/sec for throughput.

## Run

```bash
cargo build --release -p benchmark-example-node -p benchmark-example-sink
dora run dataflow.yml
```

### CSV output

Set `BENCH_CSV` to write machine-readable results:

```bash
BENCH_CSV=results.csv dora run dataflow.yml
```

CSV columns: `mode,bytes,size_label,n,avg_or_msgps,p50,p95,p99,p999,min,max` (nanos for latency, 0 for throughput percentiles).

## Example output

```
Latency:
      0B  avg=   855.2us  p50=   812.0us  p95=  1.234ms  p99=  1.567ms  p99.9=  1.890ms  min=   723.0us  max=  1.890ms  (n=100)
      8B  avg=   870.1us  p50=   834.0us  p95=  1.201ms  p99=  1.445ms  p99.9=  1.678ms  min=   745.0us  max=  1.678ms  (n=100)
     4KB  avg=   412.3us  p50=   398.0us  p95=   567.0us  p99=   612.0us  p99.9=   678.0us  min=   356.0us  max=   678.0us  (n=100)
     4MB  avg=   530.1us  p50=   510.0us  p95=   678.0us  p99=   734.0us  p99.9=   812.0us  min=   445.0us  max=   812.0us  (n=100)

Throughput:
      0B  125000 msg/s  (n=100, 0.800ms)
     4KB  95000 msg/s  (n=100, 1.053ms)
     4MB  2100 msg/s  (n=100, 47.619ms)
```

Note: latency stays flat from 4KB to 4MB because shared memory transport kicks in at the 4KB threshold (zero-copy).

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `send_output_raw()` for zero-overhead binary | Node |
| `metadata.timestamp()` for arrival timing | Sink |
| Percentile latency analysis (p50/p95/p99/p99.9) | Sink |
| `queue_size: 1000` for burst buffering | YAML (throughput input) |
| Release-mode builds for accurate measurement | Both nodes |
| Variable payload sizes (0B to ~4MB) | Node |
| Optional CSV export via `BENCH_CSV` env var | Sink |
