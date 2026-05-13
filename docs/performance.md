# Performance

Dora achieves 10-17x lower latency than ROS2 Python through zero-copy shared memory IPC, Apache Arrow columnar format, and 100% Rust internals. This document covers methodology, reproduction, and tuning.

## Architecture Advantages

| Layer | Dora | ROS2 (rclpy) |
|-------|-------|---------------|
| Runtime | Rust async (tokio) | Python + C++ middleware |
| IPC (>4KB) | Zenoh SHM zero-copy | DDS serialization + copy |
| IPC (<4KB) | TCP with bincode | DDS serialization + copy |
| Data format | Apache Arrow (zero-serde) + optional IPC framing | CDR serialization |
| Threading | Lock-free channels (flume) | GIL-bound callbacks |
| Fan-out | Arc-wrapped (O(1) per receiver) | Per-receiver copy |
| Metrics | Fire-and-forget (spawned task) | Inline |

## Benchmark Suite

### Internal benchmarks (`examples/benchmark/`)

Measures Dora's own latency and throughput across 10 payload sizes (0B to 4MB).

```bash
cd examples/benchmark
./compare.sh          # Rust vs Python sender comparison
```

Metrics reported: avg, p50, p95, p99, p99.9, min, max latency; msg/s throughput.

### ROS2 comparison (`examples/ros2-comparison/`)

Apples-to-apples comparison using identical Python workloads on both frameworks.

```bash
cd examples/ros2-comparison
./run_comparison.sh   # Requires ROS2 Humble+
```

Both sides use `time.perf_counter_ns()` timestamps embedded in payload first 8 bytes. Same message count, sizes, and sleep intervals ensure comparable results.

### Criterion micro-benchmarks

Isolated benchmarks for internal hot paths:

```bash
# Daemon message routing (fan-out x payload size matrix)
cargo bench -p dora-daemon

# Message serialization/deserialization
cargo bench -p dora-message
```

CI tracks these via `benchmark-action/github-action-benchmark` with 120% alert threshold.

## Reproducing Results

### Requirements

- Linux or macOS (shared memory IPC)
- Rust toolchain at or above the workspace MSRV (see `rust-version`
  in root `Cargo.toml`), release profile
- Python 3.10+ with `numpy`, `pyarrow`
- ROS2 Humble+ (for comparison only)

### Steps

1. Build Dora:
   ```bash
   cargo install --path binaries/cli --locked
   ```

2. Run internal benchmark:
   ```bash
   cd examples/benchmark
   BENCH_CSV=results/rust.csv dora run dataflow.yml
   ```

3. Run ROS2 comparison:
   ```bash
   cd examples/ros2-comparison
   ./run_comparison.sh
   ```

### Environment Notes

- Close background applications to reduce variance
- Use `taskset` or `cpuset` to pin processes for consistent results
- Run at least 3 iterations and report median
- Shared memory benefits appear at payloads >4KB

## Performance Tuning

### Queue sizes

Default queue size is 10. For high-throughput outputs, increase it:

```yaml
inputs:
  data:
    source: producer/output
    queue_size: 1000
```

### Payload size

Dora automatically uses shared memory for messages >4KB, avoiding copies. Structure data to exceed this threshold when low latency matters.

### Arrow format

Use Arrow arrays directly instead of converting to/from Python lists:

```python
# Fast: pass Arrow array directly
node.send_output("out", pa.array(data, type=pa.uint8()))

# Slow: convert through Python list
node.send_output("out", pa.array(list(data), type=pa.uint8()))
```

### Operator vs Node

Operators run in-process with the runtime (zero IPC overhead) but share the GIL in Python. Use Rust operators for compute-heavy work, Python operators for glue logic.

### Distributed deployment

For cross-machine communication, Dora uses Zenoh pub-sub. Latency depends on network quality. Use local deployment (single-machine) when sub-millisecond latency is required.

### Profile-Guided Optimization (PGO)

PGO trains the compiler on real execution traces and can give 5-15% wins on i-cache-heavy hot loops. For dora's pub/sub data path it typically helps throughput more than latency — the throughput phase of the benchmark is the canonical "tight loop with biased branches" workload PGO is designed for.

**Measurement on macOS-arm64** ([#331](https://github.com/dora-rs/dora/issues/331)):

| Metric | PGO / baseline geomean | Delta |
|---|---|---|
| p50 latency | 1.022 | +2.2% (noise) |
| p99 latency | 1.010 | +1.0% (noise) |
| **Throughput** | **1.256** | **+25.6%** |

**Run the measurement on your platform.** Results don't transfer across OS/arch — Linux x86 may show different numbers, and only your-platform numbers should drive your-platform decisions.

```bash
make qa-pgo-install   # one-time: cargo-pgo + llvm-tools-preview
make qa-pgo           # ~30-40 min: instrument, train, optimize, compare
```

`make qa-pgo` prints a side-by-side comparison and an explicit verdict based on the throughput geomean. Decision rule:

- **≥ +5% throughput** — PGO is worth integrating into the release pipeline for that platform
- **within ±5%** — inconclusive, re-run with multiple iterations for stable medians
- **≤ -5% throughput** — PGO regresses on this platform; do not ship

The PGO build pipeline adds roughly 2.5-3× the wall time of a plain `--profile dist` build (~17 min total on macOS-arm64). That puts it firmly in release-pipeline territory — **don't add it to per-commit CI**. The recommended shape is a one-shot job that runs against a release tag and uploads PGO'd binaries as release artifacts.

**What gets PGO'd:** only `dora-cli` (which embeds the coordinator, daemon, and runtime). User node binaries are not instrumented by this recipe; they would need their own per-binary PGO pass if their hot paths matter.

## CSV Output Format

All benchmarks support `BENCH_CSV` environment variable for machine-readable output:

```
latency,<bytes>,<label>,<n>,<avg_ns>,<p50_ns>,<p95_ns>,<p99_ns>,<p999_ns>,<min_ns>,<max_ns>
throughput,<bytes>,<label>,<n>,<msg_per_sec>,<elapsed_ns>,0,0,0,0,0
```
