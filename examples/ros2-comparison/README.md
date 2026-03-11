# Adora vs ROS2 Benchmark Comparison

Reproducible latency and throughput comparison between Adora and ROS2 using identical workloads.

## Prerequisites

- Adora CLI installed (`cargo install --path binaries/cli`)
- Python Adora package: `pip install adora-rs`
- `numpy` and `pyarrow`: `pip install numpy pyarrow`
- ROS2 Humble (or later) with `rclpy` and `std_msgs`

## Methodology

Both frameworks run the same workload:

- **Latency**: 100 messages per payload size with 5ms sleep between sends
- **Throughput**: 100 messages per payload size sent back-to-back
- **Payload sizes**: 0B, 8B, 64B, 512B, 2KB, 4KB, 16KB, 40KB, 400KB, 4MB
- **Timestamps**: `time.perf_counter_ns()` embedded in first 8 bytes of each payload
- **Metrics**: avg, p50, p95, p99, p99.9, min, max latency; messages/sec throughput

Both sides use Python APIs to ensure a fair comparison of framework overhead (not language performance).

## Quick Start

```bash
cd examples/ros2-comparison
./run_comparison.sh
```

This runs both benchmarks sequentially and prints a comparison table.

## Manual Execution

### Adora

```bash
BENCH_CSV=results/adora.csv adora run dataflow.yml --uv
```

### ROS2

Terminal 1 (receiver):
```bash
BENCH_CSV=results/ros2.csv python3 ros2_receiver.py
```

Terminal 2 (sender):
```bash
python3 ros2_sender.py
```

### Compare Results

```bash
python3 analyze.py results/adora.csv results/ros2.csv
```

## Output Format

CSV files contain one row per (mode, payload_size):

```
latency,<bytes>,<label>,<n>,<avg_ns>,<p50_ns>,<p95_ns>,<p99_ns>,<p999_ns>,<min_ns>,<max_ns>
throughput,<bytes>,<label>,<n>,<msg_per_sec>,<elapsed_ns>,0,0,0,0,0
```

## Files

| File | Description |
|------|-------------|
| `adora_sender.py` | Adora benchmark sender node |
| `adora_receiver.py` | Adora benchmark receiver node |
| `dataflow.yml` | Adora dataflow definition |
| `ros2_sender.py` | ROS2 benchmark publisher |
| `ros2_receiver.py` | ROS2 benchmark subscriber |
| `run_comparison.sh` | Orchestration script |
| `analyze.py` | CSV comparison and reporting |
