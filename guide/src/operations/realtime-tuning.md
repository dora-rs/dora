# Real-Time Tuning

Adora provides optional real-time features for latency-sensitive robotics deployments.

## Quick Start

```bash
# Start daemon with real-time profile (mlockall + SCHED_FIFO)
sudo adora daemon --rt

# Control worker threads
adora daemon --worker-threads 4

# Pin to specific CPU cores
taskset -c 2,3 adora daemon --rt
```

## What `--rt` Does

- `mlockall(MCL_CURRENT | MCL_FUTURE)` — pins all memory, prevents page faults
- `SCHED_FIFO` priority 50 (Linux only) — real-time scheduling for the main thread
- Requires `CAP_SYS_NICE` + `CAP_IPC_LOCK` capabilities

## Per-Node CPU Affinity

Instead of pinning the entire daemon with `taskset`, you can pin individual nodes to specific CPU cores via the dataflow YAML:

```yaml
- id: controller
  path: ./controller
  cpu_affinity: [2, 3]

- id: sensor
  path: ./sensor
  cpu_affinity: [4, 5]
```

The daemon calls `sched_setaffinity` on the spawned process. This is Linux only; on other platforms the field is silently ignored.

Combine with `--rt` for best results: the daemon gets real-time scheduling while each node is pinned to dedicated cores, avoiding contention.

## Full Guide

See the comprehensive [Real-Time Tuning Guide](../../../docs/realtime-tuning.md) for:

- Linux kernel tuning (CPU governor, PREEMPT_RT, boot parameters)
- Process-level tuning (CPU affinity, memory locking, thread priority)
- Systemd service configuration
- Docker and Kubernetes deployment
- Zenoh transport tuning
- Benchmarking tips
