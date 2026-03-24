# Real-Time Tuning Guide

Adora achieves 10-17x lower latency than ROS2 out of the box via zero-copy
shared memory and Apache Arrow. For latency-sensitive robotics deployments
(1-10ms control loops), additional OS-level tuning can reduce jitter further.

## Quick Summary

| Tuning | Jitter Reduction | Difficulty |
|--------|------------------|------------|
| CPU frequency governor -> performance | Eliminates 50-200us transitions | Easy |
| CPU affinity (taskset) | 2-5x (avoids CPU migration) | Easy |
| Memory locking (mlockall / --rt flag) | Eliminates 100us+ page faults | Easy |
| Thread priority (SCHED_FIFO / --rt flag) | Deterministic scheduling | Medium |
| PREEMPT_RT kernel | Sub-100us worst-case | Medium |
| Disable transparent huge pages | Avoids THP compaction stalls | Easy |

## Daemon `--rt` Flag

Adora provides an optional `--rt` flag that enables memory locking and
real-time thread priority:

```bash
# Start daemon with real-time profile
adora daemon --rt

# Requires CAP_SYS_NICE + CAP_IPC_LOCK (or run as root)
sudo adora daemon --rt
```

This sets:
- `mlockall(MCL_CURRENT | MCL_FUTURE)` — pin all memory, prevent page faults
- `SCHED_FIFO` priority 50 — deterministic scheduling for daemon threads

## Linux Kernel Tuning

### CPU Frequency Governor

```bash
# Set all CPUs to performance mode (no frequency scaling)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Verify
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

### Disable Transparent Huge Pages

```bash
echo never | sudo tee /sys/kernel/mm/transparent_hugepage/enabled
echo never | sudo tee /sys/kernel/mm/transparent_hugepage/defrag
```

### PREEMPT_RT Kernel (Optional)

For sub-100us worst-case latency, use a PREEMPT_RT patched kernel:

```bash
# Ubuntu
sudo apt install linux-image-rt-amd64

# Or build from source with CONFIG_PREEMPT_RT=y
```

### Boot Parameters

```bash
# /etc/default/grub — add to GRUB_CMDLINE_LINUX:
clocksource=tsc tsc=reliable nohz_full=2-7 isolcpus=2-7 rcu_nocbs=2-7
```

- `nohz_full=2-7` — disable timer ticks on cores 2-7 (for real-time tasks)
- `isolcpus=2-7` — isolate cores from general scheduler
- `rcu_nocbs=2-7` — offload RCU callbacks from isolated cores

## Process-Level Tuning

### CPU Affinity

```bash
# Pin daemon to cores 2-3 (avoid core 0 for IRQs)
taskset -c 2,3 adora daemon

# Pin a specific node to core 4
taskset -c 4 ./my-node

# Verify affinity
taskset -p <pid>
```

### Real-Time Priority

```bash
# Set SCHED_FIFO priority 50 (requires CAP_SYS_NICE)
chrt -f 50 adora daemon

# Set nice level (less aggressive than SCHED_FIFO)
nice -n -20 adora daemon
```

### Memory Locking

```bash
# Increase memlock limit (required for SHM + mlockall)
ulimit -l unlimited

# Or in /etc/security/limits.conf:
adora-user  soft  memlock  unlimited
adora-user  hard  memlock  unlimited
```

## Systemd Service Configuration

```ini
[Unit]
Description=Adora Daemon
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/adora daemon --rt
User=adora
Group=adora

# CPU affinity: pin to cores 2-3
CPUAffinity=2 3

# Real-time priority
Nice=-20

# Memory locking
LimitMEMLOCK=infinity

# Real-time scheduling
LimitRTPRIO=99

# Capabilities (instead of running as root)
AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK

# Restart on failure
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
```

## Container Deployment (Docker / Kubernetes)

### Docker

```bash
docker run \
  --cap-add SYS_NICE \
  --cap-add IPC_LOCK \
  --ulimit memlock=-1:-1 \
  --cpuset-cpus="2,3" \
  adora-image adora daemon --rt
```

### Kubernetes

```yaml
apiVersion: v1
kind: Pod
spec:
  containers:
    - name: adora-daemon
      command: ["adora", "daemon", "--rt"]
      resources:
        limits:
          cpu: "2"
          memory: "4Gi"
        requests:
          cpu: "2"     # Guaranteed QoS (no CPU throttling)
          memory: "4Gi"
      securityContext:
        capabilities:
          add: ["SYS_NICE", "IPC_LOCK"]
```

## Tokio Runtime Tuning

```bash
# Control worker thread count (default: num_cpus)
adora daemon --worker-threads 4

# Or via environment variable
TOKIO_WORKER_THREADS=4 adora daemon
```

## Zenoh Transport Tuning

For distributed deployments, Zenoh transport settings affect latency:

```json5
// zenoh-config.json5
{
  "transport": {
    "unicast": {
      "lowlatency": true     // Disable batching for lower latency
    }
  }
}
```

```bash
ZENOH_CONFIG=zenoh-config.json5 adora daemon
```

## Benchmarking Tips

```bash
# Pin benchmark to specific cores for reproducible results
taskset -c 2,3 adora run examples/benchmark/dataflow.yml --release

# Disable CPU frequency scaling before benchmarking
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Use --stop-after for fixed-duration runs
adora run examples/benchmark/dataflow.yml --stop-after 30s
```

## What Adora Cannot Guarantee

- **Hard real-time**: Adora runs on Linux (not an RTOS). Even with PREEMPT_RT,
  worst-case latency is microseconds, not nanoseconds.
- **Deterministic allocation**: The Rust standard allocator is not real-time safe.
  Consider jemalloc (`--features jemalloc`) for more predictable behavior.
- **Interrupt-free execution**: Network interrupts, disk I/O, and kernel scheduler
  decisions can always cause jitter. Use `isolcpus` and IRQ affinity to minimize.
