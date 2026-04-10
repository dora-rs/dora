# Real-Time Tuning Guide

Dora achieves 10-17x lower latency than ROS2 out of the box via zero-copy
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

Dora provides an optional `--rt` flag that enables memory locking and
real-time thread priority:

```bash
# Start daemon with real-time profile
dora daemon --rt

# Requires CAP_SYS_NICE + CAP_IPC_LOCK (or run as root)
sudo dora daemon --rt
```

This sets:
- `mlockall(MCL_CURRENT | MCL_FUTURE)` — pin all memory, prevent page faults
- `SCHED_FIFO` priority 50 — real-time scheduling for the main daemon thread

**Important:** `SCHED_FIFO` applies only to the main thread (the one calling
`block_on`). Tokio worker threads remain at `SCHED_OTHER`. For full RT
coverage, use `taskset` + `chrt` to promote the entire process, or use
`isolcpus` to dedicate cores.

**Warning:** `SCHED_FIFO` without guardrails can starve other processes.
Ensure `/proc/sys/kernel/sched_rt_runtime_us` is set (default 950000 = 95%)
to prevent total system hang.

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
# Ubuntu 22.04+
sudo apt install linux-lowlatency

# Debian
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
taskset -c 2,3 dora daemon

# Pin a specific node to core 4
taskset -c 4 ./my-node

# Verify affinity
taskset -p <pid>
```

### Real-Time Priority

```bash
# Set SCHED_FIFO priority 50 (requires CAP_SYS_NICE)
chrt -f 50 dora daemon

# Set nice level (less aggressive than SCHED_FIFO)
nice -n -20 dora daemon
```

### Memory Locking

```bash
# Increase memlock limit (required for SHM + mlockall)
ulimit -l unlimited

# Or in /etc/security/limits.conf:
dora-user  soft  memlock  unlimited
dora-user  hard  memlock  unlimited
```

## Systemd Service Configuration

```ini
[Unit]
Description=Dora Daemon
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/dora daemon --rt
User=dora
Group=dora

# CPU affinity: pin to cores 2-3
CPUAffinity=2 3

# Memory locking
LimitMEMLOCK=infinity

# Real-time scheduling (use instead of Nice when --rt is enabled)
LimitRTPRIO=99
# Note: Nice=-20 is ignored under SCHED_FIFO; omit when using --rt.

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
  dora-image dora daemon --rt
```

### Kubernetes

```yaml
apiVersion: v1
kind: Pod
spec:
  containers:
    - name: dora-daemon
      command: ["dora", "daemon", "--rt"]
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
dora daemon --worker-threads 4
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
ZENOH_CONFIG=zenoh-config.json5 dora daemon
```

## Benchmarking Tips

```bash
# Pin benchmark to specific cores for reproducible results
taskset -c 2,3 dora run examples/benchmark/dataflow.yml --release

# Disable CPU frequency scaling before benchmarking
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Use --stop-after for fixed-duration runs
dora run examples/benchmark/dataflow.yml --stop-after 30s
```

## What Dora Cannot Guarantee

- **Hard real-time**: Dora runs on Linux (not an RTOS). Even with PREEMPT_RT,
  worst-case latency is microseconds, not nanoseconds.
- **Deterministic allocation**: The Rust standard allocator is not real-time safe.
  Consider jemalloc or mimalloc for more predictable allocation patterns.
- **Interrupt-free execution**: Network interrupts, disk I/O, and kernel scheduler
  decisions can always cause jitter. Use `isolcpus` and IRQ affinity to minimize.
