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

## CI Coverage

The `--rt` / mlock / SCHED_FIFO paths are **manual Tier 2** (see
[`testing-matrix.md`](testing-matrix.md#soft-real-time)). Not automated
on GitHub Actions for two reasons:

1. **Privilege.** `--rt` needs `CAP_SYS_NICE` and `CAP_IPC_LOCK`. GHA
   runners are unprivileged and refuse `sudo` for anything that touches
   scheduling or memory locking.
2. **Kernel config.** `SCHED_FIFO` needs `CONFIG_RT_GROUP_SCHED` (and
   ideally a PREEMPT_RT kernel for meaningful guarantees). GHA runners
   use stock Ubuntu kernels.

A self-hosted runner with an RT-patched kernel could run this, but the
cost-benefit is poor until there's evidence of regressions in the RT
path. Tracked as #256.

## Maintainer Validation Runbook

Run this end-to-end before a release when code in `binaries/daemon/src/`
or the scheduler/priority setup has changed. Requires root or
`CAP_SYS_NICE + CAP_IPC_LOCK`.

### Prerequisites

- Linux with a reasonably modern kernel (5.x+ fine for soft-RT)
- `sudo` or capabilities granted
- `dora` CLI installed from the release candidate

### Smoke the `--rt` flag

```bash
# 1. Start a coordinator and --rt daemon
dora coordinator &
sudo dora daemon --rt &

# 2. Wait for registration
sleep 3
dora list  # should show no dataflows, daemon registered

# 3. Verify the daemon has mlockall applied.
# VmLck should be non-zero and close to VmRSS if mlockall succeeded.
grep -E '^(VmRSS|VmLck):' /proc/$(pgrep -f 'dora daemon')/status

# 4. Verify SCHED_FIFO on the daemon's main thread.
chrt -p $(pgrep -f 'dora daemon')
# Expected: "scheduling policy: SCHED_FIFO" (not SCHED_OTHER)
```

Pass criteria:
- `VmLck` > 0 (memory actually locked)
- `scheduling policy: SCHED_FIFO` (priority class applied)

### Smoke `cpu_affinity` (covered by Tier 1)

Already automated in the `cpu-affinity-smoke` nightly job — it runs the
`cpu-affinity-probe` fixture and asserts the mask lands on the spawned
process. If `cpu-affinity-smoke` is green, this part is covered. No
manual step required.

### Scope of `--rt`: daemon process only

`--rt` applies mlockall + SCHED_FIFO to the **daemon process itself**
(see `binaries/cli/src/command/daemon.rs`). It does **not** propagate
SCHED_FIFO to spawned node processes — the spawn path at
`binaries/daemon/src/spawn/prepared.rs:422` only applies `cpu_affinity`
in the `pre_exec` hook; there is no `sched_setscheduler` call on the
child.

This is by design: real-time scheduling on arbitrary user code is
risky (a tight non-yielding loop under SCHED_FIFO can lock up a core
until reboot). Nodes that need SCHED_FIFO must opt in explicitly, for
example via [`thread-priority`](https://crates.io/crates/thread-priority)
or a direct `libc::sched_setscheduler` call inside the node's own init
code.

### Verify node processes inherit `cpu_affinity` under `--rt`

The daemon-level `--rt` profile is orthogonal to `cpu_affinity` — both
should still apply correctly. Tier 1 `cpu-affinity-smoke` already
covers this for a default daemon; the manual step is to confirm the
same under `--rt`.

```bash
pkill -f "dora (coordinator|daemon)" 2>/dev/null; sleep 1
dora coordinator &
sudo dora daemon --rt &
sleep 2
dora start examples/cpu-affinity-probe/dataflow.yml --name rt-affinity --detach
sleep 3
dora logs rt-affinity 2>&1 | grep AFFINITY_MASK
# Expected: AFFINITY_MASK:0,1 (matches cpu_affinity in the fixture)
dora stop --name rt-affinity 2>/dev/null || true
pkill -f "dora (coordinator|daemon)" 2>/dev/null
```

Pass criterion: `AFFINITY_MASK:0,1`. If the mask differs or is empty,
`--rt` is breaking the `pre_exec` affinity hook — regression.

### Jitter regression test

Only if the PR touches code that could affect jitter (daemon hot path,
send/recv loops, allocator changes).

Both runs must go through an externally-started daemon so the `--rt`
profile actually applies. `dora run` spawns its own embedded coordinator
and daemon (see `binaries/cli/src/command/run.rs`) and will silently
ignore the background `--rt` daemon, so use `dora start` instead.

```bash
# Stock run — plain daemon, no --rt
pkill -f "dora (coordinator|daemon)" 2>/dev/null; sleep 1
dora coordinator &
dora daemon &
sleep 2
dora start examples/benchmark/dataflow.yml --name stock-bench --detach
sleep 32   # 30s --stop-after not supported on `dora start`; poll to completion
dora logs stock-bench > /tmp/stock.log 2>&1
dora stop --name stock-bench 2>/dev/null || true
pkill -f "dora (coordinator|daemon)" 2>/dev/null; sleep 1

# --rt run — daemon with the RT profile
dora coordinator &
sudo dora daemon --rt &
sleep 2
dora start examples/benchmark/dataflow.yml --name rt-bench --detach
sleep 32
dora logs rt-bench > /tmp/rt.log 2>&1
dora stop --name rt-bench 2>/dev/null || true
pkill -f "dora (coordinator|daemon)" 2>/dev/null; sleep 1

# Compare p99 latencies. `--rt` should be at least as low, typically 2-5x better.
grep -E 'p99|max' /tmp/stock.log /tmp/rt.log
```

Flag in the release notes if p99 regressed versus the previous release.

### When to skip this runbook

- Docs-only PR
- No changes under `binaries/daemon/src/spawn/`, `binaries/daemon/src/lib.rs`
  around the `--rt` flag, or `dora-message` descriptor fields that feed
  into the scheduler

### When this becomes automated

This runbook is the fallback until one of:
1. A self-hosted RT runner lands and runs a weekly RT smoke job
2. GHA provides a privileged RT-kernel runner class
3. The `--rt` feature is removed or superseded

At that point, update this section to reflect the new coverage.
