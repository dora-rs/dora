# Soft Real-Time: OS-Level Tuning for Latency-Sensitive Deployments

## Context

Dora achieves 10-17x lower latency than ROS2 via zero-copy shared memory and Apache Arrow. However, it provides **no soft real-time guarantees** — no CPU pinning, no memory locking, no thread priorities. The tokio runtime uses default OS scheduling (SCHED_OTHER/CFS on Linux), so a garbage collection pause, page fault, or CPU migration can cause 100+ microsecond jitter.

For robotics control loops requiring 1-10ms determinism (arm control, drone stabilization), this jitter is unacceptable.

## Current State

| Capability | Status |
|-----------|--------|
| Zero-copy shmem (>4KB) | Yes |
| Lock-free channels (flume) | Yes |
| Async runtime (tokio multi-thread) | Yes |
| Bounded queues | Yes |
| CPU affinity | No |
| Memory locking (mlockall) | No |
| Thread priorities (SCHED_FIFO/RR) | No |
| Worker thread count config | No |
| Deterministic allocator | No |
| OS tuning documentation | Minimal |

## Approach: Operational Hardening + Optional RT Profile

Two tracks: (1) documentation/guidance for OS-level tuning, (2) optional `--rt` flag that enables CPU pinning, memory locking, and thread priorities.

## Phase 1: Documentation (Zero Code Change)

### 1.1 Create `docs/realtime-tuning.md`

OS-level tuning guide for latency-sensitive deployments:

**Linux kernel:**
```bash
# PREEMPT_RT kernel (recommended for <1ms jitter)
# Or: CONFIG_PREEMPT=y for standard kernel

# Disable CPU frequency scaling
echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Disable transparent huge pages
echo never > /sys/kernel/mm/transparent_hugepage/enabled

# Set kernel timer resolution
# Boot with: clocksource=tsc tsc=reliable
```

**Process-level:**
```bash
# Pin daemon to cores 2-3 (avoid core 0 for IRQs)
taskset -c 2,3 dora daemon

# Pin a specific node to core 4
taskset -c 4 dora run dataflow.yml

# Set real-time priority (requires CAP_SYS_NICE)
chrt -f 50 dora daemon

# Lock memory to prevent page faults
# (requires ulimit -l unlimited or CAP_IPC_LOCK)
```

**Systemd service:**
```ini
[Service]
CPUAffinity=2 3
Nice=-20
LimitMEMLOCK=infinity
LimitRTPRIO=99
```

**Container deployment (Docker/K8s):**
```yaml
# K8s pod spec
resources:
  limits:
    cpu: "2"
    memory: "4Gi"
  requests:
    cpu: "2"  # guaranteed CPU
securityContext:
  capabilities:
    add: ["SYS_NICE", "IPC_LOCK"]
```

### 1.2 Update `docs/performance.md`

Add a "Latency-Sensitive Deployment" section linking to the tuning guide.

## Phase 2: Configurable Tokio Runtime

### 2.1 Worker thread count

**File:** `binaries/daemon/Cargo.toml`, `binaries/cli/src/command/daemon.rs`

Add `--worker-threads N` CLI flag (default: num_cpus):
```rust
Builder::new_multi_thread()
    .worker_threads(args.worker_threads.unwrap_or_else(num_cpus::get))
    .enable_all()
    .build()
```

### 2.2 Tokio thread naming

Name worker threads for easier `top`/`htop` identification:
```rust
Builder::new_multi_thread()
    .thread_name("dora-daemon-worker")
    .enable_all()
```

## Phase 3: Optional RT Profile (`--rt`)

### 3.1 Memory locking

**File:** `binaries/daemon/src/lib.rs`

On startup with `--rt`:
```rust
#[cfg(unix)]
unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE); }
```

Requires `CAP_IPC_LOCK` or `ulimit -l unlimited`. Log warning if it fails.

### 3.2 Thread priorities

**File:** `binaries/daemon/src/lib.rs`

On startup with `--rt`:
```rust
#[cfg(unix)]
{
    let param = libc::sched_param { sched_priority: 50 };
    unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param); }
}
```

Requires `CAP_SYS_NICE`. Log warning if it fails.

### 3.3 CPU affinity for spawned nodes

**File:** `binaries/daemon/src/spawn/spawner.rs`

YAML option:
```yaml
nodes:
  - id: controller
    path: controller
    _unstable_deploy:
      cpu_affinity: [4, 5]  # pin to cores 4-5
```

On spawn, use `libc::sched_setaffinity` on the child process.

### 3.4 Pre-warm shmem cache

**File:** `binaries/daemon/src/node_communication/mod.rs`

On startup with `--rt`, pre-allocate shmem regions to avoid page faults on first message:
```rust
// Pre-warm: allocate and touch pages
for _ in 0..SHMEM_CACHE_SIZE {
    let region = ShmemConf::new().size(MAX_REGION_SIZE).create()?;
    unsafe { region.as_slice_mut().fill(0); } // touch pages
    cache.push(region);
}
```

## Phase 4: Deterministic Allocator (Optional)

### 4.1 jemalloc or mimalloc

Add optional jemalloc behind a feature flag:
```toml
[features]
jemalloc = ["dep:tikv-jemallocator"]
```

jemalloc has more predictable allocation patterns than the system allocator. Not a hard real-time guarantee but reduces jitter.

## Key Files

| File | Change |
|------|--------|
| `docs/realtime-tuning.md` | New — OS-level tuning guide |
| `docs/performance.md` | Update — link to tuning guide |
| `binaries/cli/src/command/daemon.rs` | `--worker-threads`, `--rt` flags |
| `binaries/daemon/src/lib.rs` | mlockall, SCHED_FIFO on `--rt` |
| `binaries/daemon/src/spawn/spawner.rs` | cpu_affinity for nodes |

## Expected Impact

| Tuning | Jitter Reduction |
|--------|------------------|
| CPU affinity (taskset) | 2-5x (avoids CPU migration) |
| Memory locking (mlockall) | Eliminates 100us+ page fault spikes |
| SCHED_FIFO priority 50 | Deterministic scheduling within 10ms |
| Disable frequency scaling | Eliminates 50-200us governor transitions |
| PREEMPT_RT kernel | Sub-100us worst-case latency |
| All combined | <1ms p99.9 for control loops |

## Risks

- **SCHED_FIFO without care can hang the system** — if daemon enters infinite loop, no other userspace process can run. Mitigate with watchdog + `SCHED_DEADLINE` (Linux 3.14+).
- **mlockall with large allocations can exhaust memory** — set `RLIMIT_MEMLOCK` appropriately.
- **CPU affinity conflicts** — if user pins daemon and nodes to same cores, they contend. Guide must recommend non-overlapping core assignments.

## Non-Goals

- Hard real-time (POSIX RT, safety-certified) — use dedicated RT frameworks (ROS2 rclcpp with real-time executor)
- Kernel bypass (DPDK, io_uring direct) — too specialized
- Bare-metal / no-OS support — dora requires Linux/macOS/Windows
