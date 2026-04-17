# Dogfood Campaign Plan

**Status**: Draft design, ready to execute
**Date**: 2026-04-08
**Owner**: heyong4725 + contractor on-call rotation
**Scope**: Design and operational plan for running a realistic dora workload continuously for 7 days as pre-release evidence for any 1.0 release claim. Referenced by `plan-agentic-qa-strategy.md` Section 7.3 and `plan-dora-1.0-consolidation.md` Appendix D.

---

## 1. Purpose

A dogfood campaign is the single pre-release check no other gate can substitute for. Unit tests, integration tests, mutation testing, property testing, and adversarial review all exercise code on **small, synthetic inputs**. They catch classes of bugs that appear locally within a function or a short-lived test run. They do not catch:

- **Memory leaks** that only become visible after minutes of continuous operation
- **Shared-memory exhaustion** under realistic allocation patterns
- **Lock contention** that only manifests under sustained concurrent load
- **File handle / socket leaks** that accumulate slowly
- **Timer drift** in the HLC or scheduler
- **Recording growth** and disk exhaustion patterns
- **Graceful degradation** under OS-level pressure (low memory, high CPU, disk near-full)
- **Python GC pauses** interacting with the event loop under realistic message rates
- **Metric aggregation errors** that only show up at scale
- **Reconnect storms** when a daemon briefly disappears

These are exactly the bugs that break production deployments but pass all other gates. A dogfood campaign is the only way to surface them before users do.

## 2. Success criteria

The campaign is successful when, after 7 days of continuous operation:

| Metric | Target | Blocking? |
|---|---|---|
| Total uptime | ≥ 168 hours (no unexpected restart) | **Yes** |
| Unexpected process exits | 0 | **Yes** |
| Shared-memory OOM events | 0 | **Yes** |
| Dropped messages (per dataflow spec) | 0 | **Yes** |
| Memory growth (steady-state) | < 5% over 6 days (after 1hr warmup) | **Yes** |
| Open file handle growth | < 10 per hour | **Yes** |
| p50 latency drift | < 10% from hour 1 to hour 168 | **Yes** |
| p99 latency drift | < 20% from hour 1 to hour 168 | **Yes** |
| p99.9 latency | < 10× p50 throughout | Yes |
| CPU utilization (per node) | stable, no drift > 10% | Yes |
| Disk usage (recording + logs) | within budget | Yes |
| New GitHub issues from the dogfood | 0 | **Yes** |

Any target marked **Yes (blocking)** that fails means the campaign fails and the release is delayed. The non-blocking targets are tracked but not gating.

## 3. Workload design

A realistic workload must exercise the full stack. The reference workload:

### 3.1 Topology

```
camera_sim (Rust)                           -> 30 Hz
    | shared memory, ~2 MB per frame
vision_infer (Python, real model)           -> 30 Hz, may backpressure
    | shared memory, ~50 KB per detection
detection_filter (Rust)                     -> 30 Hz
    |-> logger (Rust, stdout + file)         -> every frame
    |-> aggregator (Rust)                    -> 1 Hz
         | TCP (< 4KB, small messages)
    metrics_sink (Rust, prometheus)          -> 1 Hz
```

**Why this topology:**
- **Rust and Python nodes mixed** — exercises both APIs and the PyO3 bridge
- **Large shared-memory messages** (2 MB frames) — the main IPC path
- **Small TCP messages** (aggregated stats) — the < 4KB control path
- **A slow consumer** (vision_infer) — exercises backpressure and queue behavior
- **Multiple fan-out** (filter -> logger + aggregator) — tests routing
- **Realistic frequencies** (30 Hz is a common sensor rate)
- **Downstream consumers at different rates** (1 Hz vs 30 Hz) — tests the rate adapters

### 3.2 Distributed across 2 machines

```
Machine A: camera_sim, vision_infer, detection_filter
Machine B: logger, aggregator, metrics_sink
```

Inter-machine communication uses Zenoh. This surfaces distributed-system bugs that a single-machine run would miss. The two machines are connected by a local network.

Alternative if only one machine is available: run both daemons on the same host with `127.0.0.1` — the distributed code path still executes, just without network variance.

### 3.3 Inputs

- **`camera_sim`**: generates synthetic RGB frames (640x480x3 = 921600 bytes, padded to 2 MB for realism) with a deterministic pattern that lets downstream nodes detect corruption. Frame number embedded in the first 16 bytes.
- **Detection**: `vision_infer` runs a real ONNX model (not a mock). Use a small YOLO variant or similar — purpose is to exercise real Python->Rust data handoff, not to benchmark AI.
- **Recording**: `.drec` recording enabled for the first hour and the last hour. Tests that the recording format survives long-running operation and that the record-node doesn't leak memory.

## 4. Monitoring

### 4.1 Metrics to collect

**Per-node metrics** (exposed via dora's prometheus endpoint or logged by each node):
- RSS memory every 10 s
- CPU time every 10 s
- Open file descriptors every 60 s
- Network bytes in/out every 10 s
- dora-specific: message count, shared-memory region count, dropped count

**Dataflow-level metrics** (from the coordinator):
- Connected daemon count
- Running node count
- Restart count per node
- WebSocket frame count (control plane)

**End-to-end latency** (instrumented in the nodes):
- `camera_sim.send_time -> vision_infer.recv_time` (hop 1)
- `vision_infer.send_time -> detection_filter.recv_time` (hop 2)
- `detection_filter.send_time -> logger.recv_time` (hop 3, cross-machine)
- `camera_sim -> logger` (end-to-end)

Latency measured by embedding the send timestamp in the Arrow metadata. Read by the consumer and diff'd against local clock.

### 4.2 Dashboard

Grafana dashboard with panels for each metric, grouped by node. Specifically:

- **Time series**: RSS, CPU, FD count, latency percentiles (p50 / p99 / p99.9) — one panel each, 168-hour window
- **Instant**: current message rate, current backpressure
- **Cumulative**: total messages, total bytes, total dropped, total restarts
- **Alerting**: any "blocking" criterion from Section 2 fires an alert

Dashboard JSON lives at `scripts/dogfood/grafana-dashboard.json` (to be created during the first run).

### 4.3 Log aggregation

All node stdout/stderr + dora's structured logs forwarded to a central log store (Loki or similar). Searchable by node name, log level, and time range. Retained for 14 days post-campaign for post-mortem.

## 5. Operational plan

### 5.1 Pre-campaign checklist

- [ ] Reference workload written and committed under `scripts/dogfood/workload/`
- [ ] Dashboard deployed
- [ ] Log aggregation live
- [ ] Both target machines provisioned with the target dora version
- [ ] `ulimit -n` and `ulimit -l` set appropriately
- [ ] `memlock` unlimited if using Zenoh SHM (post-Phase 3b)
- [ ] Disk space checked: >= 20 GB free on each machine for logs + recordings
- [ ] Baseline metrics captured at t=0 (for later comparison)

### 5.2 Kickoff

```bash
# On machine A
dora up --port 6013
dora start dogfood.yml --detach

# On machine B
dora up --port 6013 --join-coordinator <machine-A>:6013
# (nodes auto-spawn based on the dataflow spec)

# Verify
dora list
dora logs --follow
```

### 5.3 Daily checks (operational cadence)

Every 24 hours the on-call:
1. Check dashboard for anomalies (red alerts, drift)
2. Verify all nodes still running
3. Record current metrics snapshot (append to `docs/dogfood-run-YYYY-MM-DD.md`)
4. If any blocking criterion is breached: **halt the run, investigate, decide whether to restart or declare failure**

### 5.4 End-of-campaign

At t+168 hours:
1. Capture final metrics snapshot
2. Generate comparison report: day-0 baseline vs. day-7 final
3. Graceful shutdown: `dora stop dogfood` then `dora down`
4. Collect all logs, dashboard exports, and snapshots
5. Produce a `docs/dogfood-evidence-<version>.md` one-pager summarizing:
   - What workload ran
   - How long it ran (actual duration)
   - Pass/fail against each blocking criterion
   - Anomalies observed (even if not blocking)
   - Link to full dashboard export
   - Link to log archive

### 5.5 If a criterion fails

Depends on which one:

- **Hard crash**: fix the bug, re-run from scratch. Do not truncate the run.
- **Memory leak**: fix or waive with documented expected growth rate. Re-run.
- **Latency drift > 20%**: investigate root cause. Common causes: background thread buildup, timer drift, GC tuning in Python node. Fix and re-run.
- **Single dropped message after 6 days**: investigate. Usually indicates a race. Fix and re-run.

Re-running is cheap relative to the risk of shipping an untested 1.0.

## 6. Artifacts

The campaign produces, and we keep in the repo:

| Artifact | Path | Retained |
|---|---|---|
| Workload YAML + node source | `scripts/dogfood/workload/` | Permanent |
| Dashboard JSON | `scripts/dogfood/grafana-dashboard.json` | Permanent |
| Runbook | `scripts/dogfood/runbook.md` | Permanent |
| Per-run evidence report | `docs/dogfood-evidence-<version>.md` | Permanent |
| Daily snapshots | `docs/dogfood-run-YYYY-MM-DD.md` | 1 year |
| Log archive | `tar.gz` on cold storage | 6 months |
| Dashboard export (PNG) | `docs/dogfood-dashboards/<version>/` | Permanent |

## 7. Integration with the release process

1. The `dora 1.0` release tag MUST be gated on a passing dogfood run
2. The release notes cite the evidence report by path
3. The blog post announcing 1.0 includes at least one headline number from the campaign (e.g., "168 hours, zero unexpected restarts, memory flat after 1 hour warmup")
4. Subsequent minor releases (1.1, 1.2, ...) require a fresh dogfood run — not a cached pass from 1.0. Every release is its own evidence.

## 8. Cost and scheduling

**Hardware cost**: 2 machines for 7 days. If cloud-hosted, ~$50-150 depending on instance size. If on-prem, opportunity cost only.

**Human cost**:
- 1 day of setup (workload + dashboard + log aggregation)
- 15 minutes/day for daily checks during the run
- 1 day of post-run analysis and report writing

**Calendar schedule**: campaign starts >= 8 days before the planned release tag. If it fails, there's a 7-day re-run budget. If the second run fails, the release slips.

## 9. What this plan intentionally does NOT cover

- **Multi-user scenarios** (multiple CLI clients, concurrent deployments). Out of scope for the dogfood — covered by the E2E test suite.
- **Failure injection** (fault-tolerance-e2e.rs covers this; dogfood is the happy path under sustained load).
- **Full distributed deployment** (e.g., 10+ machines). The 2-machine setup tests the distributed code path; larger scale is out of scope for the POC.
- **Hardware-specific perf testing**. Dogfood tests software behavior, not raw throughput. Benchmarks in CI cover perf.

## 10. Initial rollout

For the POC on dora:

1. **Design phase (this doc)**: done 2026-04-08.
2. **Workload implementation**: deferred to when the project is ready for a 1.0 push. Estimate 1-2 days when prioritized.
3. **First run**: tied to the dora 1.0 release cycle per `plan-dora-1.0-consolidation.md`.

Until implementation, this document serves as the **specification** for what the dogfood campaign must be. It's referenced by the consolidation plan as a release gate. When a contractor is assigned to implement it, they have the full brief here.

---

## References

- [`plan-agentic-qa-strategy.md`](plan-agentic-qa-strategy.md) Section 7.3
- [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) Appendix D
- [`docs/fault-tolerance.md`](fault-tolerance.md) — complementary chaos testing (not dogfood)
- [`docs/performance.md`](performance.md) — benchmark methodology (separate from dogfood)
