# Adora Framework Audit Report

**Date**: 2026-03-21 (combined with 2026-03-13 audit)
**Scope**: Architecture, performance, security, code quality, DX, AI agent readiness, dependencies
**Perspective**: Robotics and real-time AI agent use cases

---

## Executive Summary

Adora 0.1.0 is released. The framework has genuine strengths in latency, zero-copy IPC,
and AI agent communication patterns. Since the 2026-03-13 audit, release engineering and
CI have been the focus (12 commits), with must-fix bugs (F1-F8) resolved. However, this
deep-dive audit uncovered **3 memory-safety issues** (shmem bounds check, unsound `Sync`
impl, ARM data race), **1 code execution vulnerability** (URL downloads without integrity
verification), and **30+ correctness bugs** across daemon, coordinator, and APIs. All six
performance roadmap items from the previous audit remain unaddressed. Smoke/E2E tests do
not run in CI, meaning lifecycle regressions go undetected.

### Deep Dive Summary (New This Audit)

| Area | Critical | High | Medium | Key Finding |
|------|----------|------|--------|-------------|
| Daemon Core | 3 | 6 | 5 | Shmem OOB read; listener task leaks; log deadlock |
| Coordinator & Message | 3 | 7 | 7 | Unsound `Sync`; ARM data race; auto-recovery broken |
| APIs & CLI | 4 | 6 | 9 | Merged stream bug; C++ template UB; operator gaps |
| Security | 3 | 4 | 3 | No download verification; SSRF; no TLS |
| Tests & Examples | 2 | 5 | 4 | No E2E in CI; committed artifacts; sparse daemon tests |

### Changes Since Last Audit (2026-03-13)

| Area | Changes |
|------|---------|
| Release | CLI distribution moved to crates.io + GitHub Releases (dropped PyPI) |
| CI | git-cliff v3->v4, PyPI skip-existing, Windows zig disabled |
| Python | Smoke test false positives fixed, TryRecvError::Closed fix |
| Quality | Audit must-fix items F1-F8 applied (#56), cdylib warning removed |
| Docs | Release notes rewrite, changelog regenerated, stale docs removed |

### Previous Audit Fixes Status

| # | Fix | Status |
|---|-----|--------|
| F1 | Timer burst -> Skip | Fixed |
| F2 | `assert!` in shmem -> `eyre::bail!` | Fixed |
| F3 | `std::env::set_var` UB removed | Fixed |
| F4 | Python `todo!()` panics removed | Fixed |
| F5 | `topic echo` debug hint | Fixed |
| F6 | Zenoh channel 10 -> 100 | Fixed |
| F7 | `sysinfo` moved to `spawn_blocking` | Fixed |
| F8 | `NodeId` documented + `FromStr` | Fixed |

---

## 1. Architecture

### Strengths

- Clean layered design: CLI -> Coordinator -> Daemon -> Node
- Zero-copy shared memory for messages >= 4KB with drop-token tracking
- Built-in service/action/streaming patterns with automatic correlation IDs
- Zenoh-based distributed pub-sub across machines
- Parameter server with optional persistent backend (redb)
- Recording/replay with speed control and loop mode
- Fault tolerance: node restart policies, health-check kills, circuit-breaker
- Rich CLI: 30+ commands covering full lifecycle, monitoring, tracing, diagnostics

### Previous Roadmap Items — All Unaddressed

| Item | Effort | Status | Location |
|------|--------|--------|----------|
| `Arc<Metadata>` in fan-out | S | NOT DONE | `lib.rs:3122` — full `BTreeMap` clone per receiver |
| Shmem ID pass-through for local receivers | M | NOT DONE | `lib.rs:3207-3211` — copies to AVec even for local-only |
| Separate control/data planes | L | NOT DONE | Single `mpsc::channel(1000)` for all events |
| Remove duplicate HLC per timer | XS | NOT DONE | `running_dataflow.rs:256` — private HLC breaks causality |
| Send only assigned nodes per daemon | S | NOT DONE | Full descriptor fan-out to all daemons |
| TCP vectored write | S | NOT DONE | `socket_stream_utils.rs:18-19` — two `write_all` calls |
| Arrow IPC framing | L | NOT DONE | Enable true zero-serialization; validate types at edges |
| Type validation at dataflow startup | M | NOT DONE | Catch output/input type mismatches in `check()` |
| Optimize small message path (<4KB) | M | NOT DONE | Avoid bincode + clone per subscriber |
| Configurable tokio runtime (thread count, pinning) | S | NOT DONE | Reduce jitter for latency-sensitive deployments |
| `pending_drop_tokens` O(N) scan -> VecDeque | XS | NOT DONE | Reduce allocations at high message rates |

### New Findings

| # | Finding | Severity | Location |
|---|---------|----------|----------|
| A1 | Shmem data eagerly copied to AVec before remote-receiver check; negates zero-copy for mixed topologies | High | `lib.rs:3200-3215` |
| A2 | Per-timer private HLC instance — causality not preserved across timers; metadata timestamp diverges from shared clock | Medium | `running_dataflow.rs:256` |
| A3 | 4 blocking threads per shmem node (control, events, drop, events_close); consumes tokio blocking pool | Medium | `node_communication/mod.rs` |
| A4 | TCP: three syscalls per message (8-byte len + payload + flush) with `TCP_NODELAY` | Low-Medium | `socket_stream_utils.rs:17-20` |
| A5 | Coordinator JSON-wraps already-serialized daemon messages (double-JSON) | Low | `state.rs:104-109` |
| A6 | Drop token warning fires on legitimate zero-subscriber case; misleads debugging | Low | `running_dataflow.rs:479-513` |

### Known Constraints — Status After v0.2 Sprints

| Constraint | Original Status | v0.2 Status | Sprint |
|-----------|----------------|-------------|--------|
| Single coordinator SPOF | No recovery | **Daemon auto-reconnect + ReDB default + Recovering status** | Sprint 9 (#81) |
| Static topology | Cannot add/remove nodes | **AddNode/RemoveNode/AddMapping protocol + CLI** (coordinator dispatch pending) | Sprint 10 (#82) |
| Single-threaded event loop | Zenoh pub blocks loop | **Zenoh publish offloaded to drain task** (stop <500ms under load) | Sprint 7 (#75) |
| Arrow metadata-only | No schema negotiation | **First-message type validation + field_names/schema_hash metadata** (IPC framing pending) | Sprint 11 (#83) |
| Soft real-time only | No OS tuning | **--rt flag (mlockall + SCHED_FIFO) + comprehensive tuning guide** | Sprint 12 |
| Custom shmem IPC | 4 blocking threads/node | **Zenoh SHM data plane** (35% lower latency, 3-10x throughput) | Sprint 6 (#74) |

---

## 2. Performance (Deep Dive)

### Confirmed Strengths

- Zero-copy for >= 4KB messages: flat ~400-850us latency from 4KB to 4MB
- Per-input queue policies: DropOldest (default, size=10) and Backpressure
- LRU fairness scheduler prevents high-frequency input starvation
- Timer burst mode fixed to Skip (no more timer storms)
- Multi-threaded tokio runtime for daemon/coordinator/CLI
- No Mutex/RwLock in daemon data path — lock-free event loop design
- Zenoh publishers correctly cached (lazy-created, stored in BTreeMap)
- Coordinator not on data path — messages flow Daemon-to-Daemon via Zenoh
- Prometheus metrics feature-gated; zero overhead when disabled

### Benchmark Suite

| Suite | Location | Coverage |
|-------|----------|----------|
| Shmem round-trip | `libraries/shared-memory-server/benches/shmem_bench.rs` | 64B to 16KB |
| Daemon routing | `binaries/daemon/benches/routing.rs` | Fan-out 1/4/8, payloads 64/4096/65536 |
| Message serde | `libraries/message/benches/message_serde.rs` | Bincode ser/de 64B-1MB + metadata clone |
| E2E latency | `examples/benchmark/` | Full pipeline with shmem |

Note: No benchmark results are persisted for regression tracking. No CI benchmark job.

### Message Routing Hot Path Issues

| # | Issue | Severity | Location | Impact | Recommendation |
|---|-------|----------|----------|--------|----------------|
| P1 | **`data.clone()` per fan-out receiver** — `DataMessage::Vec(AVec<u8>)` is fully memcpy'd per local subscriber. 4MB payload with 4 receivers = 16MB memcpy per message. | **Critical** | `lib.rs:3119-3123` | Single biggest bottleneck. O(N * payload_size) per message. | Wrap `DataMessage` in `Arc`. Cloning becomes O(1) atomic ref bump. **Expected: 2-5x throughput improvement for fan-out > 1.** |
| P2 | **`metadata.clone()` per receiver** — `Metadata` contains `BTreeMap<String, Parameter>` + `ArrowTypeInfo` with nested Vecs. N heap allocations per message. | High | `lib.rs:3121` | Cumulative allocation pressure at high fan-out | Wrap in `Arc<Metadata>`. |
| P3 | **Shmem data copied unconditionally** — after local delivery, shmem region is re-opened via OS mmap and copied to `AVec` even when no remote receivers exist | High | `lib.rs:3200-3214` | Unnecessary syscall + memcpy for local-only topologies | Only perform copy when `remote_receivers` is true |

### Shared Memory Path Issues

| # | Issue | Severity | Location | Impact | Recommendation |
|---|-------|----------|----------|--------|----------------|
| P4 | **Bincode ser/de through shmem for ALL messages** — every message is `bincode::serialize` -> memcpy to shmem -> `bincode::deserialize`. For 4MB payload: 3 full passes over data. | **Critical** | `channel.rs:105,189` | Negates zero-copy; dominates latency for large messages | Split protocol: bincode for metadata header only, raw bytes for payload. **Expected: 2-3x latency reduction for large messages.** |
| P5 | **Single-message-at-a-time shmem channel** — strict send -> wait-for-signal -> read. No pipelining, no ring buffer, no concurrent messages. | Medium | `channel.rs` | Limits throughput to 1 RTT per message | Implement ring buffer or multi-region concurrent protocol |
| P6 | **3 context switches per shmem receive** — `spawn_blocking` thread + `flume::bounded(0)` rendezvous + oneshot reply | Medium | `shmem.rs:20-21` | IPC overhead per message | Batch operations or use non-blocking poll |
| P7 | **No shmem region cache for received messages** — each `send_output_to_local_receivers` calls `ShmemConf::new().os_id(...).open()` (OS mmap syscall) | Medium | `lib.rs:3200-3214` | Repeated mmap/munmap on hot path | Cache mapped regions by OS ID with LRU eviction |

### Zenoh / Inter-Daemon Path Issues

| # | Issue | Severity | Location | Impact | Recommendation |
|---|-------|----------|----------|--------|----------------|
| P8 | **Full bincode serialization for Zenoh publish** — `InterDaemonEvent::Output` containing raw `AVec<u8>` payload is bincode-serialized, copying entire payload into new `Vec<u8>` before Zenoh publish | High | `lib.rs:2342-2347` | Extra memcpy per cross-machine message | Serialize metadata separately, pass raw data via Zenoh attachments |
| P9 | **Double copy on Zenoh receive** — `s.payload().to_bytes()` copies Zenoh buffer, then `bincode::deserialize` copies data field again | Medium | `lib.rs:1859-1868` | 2 memcpy per received cross-machine message | Use `s.payload().reader()` or zero-copy deserialization |

### Allocation / Timer Issues

| # | Issue | Severity | Location | Impact | Recommendation |
|---|-------|----------|----------|--------|----------------|
| P10 | **Timer allocates BTreeMap every tick** — new `BTreeMap`, `Metadata`, `ArrowTypeInfo` with fresh Vecs per timer event | Medium | `running_dataflow.rs:263-264` | 3-5 heap allocs per tick; 1000/s at 1ms timer | Pre-allocate metadata template; cache empty metadata when telemetry disabled |
| P11 | **TCP receive allocates fresh Vec per message** — `vec![0; reply_len]` on every read | Medium | `socket_stream_utils.rs:51` | Allocation pressure for high-frequency TCP nodes | Use reusable `BytesMut` buffer per connection |
| P12 | **`ProcessesToUpdate::All` in metrics** — refreshes every OS process, not just node PIDs | Medium | `lib.rs:1293` | 10-50ms on loaded systems (thousands of processes) | Use `ProcessesToUpdate::Some(&pids)` with only node PIDs. **Expected: 10-50x faster sysinfo.** |
| P13 | Unbounded per-node event channels — slow receiver causes OOM | High | `running_dataflow.rs:159` | Memory exhaustion under load | Bounded channel with drop-oldest |

### Performance Priority Stack

Fixing P1 + P4 alone would deliver the most dramatic improvement:

```
Current:  Node A -> [bincode ser] -> [shmem write] -> [shmem read] -> [bincode deser]
                 -> [data.clone()] x N receivers -> [bincode ser for Zenoh]

Optimal:  Node A -> [metadata ser] -> [shmem write raw] -> [shmem read zero-copy]
                 -> [Arc::clone()] x N receivers -> [metadata ser + raw attach for Zenoh]
```

### Unsubstantiated Claims

- **10-17x ROS2 claim**: credible but no ROS2 comparison benchmarks in repo
- **True zero-copy**: shmem transport avoids TCP copy, but bincode ser/de (P4) means 3 full data passes; not truly zero-copy end-to-end

---

## 3. Security & Robustness

### CVE / Advisory Status

| Advisory | Crate | Severity | Impact | Action |
|----------|-------|----------|--------|--------|
| RUSTSEC-2026-0041 | `lz4_flex` 0.11.5 | High (8.2) | Uninit memory leak on invalid compressed data (via Zenoh) | Upgrade to >=0.11.6 |
| RUSTSEC-2026-0037 | `quinn-proto` 0.11.13 | High (8.7) | DoS in QUIC endpoints (via Zenoh) | Upgrade when Zenoh releases fix |
| RUSTSEC-2023-0071 | `rsa` 0.9.10 | Medium (5.9) | Marvin timing attack on RSA decryption | Monitor; used by Zenoh TLS |
| RUSTSEC-2026-0049 | `rustls-webpki` 0.103.9 | Medium | CRL matching logic bug | Upgrade when available |
| RUSTSEC-2026-0009 | `time` 0.3.45 | Medium (6.8) | Stack exhaustion DoS | Upgrade to fixed version |
| RUSTSEC-2025-0141 | `bincode` 1.3.3 + 2.0.1 | Unmaintained | No active maintainer | Plan migration away |
| RUSTSEC-2025-0057 | `fxhash` 0.2.1 | Unmaintained | No active maintainer | Replace with `rustc-hash` |
| RUSTSEC-2026-0002 | `lru` 0.12.5 | Unsound | IterMut violates Stacked Borrows | Monitor; avoid `iter_mut` |

### Passed Checks

- Constant-time token comparison (`auth.rs:45`) — manual XOR accumulator
- Bearer header auth (migrated from query param)
- Token files `0o600` with fstat ownership check (TOCTOU-resistant)
- Message size caps: 1 MiB control plane, 64 MiB Zenoh payload, 1 MiB log line
- Path traversal prevention with test coverage (`artifacts.rs:50-70`)
- All `unsafe` blocks have `# Safety` doc comments
- `umask(0o077)` before shmem/credential file creation
- Watchdog, health-check kills, circuit-breaker recovery
- Shell nodes gated by `ADORA_ALLOW_SHELL_NODES=true`
- `ENV_DENYLIST` strips `LD_PRELOAD`, `DYLD_INSERT_LIBRARIES`, auth tokens from child nodes
- `shlex` argument parsing for non-shell nodes
- Node stdin closed (`Stdio::Null`) for all spawned nodes
- UUID-based artifact build IDs prevent enumeration
- Rate limiting: per-IP fixed-window on WS, 256 connection cap

### Remaining Security Issues

| # | Issue | Severity | Location | Recommendation |
|---|-------|----------|----------|----------------|
| S1 | Auth disabled for `adora run` (embedded coordinator) — any local process can connect | High | `cli/command/run.rs:193`, `coordinator/lib.rs:116` | Add `--auth` flag or at minimum log warning |
| S2 | Unbounded per-node event channels — OOM via slow receiver | High | `running_dataflow.rs:159,162` | Bounded channel with drop-oldest |
| S3 | `node.id` embedded in Python `-c` string — structurally fragile for injection | Medium | `spawn/spawner.rs:239,265,293` | Pass as env var instead |
| S4 | Artifact served via `tokio::fs::read` — no size cap, OOM on large files | Medium | `ws_server.rs:216` | Stream with size limit |
| S5 | `IpRateLimiter` HashMap grows unboundedly — no eviction | Medium | `ws_server.rs:37-70` | LRU cap or periodic sweep |
| S6 | `conda_env` not validated for safe characters | Medium | `spawn/spawner.rs:232-240` | Validate like `NodeId` |
| S7 | 20s heartbeat timeout hardcoded | Medium | `daemon/lib.rs:609` | Make configurable |
| S8 | Shared memory channel has no post-connection auth | Medium | `channel.rs` | Document trust model |
| S9 | Windows token file created with default permissions | Low | `auth.rs:111-115` | Use Windows ACL APIs |
| S10 | `unsafe impl Send for ShmemChannel` — needs soundness verification | Low | `channel.rs:259-260` | Verify `EventImpl: Send` |
| S11 | ANSI escape sequences forwarded in log lines | Low | `spawn/prepared.rs:669-697` | Strip before forwarding |
| S12 | Shared library loaded without path canonicalization | Low | `runtime/operator/shared_lib.rs:49-52` | Canonicalize first |

### Previous Security Issues Status

| Issue | Status |
|-------|--------|
| Auth off by default | Still present (S1) |
| Unbounded per-node event channels | Still present (S2) |
| Shell-node args injection | Fixed — opt-in gate |
| Rate limiter loopback exemption | Accepted — intentional for local |
| Stale auth token after crash | Improved — `remove_token` on shutdown |
| Node stderr not sanitized | Fixed — `truncate_log_line` + UTF-8 lossy |
| bincode 1.x DoS surface | Still present + now marked unmaintained |
| 20s heartbeat timeout hardcoded | Still present (S7) |

---

## 4. Code Quality

### Metrics

| Metric | Value | Target | Status |
|--------|-------|--------|--------|
| Clippy warnings | 5 (all `needless_return` in Python API) | 0 | Close |
| `rustfmt` | Clean | Clean | Pass |
| `.unwrap()` in non-test code | ~583 occurrences in 112 files | <100 | Needs work |
| TODO/FIXME/HACK | 22 in production code | <10 | Needs work |
| Files >800 lines | 3 (daemon 4081, expand 1768, node 1621) | 0 | Needs work |

### Critical Code Quality Issues

| # | Issue | Severity | Location | Recommendation |
|---|-------|----------|----------|----------------|
| Q1 | `NodeId::from(String)` panics on invalid input — called on user-supplied YAML | Critical | `id.rs:64` | Change to `TryFrom<String>` |
| Q2 | `.unwrap()` on user-controlled ROS2 config: `config.service.as_ref().unwrap()` | Critical | `ros2-bridge-node/main.rs:156,385` | Use `.context("...")?` |
| Q3 | `.unwrap()` in tracing setup in `tokio::spawn` — silent panic drop | Critical | `node/mod.rs:1304,1306,1314` | Log error, fall back |
| Q4 | `to_py_dict` error silently replaced with empty dict in `drain()` | Critical | `python/node/lib.rs:232-234` | Return error or log |
| Q5 | `daemon/src/lib.rs` at 4081 lines — monolith, hard to review/test | High | `daemon/src/lib.rs` | Split into modules |
| Q6 | `serde_json::to_vec(...).unwrap()` x25 in CLI — bypasses helper | High | `cli/src/common.rs` + 15 files | Route through `send_control_request()` |
| Q7 | `record_event(...).unwrap()` — panics on disk write failure | High | `event_stream/mod.rs:377` | Propagate or log |
| Q8 | Python `__next__` docstring says "2s timeout" but blocks indefinitely | Medium | `python/node/lib.rs:271-273` | Fix docstring |
| Q9 | `serde_yaml` 0.9.x — soft-deprecated, known merge key issues | Medium | `Cargo.toml:107` | Track migration to `serde_yml` |
| Q10 | Node communication handlers duplicated across shmem/tcp/unix_domain | Medium | `node_communication/mod.rs:35` | Unify protocol handling |
| Q11 | Python tracing init spawns infinite `sleep(1s)` loop to keep guard alive | Medium | `python/node/lib.rs:146-151` | Store guard in `Node` struct |
| Q12 | Inconsistent `serde_json` version pinning across crates | Low | Multiple `Cargo.toml` | Use `{ workspace = true }` |

### TODO Debt (22 items, highlights)

| Location | TODO | Impact |
|----------|------|--------|
| `node_communication/mod.rs:35` | Unify shmem/tcp/unix handlers | Protocol changes require 3x edits |
| `spawn/spawner.rs:211` | Multi-operator Python runtime | Silently unsupported |
| `operator/src/lib.rs:64` | Open telemetry context for operators | Operators lack observability |
| `validate.rs:48,77,91,98` | URL validation for sources | URL sources not validated |
| `cli/template/c/mod.rs:110` | Makefile for C projects | C users expect `make` |

---

## 5. Developer Experience

### Strengths

- Rich CLI: `adora run`, `doctor`, `top`, `topic echo/hz/pub`, `param`, record/replay
- Clean Python API with sync/async support and `for event in node:` iteration
- YAML dataflow spec with comprehensive docs (855-line architecture doc)
- Service/Action/Streaming patterns with zero boilerplate (Rust)
- Hot reload for Python operators
- 28 example directories covering Rust, Python, C, C++, ROS2, benchmark, service, action
- `adora doctor` provides comprehensive system diagnostics
- `adora new` scaffolds complete projects with `Cargo.toml`/`pyproject.toml` + YAML

### DX Issues

| # | Issue | Severity | Location | Recommendation |
|---|-------|----------|----------|----------------|
| D1 | **No quickstart guide** — 21 docs but no single 5-minute onramp | High | `docs/` | Create `docs/quickstart.md` |
| D2 | **Python API missing service/streaming helpers** — must manually set `request_id`, `session_id` metadata | High | `python/node/lib.rs` | Add `send_service_request()`, `send_service_response()`, `send_stream_chunk()` |
| D3 | **Operator API has no metadata access** — `Event::Input` lacks `metadata`; operators cannot do service/action/streaming | High | `operator/src/lib.rs:35-56` | Add `metadata: Metadata` to `Event::Input` |
| D4 | **Operator API type inconsistency** — input `id` is `&str`, output `send()` takes `String`; Node API uses `DataId` | High | `operator/src/lib.rs` | Align with Node API types |
| D5 | **Python template bug** — checks `event["id"] == "TICK"` (uppercase) but timer uses lowercase `tick` | High | `template/python/__node-name__/main.py:13` | Fix to lowercase `tick` |
| D6 | **No streaming pattern example** — service and action have examples, streaming does not | Medium | `examples/` | Add `examples/streaming-example/` |
| D7 | Python `close_outputs` missing — no way to signal completion to downstream | Medium | `python/node/lib.rs` | Add `close_outputs()` |
| D8 | `adora new --kind operator` not supported but referenced in operator docs | Medium | `cli/src/lib.rs:15` | Add operator kind or fix docs |
| D9 | `adora new` doesn't print next-step instructions | Low | `cli/src/command/new.rs` | Print `cd ... && adora build && adora run` |
| D10 | No doc index page linking all 21 docs | Low | `docs/` | Add `docs/README.md` |
| D11 | `examples/action-example/out/` — runtime artifacts committed to git | Low | `examples/` | Add to `.gitignore` |
| D12 | Python bytes API produces unreadable output | High | Python node API | Document send/receive pair for bytes case |
| D13 | Python metadata convention inconsistent | High | Python node API | Unify `metadata=` dict format |
| D14 | `_unstable_*` naming with no stability legend | Medium | Public APIs | Add stability docs + graduation criteria |
| D15 | No canonical robotics example | Medium | `examples/` | Add sensor -> filter -> pose pipeline |
| D16 | `Reload` event variant exposed in public API | Low | `apis/rust/node` | Add `#[doc(hidden)]` |
| D17 | `ParamUpdate` missing from api-rust.md | Low | `docs/api-rust.md` | Update docs |
| D18 | `adora doctor` doesn't check Python version | Low | `cli/src/` | Wire existing `get_python_adora_version()` |

### ROS2 Bridge Status

Feature-complete: topics (sub/pub/multi), services (client/server), actions (client/server), QoS.
Known gaps: no action cancel forwarding, no `wait_for_action_server`, node discovery blocked by ros2-client upstream bug.

---

## 6. AI Agent Readiness

### Excellent

| Pattern | Status | Details |
|---------|--------|---------|
| Service (request/reply) | Production-ready | Auto-correlation via UUIDv7 `request_id` |
| Action (goal/feedback/result) | Production-ready | 64 concurrent goals, cancellation support |
| Streaming (token generation) | Production-ready | Session/segment/chunk with `flush` interrupt |
| Python ML integration | Production-ready | NumPy, PyTorch, Transformers, CUDA IPC |
| Recording/replay | Production-ready | `.adorec` format, speed control, loop, proxy mode |
| Parameter server | Production-ready | Coordinator K/V store, optional persistence |
| Distributed deployment | Production-ready | Multi-daemon via Zenoh, machine labels |

### Gaps

| Gap | Severity | Impact |
|-----|----------|--------|
| Python missing service/streaming helpers (D2) | High | AI agent devs primarily use Python; must manually handle correlation IDs |
| Operator API missing metadata (D3) | Medium | Operators cannot participate in agent pipelines |
| No AI agent guide/tutorial | Medium | No reference architecture doc for LLM/STT/TTS pipelines |
| No streaming example | Medium | Pattern exists but no working example to copy from |
| No mid-execution topology changes | Medium | Cannot dynamically add agent nodes |
| No declarative agent DSL | Low | Write agent loop in Python/Rust (flexible) |
| 64KB parameter value limit | Low | Use external DB for large state |

### Recommended AI Agent Architecture (Unchanged)

```
                    +-------------------+
                    |   Coordinator     |
                    |  (param store)    |
                    +--------+----------+
                             |
              +--------------+--------------+
              |              |              |
         +----+----+   +----+----+   +----+----+
         | Daemon A |   | Daemon B |   | Daemon C|
         +----+----+   +----+----+   +----+----+
              |              |              |
    +---------+------+   +--+--+    +------+------+
    |         |      |   |     |    |      |      |
 Sensor    LLM    Plan  Arm   Nav  Camera  YOLO  TTS
  Node    Agent   ner  Ctrl  Ctrl   Node   Node  Node
```

- **Planner -> Executors**: Action pattern (goal + feedback)
- **LLM Agent -> Tools**: Service pattern (request/reply)
- **Camera -> Detector -> Planner**: Topic dataflow (pub/sub)
- **LLM -> TTS**: Streaming pattern (token-by-token with interrupt)
- **All agents -> Param server**: Shared state coordination

---

## 7. Dependencies (Deep Dive)

### Metrics

| Metric | Value | Concern |
|--------|-------|---------|
| Total packages in Cargo.lock | **827** | Very large; dominated by Zenoh |
| Duplicate crate versions | **89** entries (crates with 2+ versions) | High; increases compile time |
| Security advisories | **5 CVEs** + **8 unmaintained** | See Section 3 |
| Zenoh transitive deps | **~935 lines** in `cargo tree` | Single largest contributor |

### High-Impact Dependency Issues

| # | Severity | Crate | Issue | Recommendation |
|---|----------|-------|-------|----------------|
| DEP1 | **High** | `arrow` | Default features pull in `arrow-csv`, `arrow-ipc`, `arrow-json`, `arrow-row`, `arrow-string`, `arrow-cast`, `arrow-arith`, `arrow-select`. Zero usage of `arrow_csv` or `arrow_ipc` found. ~6 sub-crates compiled unnecessarily. | Set `default-features = false`, enable only `arrow-array`, `arrow-data`, `arrow-schema`, `arrow-buffer`, `arrow-select`. Add `json`/`ffi`/`pyarrow` only where needed. |
| DEP2 | **High** | `serde_yaml` 0.9.x | Officially deprecated and unmaintained. Used in 7+ crates. Known merge key issues. | Migrate to `serde_yml` (maintained fork) |
| DEP3 | **High** | `tokio` | All three binaries use `features = ["full"]` — enables `test-util`, `io-std`, `signal`, etc. unnecessarily | Use only needed features: `rt-multi-thread`, `macros`, `net`, `sync`, `fs`, `process`, `time`, `io-util` |
| DEP4 | **High** | `zenoh` | Workspace pins `"1.1.1"` but resolves to 1.7.2. Default features enable ALL transports (quic, quic_datagram, tcp, tls, udp, unixsock, websocket). **935 transitive dep lines.** | Set `default-features = false`, enable only needed transports (tcp, maybe tls). Could halve dep tree. |
| DEP5 | **Medium** | `git2` | Enables `ssh` + `vendored-openssl`. SSH likely not needed. Vendored OpenSSL adds significant compile time. | `default-features = false, features = ["vendored-openssl", "https"]` to drop SSH |
| DEP6 | **Medium** | `once_cell` | Used in `adora-message` and `adora-core`. MSRV 1.85 has `std::sync::LazyLock` stabilized. Only 1 file uses it. | Replace with `std::sync::LazyLock`, drop dep |
| DEP7 | **Medium** | `libloading` | Pinned to 0.7.3 while 0.8.x exists in tree (from Zenoh). Causes duplicate. | Bump to `libloading = "0.8"` |
| DEP8 | **Medium** | `inquire` in `adora-node-api` | TUI prompt library in a **library crate**. Pulls `console` + terminal deps into every node. Used for one interactive debug feature. | Move behind feature flag `interactive` (off by default) or move to CLI |
| DEP9 | **Medium** | `criterion` | `html_reports` feature enabled in 3 bench crates. Pulls in `plotters` (heavy compile). | Use `default-features = false` for dev builds; enable `html_reports` only when generating reports |
| DEP10 | **Medium** | `which` | v5.0.0 (daemon, core) and v7.0.3 (CLI) — two versions compiled | Align all to `which = "7"` |

### Duplicate Dependencies (Notable)

| Crate | Versions | Cause |
|-------|----------|-------|
| `nix` | 0.23, 0.26, 0.29, 0.30 | `raw_sync_2`, `shared_memory_extended`, zenoh |
| `thiserror` | v1.0.69, v2.0.18 | Transitive deps still on v1 |
| `hashbrown` | 0.14, 0.15, 0.16 | Various transitive chains |
| `crossterm` | 0.28, 0.29 | ratatui (0.28) vs CLI (0.29) |
| `flume` | 0.10, 0.11 | Direct (0.10) vs zenoh transitive (0.11) |
| `sysinfo` | 0.35, 0.36 | Zenoh (0.35) vs direct (0.36) |
| `getrandom` | 0.2, 0.3, 0.4 | Different rand/uuid chains |
| `syn` | v1, v2 | Older proc-macro deps (v1) vs current (v2) |

### Version Pinning Inconsistencies

Workspace mixes exact (`"1.1.1"`), caret (`"0.9.33"`), and major-only (`"1.0.145"`) pinning. Several crates re-declare deps instead of using `{ workspace = true }`:
- `eyre` appears as `"0.6.7"`, `"0.6.8"`, `"0.6.12"` across crates
- `serde_json` at `"1.0.86"` (daemon, coordinator) vs `"1.0.145"` (workspace)

**Recommendation**: Centralize ALL shared deps in `[workspace.dependencies]`: `eyre`, `tokio`, `uuid`, `tracing`, `futures`, `bincode`, `aligned-vec`, `chrono`, `dunce`, `itertools`, `ctrlc`, `flume`.

### Build & Binary Size

| Setting | Current | Recommendation |
|---------|---------|----------------|
| `[profile.release]` | Not configured | Add `lto = "thin"` for default release builds |
| `[profile.dist]` | `lto = "fat"`, `codegen-units = 1` | Good for distribution |
| `adora-cli` binary | Embeds daemon + coordinator + runtime | Intentional for `adora run`; could be feature-gated |
| `self_update` in CLI | ~15 direct deps for auto-update | Could be feature-gated |
| `webbrowser` in CLI | Used in 1 line (`graph.rs:72`) | Replace with lighter `open::that()` |
| `raw_sync_2 = "=0.1.5"` | Exact pin blocking upgrades | Track upstream PR; consider vendoring |

### Compile Time Hotspots (Estimated)

1. **Zenoh** — 20+ sub-crates, QUIC/TLS stack, proc macros
2. **git2 + vendored-openssl** — compiles OpenSSL from C source
3. **arrow** (15 sub-crates, ~6 unnecessary)
4. **OpenTelemetry** -> `tonic` -> `prost` (protobuf codegen) -> `syn`
5. **criterion + plotters** in dev-deps
6. **ratatui + crossterm** — TUI framework for 2 CLI commands

---

## 8. Prioritized Roadmap

### Phase 1: Stability & Correctness (v0.1.x) — Must Fix

**Panics & Safety:**
- [ ] **Q1**: `NodeId::from(String)` -> `TryFrom<String>` (panic on user input)
- [ ] **Q2**: ROS2 bridge `.unwrap()` on config -> `.context()?`
- [ ] **Q3**: Tracing setup `.unwrap()` in `tokio::spawn` -> log + fallback
- [ ] **Q4**: Python `drain()` error silently swallowed -> return error or log

**Security:**
- [ ] **S1**: Add `--auth` flag to `adora run` or log warning
- [ ] **S2/P13**: Bounded per-node event channels with drop-oldest

**Performance (highest ROI):**
- [ ] **P1**: Wrap `DataMessage` in `Arc` for fan-out (eliminates O(N*size) memcpy — **biggest single win**)
- [ ] **P2**: Wrap `Metadata` in `Arc` for fan-out

**DX:**
- [ ] **D5**: Fix Python template `"TICK"` -> `"tick"`

**Dependencies:**
- [ ] **CVE**: Upgrade Zenoh (resolves `lz4_flex`, `quinn-proto`)
- [ ] **DEP1**: `arrow` `default-features = false` (drop ~6 unused sub-crates)
- [ ] Fix 5 clippy `needless_return` warnings

### Phase 2: Performance & DX (v0.2)

**Performance:**
- [ ] **P4**: Split shmem protocol: bincode for metadata only, raw bytes for payload (**2-3x latency reduction**)
- [ ] **P3**: Only copy shmem data when remote receivers exist
- [ ] **P8**: Separate metadata ser from payload for Zenoh publish
- [ ] **P10**: Pre-allocate timer metadata template (eliminate 3-5 allocs/tick)
- [ ] **P12**: `ProcessesToUpdate::Some(&pids)` for sysinfo (**10-50x faster metrics**)
- [ ] **P5**: TCP vectored write or `BufWriter`
- [ ] **A2**: Remove per-timer HLC; use shared daemon clock only

**DX:**
- [ ] **D1**: Create `docs/quickstart.md` (5-minute getting started)
- [ ] **D2**: Python service/streaming helpers (`send_service_request`, `send_stream_chunk`)
- [ ] **D3**: Add metadata to Operator API `Event::Input`
- [ ] **D4**: Align Operator API types with Node API (`DataId`, `&str` send)
- [ ] **Q5**: Split `daemon/src/lib.rs` (4081 lines) into modules
- [ ] Add streaming pattern example
- [ ] ROS2 side-by-side benchmark to substantiate 10-17x claim

**Dependencies:**
- [ ] **DEP4**: Zenoh `default-features = false` — enable only needed transports
- [ ] **DEP3**: Tokio `features = ["full"]` -> explicit feature list
- [ ] **DEP6**: Replace `once_cell` with `std::sync::LazyLock`
- [ ] **DEP8**: Move `inquire` behind feature flag in `adora-node-api`
- [ ] **DEP10**: Align `which` to v7 across all crates
- [ ] Centralize all deps in `[workspace.dependencies]`

### Phase 3: Scale & Hardening (v0.3+)

**Architecture:**
- [ ] Coordinator standby / leader election
- [ ] Dynamic topology (mid-execution node add/remove)
- [ ] Separate control/data planes in daemon event loop
- [ ] **P5**: Shmem ring buffer / concurrent messaging protocol

**Security:**
- [ ] **S4**: Stream artifact downloads with size cap
- [ ] **S5**: LRU-capped rate limiter map
- [ ] Configurable heartbeat timeout (S7)
- [ ] Token expiry and rotation

**Code Quality:**
- [ ] Reduce `.unwrap()` count from ~583 to <100
- [ ] Node communication handler unification (Q10)
- [ ] Arrow type validation at dataflow startup

**Dependencies:**
- [ ] **DEP2**: `serde_yaml` -> `serde_yml` migration
- [ ] bincode migration (unmaintained — RUSTSEC-2025-0141)
- [ ] **DEP5**: Drop `git2` SSH + vendored-openssl bloat
- [ ] Add `[profile.release]` with `lto = "thin"`
- [ ] CI benchmark regression tracking

**DX:**
- [ ] AI agent guide / tutorial document
- [ ] `#[adora::node]` proc macro for Rust boilerplate
- [ ] Distributed parameter server

---

## Appendix A: Dependency Advisories (Full)

| Advisory | Crate | Type | Severity |
|----------|-------|------|----------|
| RUSTSEC-2026-0041 | `lz4_flex` 0.11.5 | Vulnerability | High (8.2) |
| RUSTSEC-2026-0037 | `quinn-proto` 0.11.13 | Vulnerability | High (8.7) |
| RUSTSEC-2023-0071 | `rsa` 0.9.10 | Vulnerability | Medium (5.9) |
| RUSTSEC-2026-0049 | `rustls-webpki` 0.103.9 | Vulnerability | Medium |
| RUSTSEC-2026-0009 | `time` 0.3.45 | Vulnerability | Medium (6.8) |
| RUSTSEC-2025-0141 | `bincode` 1.3.3 + 2.0.1 | Unmaintained | - |
| RUSTSEC-2025-0057 | `fxhash` 0.2.1 | Unmaintained | - |
| RUSTSEC-2020-0016 | `net2` 0.2.39 | Deprecated | - |
| RUSTSEC-2025-0119 | `number_prefix` 0.4.0 | Unmaintained | - |
| RUSTSEC-2024-0436 | `paste` 1.0.15 | Unmaintained | - |
| RUSTSEC-2025-0134 | `rustls-pemfile` 2.2.0 | Unmaintained | - |
| RUSTSEC-2026-0012 | `keccak` 0.1.5 | Unsound | - |
| RUSTSEC-2026-0002 | `lru` 0.12.5 | Unsound | - |

Most are transitive through Zenoh 1.7.2. Upgrading Zenoh will resolve the majority.

---

## Appendix B: Deep Dive Findings

### B1. Daemon Core

#### Critical

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| DC1 | **Unsafe shmem read with no length validation** — `len` from IPC channel used to index shmem region without bounds check. Malicious/buggy node can cause OOB read. | `lib.rs:3207-3211` | Memory safety violation |
| DC2 | **TCP/UDS listener loops never terminate** — `loop` around `listener.accept()` has no exit condition. Listeners leak as zombie tasks when dataflows finish. | `spawn/prepared.rs:567-585` | Resource leak over time |
| DC3 | **4 blocking threads per shmem node** — each node spawns 4 `spawn_blocking` threads (control, events, drop, events_close). 10 nodes = 40 blocking threads. | `node_communication/mod.rs` | Thread pool exhaustion |

#### High

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| DC4 | **Log pipeline deadlock under backpressure** — daemon_tx(1000) fills -> log forwarder blocks -> child stdout pipe fills -> child process blocks on write | `lib.rs:467`, `prepared.rs:629-860` | Silent node stall |
| DC5 | **`fsync` called per log line** — `sync_all()` on every log line write; 1000 lines/s = 1000 fsync/s | `prepared.rs:844` | Severe I/O bottleneck |
| DC6 | **Python metrics only aggregate direct children** — `uv` -> `python` grandchild CPU/memory not captured | `lib.rs:1317-1325` | Incorrect metrics for Python nodes |
| DC7 | **`AllNodesReady` race is fatal** — if coordinator sends before local nodes finish, daemon bails with hard error | `pending.rs:131-133` | Crash on slow machines |
| DC8 | **Python operator count check is off-by-one** — `> 2` should be `> 1`; silently allows 2 Python operators | `spawn/spawner.rs:211-215` | Incorrect validation |
| DC9 | **Grace-period kill triggers spurious Drop warning** — `ProcessHandle::Drop` sends redundant Kill after explicit Kill | `running_dataflow.rs:302-371` | Misleading logs |

#### Medium

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| DC10 | Blocking `std::fs::create_dir_all` in async context — stalls event loop on slow FS | `spawner.rs:144`, `prepared.rs:430` | Latency spike |
| DC11 | HLC update failure swallowed with only warning — no source attribution | `lib.rs:559` | Debugging difficulty |
| DC12 | `read_last_n_lines` buffer estimation slightly off | `lib.rs:2993-3041` | Minor extra allocation |
| DC13 | Log self-delivery check fails when `node_id` is None — potential amplification loop | `lib.rs:2781-2786` | Log flooding |
| DC14 | `std::mem::forget(tx)` for permanent dead channel — intentional but undocumented leak | `lib.rs:3498-3533` | Minor resource leak |

### B2. Coordinator & Message

#### Critical

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| CM1 | **Shmem channel data race on ARM** — `send_raw` write + Release store + event signal may not ensure happens-before on non-x86 without fence in event impl | `channel.rs:119-143` | Silent data corruption on ARM |
| CM2 | **Unsound `Sync` impl on `ShmemChannel`** — struct contains non-`Sync` raw pointers; concurrent `&ShmemChannel` access = data race | `channel.rs:259-260` | Undefined behavior |
| CM3 | **Server drop without client signal** — if server drops before client disconnects, client blocks forever in `receive()` with `Timeout::Infinite` | `channel.rs:262-286` | Hang on cleanup |

#### High

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| CM4 | `expect` in token generation — `getrandom` failure panics coordinator | `auth.rs:59` | Process crash |
| CM5 | `unwrap` on semver parse — custom build env could crash | `lib.rs:89`, `node_to_daemon.rs:86` | Process crash |
| CM6 | 10 `unwrap()` calls in prometheus metric registration | `prometheus_metrics.rs:28-57` | Crash on metric name collision |
| CM7 | Auto-recovery hardcodes `uv: false` — Python UV dataflows silently fail on recovery | `handlers.rs:601-608` | Recovery failure |
| CM8 | `TopicPublish` validation reuses `TopicSubscribe` check incorrectly — can't publish to dynamic outputs | `ws_control.rs:312-316` | Feature limitation |
| CM9 | Heartbeat timeout doesn't clean up `running_dataflows` — orphaned entries persist | `lib.rs:1241-1249` | State corruption |
| CM10 | `found_rx.await.unwrap_or(false)` x4 — masks coordinator panics as "not found" | `ws_control.rs:150,168,213,317` | Error masking |

#### Medium

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| CM11 | `finished_builds` HashMap never pruned — grows unboundedly | `lib.rs:246` | Memory leak |
| CM12 | `dataflow_results` / `archived_dataflows` never pruned | `lib.rs:249` | Memory leak |
| CM13 | `destroy_daemons` fails fast on first error — remaining daemons not destroyed | `handlers.rs:601-606` | Partial cleanup |
| CM14 | MAX_WS_CONNECTIONS=256 is global — log streaming can starve daemon connections | `ws_server.rs` | Connection starvation |
| CM15 | `send_log_message` iterates subscribers sequentially with 100ms timeout each — N*100ms blocking | `handlers.rs:73` | Event loop stall |
| CM16 | No upper bound on MetadataParameters map size | `metadata.rs:48` | Memory exhaustion |
| CM17 | `NodeId(node.to_owned())` in `InputMapping::from_str` bypasses validation | `config.rs:272` | Invalid IDs accepted |

### B3. APIs & CLI

#### Critical

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| AC1 | **Panics in `init_tracing` async task** — 3 `.unwrap()` calls in spawned task; silent tracing loss | `node/mod.rs:1304,1306,1314` | Silent feature loss |
| AC2 | **`record_event().unwrap()`** — I/O error panics node's main event loop | `event_stream/mod.rs:377` | Node crash on disk error |
| AC3 | **Python tracing `.unwrap()` in shared RUNTIME** — destabilizes all Python nodes | `python/node/lib.rs:147` | Process-wide instability |
| AC4 | **`ws_client` `.expect()` on request build** — reachable from invalid token content | `ws_client.rs:94` | CLI crash |

#### High

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| AC5 | **`drain()` returns `None` for merged streams** — signals "closed" for active stream | `python/node/lib.rs:233-237` | Python event loop premature exit |
| AC6 | **`try_recv` returns `Closed` for merged streams** — same semantic error | `python/node/lib.rs:579-583` | Incorrect closed detection |
| AC7 | **C++ template UB** — `std::string(result.error)` when `result.error.ptr` is null | `template/cxx/node-template.cc:27` | UB in generated projects |
| AC8 | **Operator OTel context always empty** — `String::new()` with `// TODO` | `operator/lib.rs:64` | Broken distributed tracing |
| AC9 | **Operator missing event variants** — no `InputRecovered`, `ParamUpdate`, `Reload` | `operator/lib.rs` | No graceful degradation |
| AC10 | **Runtime `.unwrap()` on operator channel get** — race can panic runtime | `runtime/lib.rs:244` | Runtime crash |

#### Medium

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| AC11 | `eprintln!` in production code path | `node/mod.rs:513` | Bypass tracing |
| AC12 | Shmem cache `clear()` on budget exceed — cliff-edge eviction | `node/mod.rs:1118` | Cache thrashing |
| AC13 | WS session doesn't clean up `topic_subscribers` on disconnect | `ws_client.rs:384-393` | Hanging receivers |
| AC14 | Python `__next__` docstring says "2s timeout" but blocks indefinitely | `python/node/lib.rs:271-273` | Misleading docs |
| AC15 | Rust template: unused `metadata` variable generates compiler warning | `template/rust/node/main-template.rs` | Bad first impression |
| AC16 | Operator template: unused `data` variable generates compiler warning | `template/rust/operator/lib-template.rs` | Bad first impression |
| AC17 | Python template: `"TICK"` should be `"tick"` | `template/python/main.py` | Node never fires |
| AC18 | Node `log()` uses `chrono::Utc::now()` instead of HLC — different clock domain | `node/mod.rs:886` | Uncorrelatable logs |
| AC19 | `hz.rs` mutex poison handling inconsistent — line 110 safe, line 125 panics | `cli/command/topic/hz.rs:110,125` | Inconsistent crash |

### B4. Security (Comprehensive)

#### Critical

| # | Finding | OWASP | Location | Impact |
|---|---------|-------|----------|--------|
| SEC1 | **Downloaded binaries executed without integrity verification** — no checksum, signature, or hash on URL-sourced nodes | A08 | `download/lib.rs:44-76`, `spawn/command.rs:51` | Arbitrary code execution via MITM |
| SEC2 | **`source_is_url` accepts any scheme** — `file://`, `gopher://`, `smb://` pass through to reqwest | A10 | `descriptor/mod.rs:322-324` | SSRF / local file read |
| SEC3 | **No TLS on any transport** — auth tokens transmitted in cleartext over `ws://` and plain TCP | A05 | All transports | Token theft on shared networks |

#### High

| # | Finding | OWASP | Location | Impact |
|---|---------|-------|----------|--------|
| SEC4 | Full dataflow descriptor serialized to child process env vars — world-readable via `/proc/<pid>/environ` | A09 | `spawner.rs:157-159,320-322` | Topology/config leak |
| SEC5 | `download_file` buffers entire response in memory — no size limit | A04 | `download/lib.rs:57-60` | OOM via large download |
| SEC6 | `Content-Disposition` parsed with manual string split — fragile | A03 | `download/lib.rs:8-14` | Unexpected filenames |
| SEC7 | `conda_env` not validated — leading dashes can manipulate conda args | A05 | `spawner.rs:226,236` | Argument injection |

### B5. Examples & Tests

#### Critical

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| ET1 | **Smoke/E2E/fault-tolerance tests absent from CI** — only `cargo test --all` runs; the 30+ lifecycle tests never execute | `.github/workflows/ci.yml` | Regressions undetected |
| ET2 | **Run artifact log files committed to git** — 12+ example `out/` dirs contain JSONL logs from past runs | `examples/*/out/`, `tests/dataflows/out/` | Repository bloat |

#### High

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| ET3 | Two full-stack E2E tests permanently `#[ignore]`d with no CI path | `tests/ws-cli-e2e.rs:431,477` | Dead test guards |
| ET4 | Smoke test `run_smoke_test` can't distinguish success from crash — only checks for absence of "Running" | `tests/example-smoke.rs:195` | False passes |
| ET5 | Daemon monolith (4081 lines) has only 8 unit tests | `binaries/daemon/src/lib.rs` | Critical code undertested |
| ET6 | No tests for fault_tolerance.rs, spawner.rs, pending.rs, socket_stream_utils.rs | `binaries/daemon/src/` | Zero coverage on critical paths |
| ET7 | Recording library has only 7 unit tests — no edge case coverage | `libraries/recording/src/lib.rs` | Corruption undetected |

#### Medium

| # | Finding | Location | Impact |
|---|---------|----------|--------|
| ET8 | `typed-dataflow` example has no smoke test and no README | `examples/typed-dataflow/` | Type system feature untested |
| ET9 | Benchmarks not in CI, no regression thresholds | `benches/` dirs | Performance regressions undetected |
| ET10 | `routing.rs` benchmark uses `current_thread` runtime — underestimates | `daemon/benches/routing.rs` | Misleading numbers |
| ET11 | Smoke tests require `--test-threads=1` with no enforcement | `tests/example-smoke.rs` | Flaky when parallelized |

---

## Appendix C: Updated Roadmap (Incorporating Deep Dive)

### Immediate (v0.1.1 patch) — Safety & Correctness

- [ ] **DC1**: Bounds-check shmem `len` before indexing (`memory.len() >= len`)
- [ ] **CM2**: Remove `unsafe impl Sync for ShmemChannel`
- [ ] **SEC1**: Add sha256 digest verification for URL-sourced node downloads
- [ ] **SEC2**: Restrict URL schemes to `https://` only
- [ ] **DC7**: Convert `AllNodesReady` race from `bail!` to warn + retry
- [ ] **DC8**: Fix Python operator count check `> 2` -> `> 1`
- [ ] **AC5/AC6**: Fix `drain()`/`try_recv()` for merged streams (return `Some(vec![])` / `Empty`)
- [ ] **AC7**: Fix C++ template null pointer UB
- [ ] **ET2**: Add `**/*.jsonl` and `**/log_*` to example `.gitignore` files; remove committed artifacts
- [ ] **AC15/AC16/AC17**: Fix template compiler warnings and `"TICK"` bug

### Short-term (v0.1.x) — Reliability

- [ ] **DC2**: Thread cancellation token into listener loops; break on dataflow finish
- [ ] **DC4**: Use `try_send` for all log-related daemon_tx sends (prevent deadlock)
- [ ] **DC5**: Remove per-line `sync_all()` from log writer
- [ ] **DC6**: Recursive child process aggregation for Python metrics
- [ ] **CM3**: Signal client event on server drop (prevent infinite hang)
- [ ] **CM7**: Preserve `uv` flag in auto-recovery
- [ ] **CM9**: Clean up `running_dataflows` on daemon heartbeat timeout
- [ ] **CM11/CM12**: Add TTL eviction for `finished_builds` and `archived_dataflows`
- [ ] **ET1**: Add CI E2E job running smoke, ws-cli-e2e, and fault-tolerance tests
- [ ] **ET3**: Remove `#[ignore]` from E2E tests or add `--ignored` CI step
- [ ] **ET4**: Check for "Failed" state in `run_smoke_test` polling loop

### Medium-term (v0.2) — Performance & Hardening

(Existing Phase 2 items plus:)
- [ ] **CM1**: Add explicit memory fence before shmem event signal (ARM safety)
- [ ] **SEC3**: Add TLS support for coordinator WebSocket
- [ ] **SEC4**: Pass node config via temp file instead of env vars
- [ ] **CM15**: Use `FuturesUnordered` for concurrent log subscriber delivery
- [ ] **CM14**: Separate connection limits per endpoint (daemon vs CLI)
- [ ] **ET5/ET6**: Expand daemon unit tests for routing, restart, pending delivery
- [ ] **ET9**: Add CI benchmark job with regression tracking

---

## Appendix D: Files Referenced

### Daemon
- `binaries/daemon/src/lib.rs` — main event loop, routing, metrics (4081 lines)
- `binaries/daemon/src/running_dataflow.rs` — timer setup, subscriber channels, drop tokens
- `binaries/daemon/src/node_communication/mod.rs` — shmem/tcp/uds dispatch
- `binaries/daemon/src/node_communication/shmem.rs` — shmem listener loop
- `binaries/daemon/src/node_communication/tcp.rs` — TCP listener loop
- `binaries/daemon/src/socket_stream_utils.rs` — TCP message framing
- `binaries/daemon/src/spawn/spawner.rs` — node process spawning, env handling
- `binaries/daemon/src/spawn/prepared.rs` — node lifecycle, log pipeline
- `binaries/daemon/src/spawn/command.rs` — command construction, shlex parsing
- `binaries/daemon/src/fault_tolerance.rs` — circuit breaker, restart policies
- `binaries/daemon/src/pending.rs` — deferred input delivery
- `binaries/daemon/benches/routing.rs` — fan-out routing benchmark

### Coordinator
- `binaries/coordinator/src/lib.rs` — coordinator event loop, heartbeat (900+ lines)
- `binaries/coordinator/src/ws_server.rs` — WebSocket server, rate limiter, auth
- `binaries/coordinator/src/ws_control.rs` — CLI-facing control handlers
- `binaries/coordinator/src/ws_daemon.rs` — daemon-facing message routing
- `binaries/coordinator/src/state.rs` — daemon connection state, pending replies
- `binaries/coordinator/src/handlers.rs` — build/start/stop/log handlers
- `binaries/coordinator/src/artifacts.rs` — path traversal protection
- `binaries/coordinator/src/prometheus_metrics.rs` — metrics registration

### Libraries
- `libraries/shared-memory-server/src/lib.rs` — shmem server/client
- `libraries/shared-memory-server/src/channel.rs` — zero-copy IPC channel, unsafe Send/Sync
- `libraries/message/src/lib.rs` — protocol message types
- `libraries/message/src/auth.rs` — token generation, file I/O, discovery
- `libraries/message/src/id.rs` — NodeId/DataId validation
- `libraries/message/src/metadata.rs` — metadata structure, parameters
- `libraries/message/src/config.rs` — node configuration, InputMapping
- `libraries/core/src/descriptor/` — YAML parsing, validation, expansion (1768 lines)
- `libraries/extensions/download/src/lib.rs` — URL download, Content-Disposition
- `libraries/recording/src/lib.rs` — .adorec format

### APIs
- `apis/rust/node/src/node/mod.rs` — Rust node API (1621 lines)
- `apis/rust/node/src/event_stream/mod.rs` — event stream, scheduler (764 lines)
- `apis/rust/node/src/event_stream/data_conversion.rs` — Arrow data conversion
- `apis/rust/operator/src/lib.rs` — Rust operator API (69 lines)
- `apis/rust/operator/src/raw.rs` — FFI operator bridge
- `apis/python/node/src/lib.rs` — Python node API (682 lines)

### CLI & Runtime
- `binaries/cli/src/lib.rs` — command structure, argument parsing
- `binaries/cli/src/common.rs` — CLI helper functions
- `binaries/cli/src/ws_client.rs` — WebSocket client (637 lines)
- `binaries/cli/src/command/run.rs` — `adora run` command
- `binaries/cli/src/template/` — scaffolding templates (Rust, Python, C, C++)
- `binaries/runtime/src/lib.rs` — operator runtime
- `binaries/runtime/src/operator/shared_lib.rs` — dynamic library loading

### Tests & Examples
- `tests/example-smoke.rs` — 27 smoke tests (not in CI)
- `tests/ws-cli-e2e.rs` — WebSocket E2E tests (#[ignore]d)
- `tests/fault-tolerance-e2e.rs` — fault tolerance tests (not in CI)
- `examples/benchmark/` — latency/throughput benchmarks
- `examples/service-example/` — request/reply pattern
- `examples/action-example/` — goal/feedback/result pattern
