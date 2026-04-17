# Dora Framework Audit Report

**Date**: 2026-03-13
**Scope**: Architecture, performance, security, DX, AI agent readiness
**Perspective**: Robotics and real-time AI agent use cases

---

## Executive Summary

Dora is a well-engineered, production-ready Rust robotics framework with genuine
strengths in latency, zero-copy IPC, and AI agent communication patterns. At 0.1.0
it is at a strong foundation stage. This audit identified critical bugs (now fixed),
architectural constraints worth documenting, and high-impact improvements for the
robotics + AI agent audience.

### What Was Fixed (This Audit)

| # | Fix | Location | Type |
|---|-----|----------|------|
| F1 | Timer burst mode -> Skip | `running_dataflow.rs:254` | Must-fix |
| F2 | `assert!` in shmem IPC -> `eyre::bail!` | `shared-memory-server/src/lib.rs:38,51` | Must-fix |
| F3 | `std::env::set_var` UB removed | `node/mod.rs:585-608` | Must-fix |
| F4 | Python `todo!()` panics removed | `apis/python/node/src/lib.rs:579,616,626` | Must-fix |
| F5 | `topic echo` debug-mode hint | `cli/src/command/topic/echo.rs` | Should-fix |
| F6 | Zenoh channel bound 10 -> 100 | `daemon/src/lib.rs:238` | Should-fix |
| F7 | `sysinfo` moved to `spawn_blocking` | `daemon/src/lib.rs:1280-1298` | Should-fix |
| F8 | `NodeId` documented + `FromStr` highlighted | `message/src/id.rs` | Should-fix |

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

### Known Constraints (Document)

| Constraint | Impact | Mitigation |
|-----------|--------|------------|
| **Single coordinator SPOF** | Coordinator crash kills all daemons after 20s | CoordinatorStore supports crash recovery; no standby/leader election yet |
| **Static topology** | Cannot add/remove nodes mid-execution | Use parameter-based node enablement or restart dataflow |
| **Single-threaded event loop** | Control-plane latency spikes affect data routing | Event timing warning fires at >100ms; separate planes in future |
| **Arrow metadata-only** | No IPC framing or schema negotiation | Nodes agree out-of-band on byte layout; type info in metadata |
| **Soft real-time only** | No SCHED_FIFO, CPU pinning, or mlockall | Acceptable for most robotics; document OS-level tuning |

### Roadmap Items

| Priority | Item | Effort | Impact |
|----------|------|--------|--------|
| High | Wrap `Metadata` in `Arc` for fan-out | S | Eliminates N BTreeMap clones per message at fan-out |
| High | Pass shmem ID to local receivers directly | M | True zero-copy for co-located nodes |
| High | Separate control/data planes in event loop | L | Reduce latency jitter from control operations |
| Medium | Remove duplicate HLC per timer | XS | `running_dataflow.rs:255` — use daemon clock only |
| Medium | Send only assigned nodes per daemon | S | `run/mod.rs:54-58` — avoid full descriptor fan-out |
| Medium | TCP vectored write | S | `socket_stream_utils.rs:17-21` — reduce syscalls |
| Medium | Arrow IPC framing | L | Enable true zero-serialization; validate types at edges |
| Medium | Type validation at dataflow startup | M | Catch output/input type mismatches in `check()` |
| Low | Coordinator standby / leader election | XL | Eliminate SPOF for distributed deployments |
| Low | Dynamic topology (mid-execution node add/remove) | XL | Required for fully dynamic AI agent systems |

---

## 2. Performance

### Confirmed

- Zero-copy for >= 4KB messages: flat ~500us latency from 4KB to 4MB
- Shmem cache (20 regions / 256MB) reduces allocation frequency
- Per-input queue policies: DropOldest (default, size=10) and Backpressure
- LRU fairness scheduler prevents high-frequency input starvation
- Criterion benchmarks for daemon routing, shmem round-trip, message serde

### Unsubstantiated

- **10-17x ROS2 claim**: credible but no ROS2 comparison benchmarks in repo
- **Metadata cloning overhead**: not quantified; could dominate high-fan-out scenarios

### Roadmap Items

| Priority | Item | Effort | Impact |
|----------|------|--------|--------|
| High | Add ROS2 side-by-side benchmark | M | Substantiate performance claims |
| High | Quantify metadata clone overhead | S | Profile and optimize if significant |
| Medium | Optimize small message path (<4KB) | M | Avoid bincode + clone per subscriber |
| Medium | Configurable tokio runtime (thread count, pinning) | S | Reduce jitter for latency-sensitive deployments |
| Low | `pending_drop_tokens` O(N) scan -> VecDeque | XS | Reduce allocations at high message rates |

---

## 3. Security & Robustness

### Passed Checks

- Constant-time token comparison (`auth.rs:45`)
- Token files created `0600` with TOCTOU-resistant fstat ownership check
- Message size caps enforced at all boundaries
- Path traversal prevention with test coverage
- All `unsafe` blocks have `# Safety` doc comments
- `umask(0o077)` before shmem/credential file creation
- Watchdog, health-check kills, circuit-breaker recovery present

### Remaining Issues

| Priority | Issue | Location | Recommendation |
|----------|-------|----------|----------------|
| High | Auth off by default | `coordinator/src/lib.rs:69` | Make auth opt-out, not opt-in |
| High | Unbounded per-node event channels | `running_dataflow.rs:159` | Replace with bounded + drop-oldest |
| High | Shell-node args injection | `spawn/command.rs:124` | Log warning when shell nodes enabled |
| Medium | Rate limiter loopback exemption | `ws_server.rs:50` | Separate rate limit for daemon endpoint |
| Medium | Stale auth token after crash | `auth.rs:83` | Add token expiry or rotation |
| Medium | Node stderr not sanitized | `state.rs:234` | Strip ANSI/control chars before logging |
| Low | `bincode 1.x` DoS surface | All shmem IPC | Upgrade to bincode 2.x |
| Low | 20s heartbeat timeout hardcoded | `lib.rs:607` | Make configurable |

---

## 4. Developer Experience

### Strengths

- Rich CLI: `dora run`, `doctor`, `top`, `topic echo/hz/pub`, `param`, record/replay
- Clean Python API with sync/async support
- YAML dataflow spec with comprehensive documentation (855-line architecture doc)
- Service/Action/Streaming patterns with zero boilerplate
- Hot reload for Python operators

### Remaining Issues

| Priority | Issue | Recommendation |
|----------|-------|----------------|
| High | Rust node scaffold inconsistent with examples | Add tokio runtime + tracing to template |
| High | Python bytes API produces unreadable output | Document send/receive pair for bytes case |
| High | Python metadata convention inconsistent | Unify `metadata=` dict format |
| Medium | `dora new --kind node` produces no YAML | Add minimal `dataflow-example.yml` |
| Medium | `_unstable_*` naming with no stability legend | Add stability docs + graduation criteria |
| Medium | No canonical robotics example | Add sensor -> filter -> pose pipeline example |
| Medium | No getting-started tutorial | Add `docs/quickstart.md` with expected output |
| Medium | Operator API uses `String` not `DataId` | Align with Node API (breaking, do at 0.1.x) |
| Low | `Reload` event variant exposed in public API | Add `#[doc(hidden)]` |
| Low | `ParamUpdate` missing from api-rust.md | Update docs |
| Low | `dora doctor` doesn't check Python version | Wire existing `get_python_dora_version()` |

---

## 5. AI Agent Readiness

### Excellent

| Pattern | Status | Details |
|---------|--------|---------|
| Service (request/reply) | Production-ready | Auto-correlation via UUIDv7 `request_id` |
| Action (goal/feedback/result) | Production-ready | 64 concurrent goals, cancellation support |
| Streaming (token generation) | Production-ready | Session/segment/chunk with `flush` interrupt |
| Python ML integration | Production-ready | NumPy, PyTorch, Transformers, YOLOv8, Whisper, CUDA IPC |
| Recording/replay | Production-ready | `.adorec` format, speed control, loop mode |
| Parameter server | Production-ready | Coordinator K/V store, optional persistence |
| Distributed deployment | Production-ready | Multi-daemon via Zenoh, machine labels |

### Gaps

| Gap | Severity | Workaround |
|-----|----------|------------|
| No mid-execution topology changes | Medium | Restart dataflow; use parameter-based enablement |
| No declarative agent DSL | Low | Write agent loop in Python/Rust (flexible) |
| 64KB parameter value limit | Low | Fine for config; use external DB for large state |
| Parameter server centralized | Low | Coordinator is single access point |
| No built-in LLM framework | Low | Bring your own (Transformers, etc.) |

### Recommended AI Agent Architecture

```
                    ┌─────────────────┐
                    │   Coordinator   │
                    │  (param store)  │
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
         ┌────┴────┐   ┌────┴────┐   ┌────┴────┐
         │ Daemon A │   │ Daemon B │   │ Daemon C│
         └────┬────┘   └────┬────┘   └────┬────┘
              │              │              │
    ┌─────────┼──────┐   ┌──┴──┐    ┌──────┼──────┐
    │         │      │   │     │    │      │      │
 Sensor    LLM    Plan  Arm   Nav  Camera  YOLO  TTS
  Node    Agent   ner  Ctrl  Ctrl   Node   Node  Node
```

- **Planner -> Executors**: Action pattern (goal + feedback)
- **LLM Agent -> Tools**: Service pattern (request/reply)
- **Camera -> Detector -> Planner**: Topic dataflow (pub/sub)
- **LLM -> TTS**: Streaming pattern (token-by-token with interrupt)
- **All agents -> Param server**: Shared state coordination

---

## 6. Prioritized Roadmap

### Phase 1: Stability & Correctness (v0.1.x)

- [ ] Enable auth by default in `dora run`
- [ ] Bounded per-node event channels with drop-oldest
- [ ] `Arc<Metadata>` in fan-out loop
- [ ] Fix Rust node scaffold template
- [ ] Unify Python metadata convention
- [ ] Document bytes API send/receive pair
- [ ] Add stability legend for `_unstable_*`

### Phase 2: Performance & DX (v0.2)

- [ ] True zero-copy for local receivers (pass shmem ID)
- [ ] Separate control/data planes in daemon event loop
- [ ] ROS2 side-by-side benchmark
- [ ] Getting-started tutorial (`docs/quickstart.md`)
- [ ] Canonical robotics example (sensor fusion pipeline)
- [ ] Align Operator API with Node API (`DataId`, proper error types)
- [ ] Arrow type validation at dataflow startup
- [ ] `#[dora::node]` proc macro for Rust boilerplate

### Phase 3: Scale & Distribution (v0.3+)

- [ ] Coordinator standby / leader election
- [ ] Dynamic topology (mid-execution node add/remove)
- [ ] Configurable heartbeat timeouts
- [ ] Token expiry and rotation
- [ ] Distributed parameter server
- [ ] bincode 2.x migration

---

## Appendix: Files Referenced

### Core Architecture
- `binaries/daemon/src/lib.rs` — main daemon event loop, routing, metrics
- `binaries/daemon/src/running_dataflow.rs` — timer setup, subscriber channels
- `binaries/daemon/src/node_communication/mod.rs` — shmem region sizing
- `binaries/coordinator/src/state.rs` — coordinator-daemon connection
- `libraries/shared-memory-server/src/` — zero-copy IPC primitives
- `libraries/message/src/` — protocol types, metadata, auth

### APIs
- `apis/rust/node/src/node/mod.rs` — Rust node API
- `apis/rust/operator/src/lib.rs` — Rust operator API
- `apis/python/node/src/lib.rs` — Python node API (PyO3)

### CLI
- `binaries/cli/src/command/topic/echo.rs` — topic echo command
- `binaries/cli/src/template/` — scaffolding templates

### Examples
- `examples/benchmark/` — latency/throughput benchmarks
- `examples/python-operator-dataflow/llm_op.py` — LLM integration
- `examples/service-example/` — request/reply pattern
- `examples/action-example/` — goal/feedback/result pattern
