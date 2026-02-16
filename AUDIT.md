# Adora-rs Deep Audit Report

**Date**: 2026-02-16
**Audited by**: Claude Code (multi-agent: architect, security-reviewer, code-reviewer)

## Overall Grade: B / B+

Solid distributed robotics framework with excellent core design decisions, but significant gaps in security, testing, and accumulated technical debt from rapid growth.

---

## What's Done Well

- **Zero-copy architecture**: Shared memory for >4KB messages + Apache Arrow columnar format is a genuinely excellent design
- **Clean layering**: CLI -> Coordinator -> Daemon -> Nodes/Operators is well-separated
- **UHLC (Hybrid Logical Clock)**: Proper distributed causality tracking -- shows real systems thinking
- **Multi-language support**: Rust/Python/C/C++ APIs with clean FFI boundaries
- **Structured concurrency**: Proper tokio + bounded flume channels, no unbounded growth
- **Restart policies, telemetry, version checking**: Production-readiness signals are there
- **Workspace management**: Excellent use of workspace-level versioning and metadata

---

## Critical Issues (P0 -- Fix Before Production)

### 1. Shell Command Injection

**`binaries/daemon/src/spawn/command.rs:24-28`**

```rust
let cmd = Command::new("sh");
cmd.args(["-c", &node.args.clone().unwrap_or_default()])
```

User-controlled YAML `args` are passed directly to shell. Any malicious YAML can execute arbitrary commands. This is a **remote code execution vector**.

### 2. No Authentication on Any Network Connection

CLI<->Coordinator, Coordinator<->Daemon, Daemon<->Node -- all TCP connections have **zero authentication**. Anyone on the network can register as a daemon, inject nodes, or stop dataflows.

### 3. Unbounded TCP Message Allocation (DoS)

**`binaries/daemon/src/socket_stream_utils.rs:14-24`**

```rust
let reply_len = u64::from_le_bytes(raw) as usize;  // Can be 16 EiB
let mut reply = vec![0; reply_len];  // OOM crash
```

Attacker sends a huge `reply_len` value -> instant memory exhaustion.

### 4. Near-Zero Test Coverage on Critical Binaries

- Daemon: **0 unit tests** (most complex component, 1300+ lines)
- Coordinator: **0 unit tests**
- CLI: **0 unit tests**
- Only ~95 `#[test]` annotations across 482 files
- No end-to-end integration tests for the distributed system

For a real-time robotics framework, this is dangerous.

---

## High Severity Issues (P1)

### 5. Daemon is a God Object

`binaries/daemon/src/lib.rs` is 1300+ lines with 14+ fields, 55+ functions, 700+ line match statements. Mixes process management, communication, logging, metrics, git, and builds. Needs decomposition into `Orchestrator`, `CommunicationManager`, `ProcessManager`.

### 6. 247 `.unwrap()` Calls in Non-Test Code

Including in library code (`libraries/message/src/lib.rs:74`). Any of these is a potential panic in production -- unacceptable for a real-time system.

### 7. Excessive `.clone()` in Hot Paths (~291 in binaries/)

`NodeId`, `DataId`, `DaemonId` are all `String` wrappers being cloned constantly in event loops. Should use `Arc<str>` or string interning.

### 8. Forked Dependency -- Supply Chain Risk

```toml
# TODO use upstream release once PR is merged
raw_sync_2 = "0.1.5"  # personal fork
```

Critical IPC dependency pinned to an unvetted personal crate.

### 9. Shared Memory Uses `assert!` Instead of Error Handling

**`libraries/shared-memory-server/src/channel.rs:94-120`**

```rust
assert!(msg.len() <= self.memory.len() - self.data_offset);  // Panics!
unsafe { self.data_mut().copy_from_nonoverlapping(...) }
```

Panic in shared memory path crashes the daemon. The following unsafe block can corrupt memory.

### 10. No Documentation on Core Library

`adora-core` has **0 doc comments** on public APIs. `DescriptorExt` trait, build utilities, Zenoh config -- all undocumented.

### 11. `todo!()` in Critical Control Flow

**`binaries/runtime/src/lib.rs:177`**

```rust
todo!("instruct adora-daemon/adora-coordinator to stop other nodes");
```

This is in the `StopAll` operator path -- will panic at runtime.

---

## Medium Severity Issues (P2)

| # | Issue | Location |
|---|-------|----------|
| 12 | Coordinator stores all state in memory -- restart loses everything | `coordinator/src/lib.rs` |
| 13 | `start_inner()` is a 1500+ line function | `coordinator/src/lib.rs:189` |
| 14 | World-readable shared memory regions (no permission restriction) | `daemon/src/node_communication/mod.rs` |
| 15 | URL downloads without checksum verification | `daemon/src/spawn/command.rs:31-39` |
| 16 | No backpressure mechanism documented | Bounded channels but no policy |
| 17 | Hardcoded magic numbers (`100ms`, `500 lines`, port `53291`) | Multiple files |
| 18 | Inconsistent error types (`eyre::Result` vs `Result<T, String>`) | Coordinator and daemon mix patterns |
| 19 | 31 TODO/FIXME comments, 5 `#[allow(dead_code)]` suppressions | Across codebase |
| 20 | `#[allow(clippy::too_many_arguments)]` -- lint suppression over fix | `daemon/src/lib.rs:352` |
| 21 | 177 `unsafe` blocks with `#![allow(clippy::missing_safety_doc)]` | Shared memory + message crates |

---

## Prioritized Remediation Plan

### Week 1-2 (Security)

1. Add max message size constant to TCP receive (trivial fix, huge impact)
2. Sanitize or remove `SHELL_SOURCE` node type
3. Add token-based auth for daemon registration
4. Replace `assert!` with `Result` in shared memory paths
5. Fix the `todo!()` in runtime StopAll

### Month 1 (Stability)

6. Add unit tests to daemon, coordinator, CLI -- target 50% coverage
7. Split daemon god object into modules
8. Replace `unwrap()` with proper error propagation in library code
9. Upstream `raw_sync_2` or migrate to maintained alternative
10. Document `adora-core` public APIs

### Month 2-3 (Scale)

11. Add coordinator state persistence (sqlite/sled)
12. Reduce `.clone()` overhead with `Arc<str>` for IDs
13. Integration test suite for distributed scenarios
14. Add end-to-end benchmarks (only 1 benchmark file found currently)

---

## Honest Bottom Line

**The architecture is genuinely good.** Zero-copy shared memory, Arrow format, distributed coordination with hybrid logical clocks -- these are the right choices. The crate boundaries are sensible and the multi-language story is compelling.

**But the engineering rigor hasn't kept pace with the ambition.** A distributed real-time robotics framework with zero tests on its daemon, unauthenticated network connections, and shell injection vulnerabilities is not production-ready. The 247 unwraps and 291 clones in hot paths suggest "make it work" without "make it right."

**The good news**: the codebase is well-structured for refactoring. The problems are fixable without rearchitecting. A focused 2-3 month effort on security, testing, and daemon decomposition would elevate this from B to A-grade.
