# Adora v0.1.1+ Sprint Plan

**Based on**: audit-report-2026-03-21.md (combined audit)
**Date**: 2026-03-23

---

## v0.1.1 — Safety & Correctness (Sprint 1-2)

### Sprint 1: Memory Safety + Templates (quick wins)

| # | ID | Task | Files | Effort |
|---|-----|------|-------|--------|
| 1 | DC1 | Bounds-check shmem `len` before `[..len]` indexing | `daemon/src/lib.rs:3207` | XS |
| 2 | CM2 | Remove `unsafe impl Sync for ShmemChannel` | `shared-memory-server/src/channel.rs:259-260` | XS |
| 3 | DC8 | Fix Python operator count `> 2` -> `> 1` | `daemon/src/spawn/spawner.rs:211` | XS |
| 4 | AC7 | Fix C++ template null ptr UB | `cli/src/template/cxx/node-template.cc:27` | XS |
| 5 | AC15/AC16 | Fix Rust template unused variable warnings | `cli/src/template/rust/node/main-template.rs`, `operator/lib-template.rs` | XS |
| 6 | AC17/D5 | Fix Python template `"TICK"` -> `"tick"` | `cli/src/template/python/__node-name__/__node_name__/main.py` | XS |
| 7 | ET2 | Gitignore committed artifacts; remove from repo | `examples/*/out/.gitignore`, `tests/dataflows/out/` | S |
| 8 | SEC2 | Restrict URL schemes to `https://` (+ `http://`) | `libraries/core/src/descriptor/mod.rs:322` | XS |
| 9 | AC5/AC6 | Fix `drain()`/`try_recv()` for merged streams | `apis/python/node/src/lib.rs:233,579` | S |

### Sprint 2: Panics + Security + Performance Foundations

| # | ID | Task | Files | Effort |
|---|-----|------|-------|--------|
| 10 | Q1 | `NodeId::from(String)` -> `TryFrom<String>` | `libraries/message/src/id.rs:61-68` + call sites | M |
| 11 | Q2 | ROS2 bridge `.unwrap()` -> `.context()?` | `binaries/ros2-bridge-node/src/main.rs:156,385` | XS |
| 12 | Q3 | Tracing `.unwrap()` in `tokio::spawn` -> log + fallback | `apis/rust/node/src/node/mod.rs:1304,1306,1314` | S |
| 13 | Q4 | Python `drain()` error -> log instead of silent empty dict | `apis/python/node/src/lib.rs:232-234` | XS |
| 14 | DC7 | `AllNodesReady` race: `bail!` -> warn + queue | `daemon/src/pending.rs:131-133` | S |
| 15 | S1 | Log warning when `adora run` uses no auth | `cli/src/command/run.rs` | XS |
| 16 | P1 | Wrap `DataMessage` in `Arc` for fan-out | `daemon/src/lib.rs:3119`, `message/src/` NodeEvent | M |
| 17 | P2 | Wrap `Metadata` in `Arc` for fan-out | Same as P1 | S |
| 18 | S2/P13 | Bounded per-node event channels | `daemon/src/running_dataflow.rs:159` | M |
| 19 | DEP1 | Arrow `default-features = false` | `Cargo.toml` workspace + per-crate features | S |
| 20 | CVE | Upgrade Zenoh (resolves lz4_flex, quinn-proto) | `Cargo.toml`, `Cargo.lock` | S |

---

## v0.1.x — Reliability (Sprint 3)

| # | ID | Task | Files | Effort |
|---|-----|------|-------|--------|
| 21 | DC2 | Cancellation token for listener loops | `daemon/src/spawn/prepared.rs:567-585`, `node_communication/shmem.rs` | M |
| 22 | DC4 | `try_send` for log-related daemon_tx sends | `daemon/src/spawn/prepared.rs:666,748` | S |
| 23 | DC5 | Remove per-line `sync_all()` from log writer | `daemon/src/spawn/prepared.rs:844` | XS |
| 24 | DC6 | Recursive child process aggregation | `daemon/src/lib.rs:1317-1325` | S |
| 25 | CM3 | Signal client event on server drop | `shared-memory-server/src/channel.rs:262-286` | S |
| 26 | CM7 | Preserve `uv` flag in auto-recovery | `coordinator/src/handlers.rs:601-608` | XS |
| 27 | CM9 | Clean `running_dataflows` on heartbeat timeout | `coordinator/src/lib.rs:1241-1249` | S |
| 28 | CM11/12 | TTL eviction for `finished_builds` + `archived_dataflows` | `coordinator/src/lib.rs:246,249` | S |
| 29 | ET1 | Add CI E2E job (smoke + ws-cli-e2e + fault-tolerance) | `.github/workflows/ci.yml` | M |
| 30 | ET3 | Remove `#[ignore]` or add `--ignored` CI step | `tests/ws-cli-e2e.rs:431,477` | XS |
| 31 | ET4 | Check for "Failed" in smoke test polling | `tests/example-smoke.rs:195` | XS |
| 32 | SEC1 | sha256 digest for URL-sourced downloads | `libraries/extensions/download/src/lib.rs`, descriptor YAML schema | M |

---

## v0.2 — Performance & DX (Sprint 4-5)

### Sprint 4: Performance

| # | ID | Task | Files | Effort |
|---|-----|------|-------|--------|
| 33 | P4 | Split shmem protocol (bincode metadata + raw bytes) | `shared-memory-server/src/channel.rs`, `daemon/src/lib.rs` | L |
| 34 | P3 | Only copy shmem data when remote receivers exist | `daemon/src/lib.rs:3200-3215` | M |
| 35 | P8 | Separate metadata ser from Zenoh payload | `daemon/src/lib.rs:2342-2347` | M |
| 36 | A2 | Remove per-timer HLC; use shared daemon clock | `daemon/src/running_dataflow.rs:256` | XS |
| 37 | P10 | Pre-allocate timer metadata template | `daemon/src/running_dataflow.rs:263-264` | S |
| 38 | P12 | `ProcessesToUpdate::Some(&pids)` for sysinfo | `daemon/src/lib.rs:1293` | S |
| 39 | P5 | TCP vectored write or `BufWriter` | `daemon/src/socket_stream_utils.rs:17-20` | S |
| 40 | Q5 | Split daemon `lib.rs` into modules | `daemon/src/lib.rs` (4081 lines) | L |

### Sprint 5: DX + Dependencies

| # | ID | Task | Files | Effort |
|---|-----|------|-------|--------|
| 41 | D1 | Create `docs/quickstart.md` | `docs/quickstart.md` (new) | M |
| 42 | D2 | Python service/streaming helpers | `apis/python/node/src/lib.rs` | M |
| 43 | D3 | Add metadata to Operator API `Event::Input` | `apis/rust/operator/src/lib.rs` | M |
| 44 | D4 | Align Operator API types (`DataId`, `&str` send) | Same as D3 | S |
| 45 | — | Add streaming pattern example | `examples/streaming-example/` (new) | M |
| 46 | DEP4 | Zenoh `default-features = false` | `Cargo.toml` | S |
| 47 | DEP3 | Tokio explicit feature list | Multiple `Cargo.toml` | S |
| 48 | DEP6 | Replace `once_cell` -> `std::sync::LazyLock` | `libraries/message/src/config.rs` | XS |
| 49 | DEP8 | Move `inquire` behind feature flag | `apis/rust/node/Cargo.toml` | S |
| 50 | DEP10 | Align `which` to v7 | Multiple `Cargo.toml` | XS |
| 51 | — | Centralize deps in `[workspace.dependencies]` | `Cargo.toml` + per-crate | M |
| 52 | — | Add `[profile.release]` with `lto = "thin"` | `Cargo.toml` | XS |

---

## Dependency Graph

```
Sprint 1 (no deps)
  |
Sprint 2
  ├── P1/P2 (Arc wrapping) --> Sprint 4 P4 (shmem protocol split)
  ├── S2 (bounded channels) --> Sprint 3 DC2 (listener cancellation)
  └── Q1 (NodeId TryFrom) --> standalone, many call sites
       |
Sprint 3
  ├── ET1 (CI E2E) --> validates everything after
  └── DC2 (cancellation) --> Sprint 4 Q5 (daemon split)
       |
Sprint 4
  ├── P4 (shmem protocol) --> most impactful perf change
  └── Q5 (daemon split) --> enables future maintainability
       |
Sprint 5 (no deps, parallel with Sprint 4)
```

## Open Questions

1. **Q1 (NodeId)**: Keep `From<String>` with debug-assert, or remove entirely?
2. **S2 (bounded channels)**: Default capacity? Configurable per-node in YAML? Drop-oldest or backpressure?
3. **P1 (Arc)**: `Arc<DataMessage>` + `Arc<Metadata>` separately, or single `Arc<NodeEventPayload>`?
4. **Zenoh upgrade**: Pinned to 1.7.x or jump to latest?
5. **D3 (Operator metadata)**: Breaking change — ship in v0.2 as documented?
