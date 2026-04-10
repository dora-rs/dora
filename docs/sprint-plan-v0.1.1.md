# Dora v0.1.1+ Sprint Plan

**Based on**: audit-report-2026-03-21.md (combined audit)
**Created**: 2026-03-23
**Updated**: 2026-03-24 (v0.1.1 sprints complete; see roadmap-v0.2.md for sprints 6-12)

---

## Summary

| Sprint | Done | Deferred | Total |
|--------|------|----------|-------|
| Sprint 1 | 8 | 1 | 9 |
| Sprint 2 | 11 | 0 | 11 |
| Sprint 3 | 11 | 1 | 12 |
| Sprint 4 | 6 | 2 | 8 |
| Sprint 5 | 6 | 6 | 12 |
| Review fixes | 16 | — | 16 |
| **Total** | **58** | **10** | **68** |

---

## v0.1.1 — Safety & Correctness (Sprint 1-2)

### Sprint 1: Memory Safety + Templates (quick wins)

| # | ID | Task | Status | Commit |
|---|-----|------|--------|--------|
| 1 | DC1 | Bounds-check shmem `len` before `[..len]` indexing | DONE | `9711789` |
| 2 | CM2 | Document `unsafe impl Sync for ShmemChannel` safety invariants | DONE | `9711789` (kept Sync — needed by EventStream; added docs) |
| 3 | DC8 | Fix Python operator count `> 2` -> `> 1` | DONE | `9711789` |
| 4 | AC7 | Fix C++ template null ptr UB | SKIP | False positive — CXX `rust::String` is always valid |
| 5 | AC15/AC16 | Fix Rust template unused variable warnings | DONE | `9711789` |
| 6 | AC17/D5 | Fix Python template `"TICK"` -> `"tick"` | DONE | `9711789` |
| 7 | ET2 | Gitignore committed artifacts; remove from repo | DONE | `9711789` (25 .gitignore files) |
| 8 | SEC2 | Restrict URL schemes to `https://` (+ `http://`) | DONE | `9711789` |
| 9 | AC5/AC6 | Fix `drain()`/`try_recv()` for merged streams | DONE | `9711789` |

### Sprint 2: Panics + Security + Performance Foundations

| # | ID | Task | Status | Commit |
|---|-----|------|--------|--------|
| 10 | Q1 | NodeId CLI validation via `.parse()` instead of `.into()` | DONE | `58df7e2` (8 CLI call sites + topic/pub) |
| 11 | Q2 | ROS2 bridge `.unwrap()` -> `.context()?` | DONE | `6f2e032` |
| 12 | Q3 | Tracing `.unwrap()` in `tokio::spawn` -> log + fallback | DONE | `6f2e032` |
| 13 | Q4 | Python `drain()` error -> log instead of silent empty dict | DONE | `6f2e032` |
| 14 | DC7 | `AllNodesReady` race: `bail!` -> warn + proceed | DONE | `6f2e032` |
| 15 | S1 | Log warning when `dora run` uses no auth | DONE | `6f2e032` |
| 16 | P1 | Wrap `DataMessage` in `Arc` for fan-out | DONE | `425fde8` |
| 17 | P2 | Wrap `Metadata` in `Arc` for fan-out | DONE | `425fde8` |
| 18 | S2/P13 | Bounded per-node event channels (capacity 1000) | DONE | `04ac605` |
| 19 | DEP1 | Arrow `default-features = false` | DONE | `6f9ac14` (15 -> 12 sub-crates) |
| 20 | CVE | Upgrade Zenoh 1.1.1 -> 1.8.0 | DONE | `3d1293c` (lz4_flex/quinn-proto still transitive — Zenoh upstream) |

### Review Fixes (post-Sprint 2)

| # | ID | Task | Status | Commit |
|---|-----|------|--------|--------|
| R1 | — | Reserve channel headroom for control events (Stop, InputClosed) | DONE | `63b589b` |
| R2 | — | Switch finished_builds/archived_dataflows to IndexMap (FIFO eviction) | DONE | `da58d8e` |
| R3 | — | send_with_timestamp returns Ok(true)/Ok(false) for pending counter | DONE | `da58d8e` |
| R4 | — | Fix topic/pub_.rs `.into()` -> `.parse()` (missed in Q1) | DONE | `63b589b` |
| R5 | — | Add `# Panics` doc to `From<&str> for DataId` | DONE | `da58d8e` |
| R6 | — | Log file flush note on rotation | DONE | `da58d8e` |
| R7 | — | IndexMap dep added to coordinator | DONE | `da58d8e` |
| R8 | — | Control event `.is_ok()` -> `.ok() == Some(true)` (9 call sites) | DONE | `3df9f59` |
| R9 | — | Drop channels reverted to unbounded (shmem leak prevention) | DONE | `3df9f59` |
| R10 | — | `send_service_response` warns when `request_id` missing | DONE | `795d96f` |
| R11 | — | Download error message fix: literal `{uri}` -> format `{url}` | DONE | `795d96f` |
| R12 | — | `send_service_request` warns on overwritten `request_id` | DONE | `795d96f` |
| R13 | — | Shmem `spawn_blocking` bounded leak documented | DONE | `795d96f` |
| R14 | — | `send_service_response` docstring: metadata is required | DONE | `795d96f` |
| R15 | — | Zenoh version `1.8.0` -> `~1.8` (allow patch updates) | DONE | `795d96f` |
| R16 | — | Shmem `mut shutdown` comment added | DONE | `795d96f` |

---

## v0.1.x — Reliability (Sprint 3)

| # | ID | Task | Status | Commit |
|---|-----|------|--------|--------|
| 21 | DC2 | Listener loop shutdown via watch channel | DONE | `904131f` (TCP/UDS select!, shmem select!) |
| 22 | DC4 | `try_send` for log-related daemon_tx sends | DONE | `8064b83` |
| 23 | DC5 | Remove per-line `sync_all()` from log writer | DONE | `8064b83` |
| 24 | DC6 | Recursive child process aggregation | DONE | `8064b83` |
| 25 | CM3 | Signal client event on server drop | DEFERRED | Touches shmem channel internals |
| 26 | CM7 | Preserve `uv` flag in auto-recovery | DONE | `8064b83` |
| 27 | CM9 | Clean `running_dataflows` on daemon heartbeat timeout | DONE | `8064b83` |
| 28 | CM11/12 | Cap `finished_builds` (100) + `archived_dataflows` (200) | DONE | `8064b83` + `da58d8e` (IndexMap) |
| 29 | ET1 | Add CI E2E job (ws-cli-e2e + fault-tolerance) | DONE | `8064b83` |
| 30 | ET3 | Remove `#[ignore]` from 2 E2E tests | DONE | `8064b83` |
| 31 | ET4 | Check for "Failed" state in smoke test polling | DONE | `8064b83` |
| 32 | SEC1 | sha256 digest for URL-sourced downloads | DONE | `4e9e12f` (optional expected_sha256 param) |

---

## v0.2 — Performance & DX (Sprint 4-5)

### Sprint 4: Performance

| # | ID | Task | Status | Commit |
|---|-----|------|--------|--------|
| 33 | P4 | Split shmem protocol (bincode metadata + raw bytes) | DEFERRED | L effort — protocol change |
| 34 | P3 | Only copy shmem data when remote receivers exist | DONE | `bf96b1b` |
| 35 | P8 | Separate metadata ser from Zenoh payload | DEFERRED | M effort, coupled with P4 |
| 36 | A2 | Remove per-timer HLC; use shared daemon clock | DONE | `bf96b1b` |
| 37 | P10 | Pre-allocate timer metadata template | DONE | `bf96b1b` |
| 38 | P12 | Pre-build parent->children map for metrics aggregation | DONE | `bf96b1b` (O(P+N*D) vs O(N*P)) |
| 39 | P5 | TCP single-buffer write (was 2-3 syscalls) | DONE | `bf96b1b` |
| 40 | Q5 | Split daemon `lib.rs` into modules | DEFERRED | L effort — large refactor |

### Sprint 5: DX + Dependencies

| # | ID | Task | Status | Commit |
|---|-----|------|--------|--------|
| 41 | D1 | Create `docs/quickstart.md` | DONE | `934f1c4` |
| 42 | D2 | Python service/streaming helpers | DONE | `4e9e12f` (send_service_request, send_service_response) |
| 43 | D3 | Add metadata to Operator API `Event::Input` | DONE | `4e9e12f` (metadata: &types::Metadata field) |
| 44 | D4 | Align Operator API types (`DataId`, `&str` send) | DONE | `4e9e12f` (send takes &str, not String) |
| 45 | — | Add streaming pattern example | DEFERRED | M effort |
| 46 | DEP4 | Zenoh `default-features = false` | DEFERRED | Need to identify required transports |
| 47 | DEP3 | Tokio explicit feature list | DEFERRED | Too invasive |
| 48 | DEP6 | Replace `once_cell` -> `std::sync::OnceLock` | DONE | `4b6e15c` |
| 49 | DEP8 | Move `inquire` behind feature flag | REVERTED | `3df9f59` — module too deeply integrated for `#[cfg]` |
| 50 | DEP10 | Align `which` to v7 across all crates | DONE | `4b6e15c` |
| 51 | — | Centralize deps in `[workspace.dependencies]` | DEFERRED | M effort |
| 52 | — | Add `[profile.release]` with `lto = "thin"` | DONE | `4b6e15c` |

---

## Deferred Items — Updated v0.2.0

Most deferred items were addressed in v0.2.0 Phases 1-3. Remaining:

| # | ID | Task | Status |
|---|-----|------|--------|
| 25 | CM3 | Signal shmem client on server drop | **Eliminated** — Zenoh SHM replaced custom shmem |
| 33 | P4 | Split shmem protocol | **Eliminated** — Zenoh SHM replaced custom shmem |
| 35 | P8 | Separate Zenoh metadata from payload | **Eliminated** — zenoh attachments handle this |
| 45 | — | Streaming pattern example | **DONE** (Sprint 12) |
| 46 | DEP4 | Zenoh default-features = false | **DONE** (Phase 1) |
| 49 | DEP8 | Move `inquire` behind feature flag | REVERTED — too invasive |
| 51 | — | Centralize workspace deps | **DONE** (Phase 1) |
| 40 | Q5 | Split daemon lib.rs | **Deferred to v0.3** |
| 47 | DEP3 | Tokio explicit feature list | **DONE** (Phase 1) |

---

## Resolved Open Questions

1. **Q1 (NodeId)**: Kept `From<String>` for internal use (known-valid strings). CLI call sites migrated to `.parse()`. `From<String>` panics are documented.
2. **S2 (bounded channels)**: Capacity 1000, drop-newest for data events, 50-slot headroom reserved for control events. `send_with_timestamp` returns `Ok(true/false)`.
3. **P1 (Arc)**: `Arc<DataMessage>` + `Arc<Metadata>` separately in `NodeEvent::Input`. Consumers use `Arc::unwrap_or_clone` (zero-cost when ref count is 1).

4. **Zenoh upgrade**: Upgraded to 1.8.0 (no breaking changes). CVEs for lz4_flex/quinn-proto are Zenoh transitive — tracked upstream.
5. **D3 (Operator metadata)**: Shipped. Breaking change: `Event::Input` now has `metadata` field; `send()` takes `&str`.

## Remaining Open Questions

6. **lz4_flex / quinn-proto CVEs**: Transitive through zenoh-transport. Monitor Zenoh releases for bump.
7. **DEP8 (inquire)**: Module gating requires `#[cfg(feature)]` on `DaemonChannel` enum variants. Needs broader refactor.
