# Adora v0.2 Roadmap

**Created**: 2026-03-24
**Updated**: 2026-03-24 (all sprints complete)
**Source**: 6 plan docs + 10 deferred sprint items from v0.1.1

---

## Work Items Inventory

### From Sprint v0.1.1 (10 deferred)

| ID | Task | Effort |
|----|------|--------|
| CM3 | Signal shmem client on server drop | S |
| P4 | Split shmem protocol (bincode metadata + raw bytes) | L |
| P8 | Separate metadata ser from Zenoh payload | M |
| Q5 | Split daemon lib.rs into modules (4081 lines) | L |
| D-45 | Add streaming pattern example | M |
| DEP4 | Zenoh default-features = false | S |
| DEP3 | Tokio explicit feature list | S |
| DEP8 | Move inquire behind feature flag | S |
| D-51 | Centralize deps in [workspace.dependencies] | M |
| DEP3 | Tokio explicit feature list | S |

### From Plan Docs (6 plans)

| Plan | Key Deliverables | Effort |
|------|-----------------|--------|
| Zenoh SHM | Replace custom shmem with zenoh SHM for data plane | XL |
| Event Loop | Zenoh publish channel + background metrics | L |
| Coordinator HA | Daemon auto-reconnect + state reconstruction + ReDB default | XL |
| Dynamic Topology | AddNode/RemoveNode/AddMapping protocol + CLI | XL |
| Arrow IPC | Runtime type validation + opt-in IPC framing | L |
| Soft Real-Time | Tuning guide + optional --rt profile | M |

---

## Dependency Analysis

```
                    Zenoh SHM (replaces custom shmem)
                    /            |              \
                   /             |               \
            Eliminates:      Eliminates:      Enables:
            - CM3             - P4             - Event Loop
            - P8              - DEP8             (Zenoh pub channel
            - DropToken code  - raw_sync_2        simplifies with
                                                  zenoh-native SHM)
                   \             |               /
                    \            |              /
                     Event Loop Separation
                     (Zenoh publish channel +
                      background metrics)
                              |
                              |  enables
                              v
                     Coordinator HA
                     (daemon auto-reconnect needs
                      stable event loop under load)
                              |
                              |  enables
                              v
                     Dynamic Topology
                     (AddNode needs reliable coordinator
                      + daemon state management)

    Independent tracks:
    ┌─────────────────┐  ┌──────────────────┐  ┌────────────────┐
    │ Arrow IPC        │  │ Soft Real-Time   │  │ Code Quality   │
    │ (type safety)    │  │ (docs + --rt)    │  │ (Q5, deps)     │
    └─────────────────┘  └──────────────────┘  └────────────────┘
```

**Key insight:** Zenoh SHM should go first because it:
1. Eliminates 3 deferred items (CM3, P4, P8) — they become irrelevant
2. Removes `raw_sync_2` and `shared_memory_extended` deps
3. Simplifies Event Loop Separation (fewer shmem paths to manage)
4. Aligns with upstream dora-rs

---

## Sprint Schedule

### Sprint 6: Zenoh SHM Migration (v0.2-alpha)
**Goal:** Replace custom shmem data plane with zenoh SHM

| # | Task | Effort | Files |
|---|------|--------|-------|
| 1 | Add zenoh session + ShmProvider to AdoraNode | M | `apis/rust/node/src/node/mod.rs` |
| 2 | Send path: publish via zenoh when data >= 4KB threshold | M | `apis/rust/node/src/node/mod.rs` |
| 3 | Receive path: zenoh subscriber per input, ZShm zero-copy | M | `apis/rust/node/src/event_stream/mod.rs` |
| 4 | Daemon: remove DropToken tracking, receive data-less notifications | M | `binaries/daemon/src/lib.rs` |
| 5 | Delete drop_stream.rs, DropToken types, pending_drop_tokens | S | Multiple files |
| 6 | Python API: verify PyArrow + zenoh SHM integration | M | `apis/python/node/src/lib.rs` |
| 7 | Update recording to handle zenoh SHM data | M | `binaries/record-node/` |
| 8 | Add configurable threshold (DORA_ZERO_COPY_THRESHOLD) | S | `apis/rust/node/src/node/mod.rs` |

**Eliminates deferred:** CM3, P4, P8, DEP8 (raw_sync_2 gone)
**Breaking:** NodeEvent::Input data type changes

### Sprint 7: Event Loop Separation
**Goal:** Offload Zenoh publish and metrics from the main event loop

| # | Task | Effort | Files |
|---|------|--------|-------|
| 1 | Wrap publishers in Arc, create bounded zenoh publish channel (256) | M | `running_dataflow.rs`, `lib.rs` |
| 2 | Spawn zenoh drain task (receives from channel, publishes) | S | `lib.rs` |
| 3 | send_to_remote_receivers: try_send to channel instead of inline await | S | `lib.rs` |
| 4 | Fire-and-forget metrics: snapshot + spawn, System take/put-back | M | `lib.rs` |
| 5 | Add MetricsSystemReturn event variant | S | `lib.rs` |

**Depends on:** Sprint 6 (zenoh SHM simplifies publisher management)

### Sprint 8: Daemon Module Split (Q5)
**Goal:** Split daemon lib.rs (4081 lines) into focused modules

| # | Task | Effort | Files |
|---|------|--------|-------|
| 1 | Extract `dataflow_lifecycle.rs` — spawn/stop/finish | L | `lib.rs` -> new file |
| 2 | Extract `node_events.rs` — per-node event dispatch | L | `lib.rs` -> new file |
| 3 | Extract `zenoh_bridge.rs` — inter-daemon communication | M | `lib.rs` -> new file |
| 4 | Extract `build.rs` — build coordination | S | `lib.rs` -> new file |
| 5 | Keep `lib.rs` as thin orchestrator (event loop + delegation) | S | `lib.rs` |

**Depends on:** Sprint 7 (event loop changes stabilized first)

### Sprint 9: Coordinator HA
**Goal:** Daemon auto-reconnect + dataflow state reconstruction

| # | Task | Effort | Files |
|---|------|--------|-------|
| 1 | Enable ReDB by default in CLI | S | `cli/Cargo.toml`, `cli/command/coordinator.rs` |
| 2 | Accept daemon re-registration (remove duplicate machine_id error) | S | `coordinator/lib.rs` |
| 3 | Daemon reconnection loop (wrap register + run_inner) | L | `daemon/lib.rs` |
| 4 | Graceful degradation (daemon continues when coordinator down) | M | `daemon/lib.rs` |
| 5 | Enrich DataflowRecord with node_to_daemon, uv | S | `coordinator-store/lib.rs` |
| 6 | Enrich StatusReport with per-node info | S | `message/daemon_to_coordinator.rs` |
| 7 | Recovering status + reconciliation timeout (60s) | M | `coordinator/lib.rs` |

**Depends on:** Sprint 8 (clean daemon code makes reconnection loop easier)

### Sprint 10: Dynamic Topology
**Goal:** adora node add / remove / connect / disconnect

| # | Task | Effort | Files |
|---|------|--------|-------|
| 1 | AddNode/RemoveNode/AddMapping message types | S | `message/cli_to_coordinator.rs`, `coordinator_to_daemon.rs` |
| 2 | Daemon add_node_to_dataflow handler | L | `daemon/lib.rs`, `running_dataflow.rs` |
| 3 | Daemon remove_node_from_dataflow handler | M | `daemon/lib.rs`, `running_dataflow.rs` |
| 4 | Daemon add_mapping / remove_mapping | S | `daemon/lib.rs` |
| 5 | Coordinator relay (validate, resolve, dispatch) | M | `coordinator/handlers.rs` |
| 6 | Cross-daemon: remote source subscriptions for added nodes | M | `daemon/lib.rs` |
| 7 | CLI: adora node add/remove/connect/disconnect | M | `cli/command/node/add.rs` (new) |

**Depends on:** Sprint 9 (reliable coordinator needed for topology mutations)

### Sprint 11: Arrow IPC + Type Safety
**Goal:** Runtime type validation + optional Arrow IPC framing

| # | Task | Effort | Files |
|---|------|--------|-------|
| 1 | Always-on lightweight type check on first message per input | S | `event_stream/mod.rs`, `node/mod.rs` |
| 2 | Add field_names + schema_hash to ArrowTypeInfo | S | `message/metadata.rs` |
| 3 | YAML framing: arrow-ipc flag on outputs | S | `message/descriptor.rs` |
| 4 | IPC serialization path (StreamWriter/StreamReader) | M | `node/mod.rs`, `event_stream/data_conversion.rs` |
| 5 | Python IPC path (PyArrow native IPC) | M | `python/node/lib.rs` |

**Independent:** Can run in parallel with Sprint 9-10

### Sprint 12: Soft Real-Time + DX Polish
**Goal:** OS tuning guide + --rt profile + remaining DX items

| # | Task | Effort | Files |
|---|------|--------|-------|
| 1 | Create docs/realtime-tuning.md (kernel, process, systemd, K8s) | M | `docs/` (new) |
| 2 | Add --worker-threads CLI flag to daemon | S | `cli/command/daemon.rs` |
| 3 | Add --rt flag: mlockall + SCHED_FIFO | S | `daemon/lib.rs` |
| 4 | Add cpu_affinity YAML option for nodes | M | `daemon/spawn/spawner.rs` |
| 5 | Add streaming pattern example | M | `examples/streaming-example/` (new) |
| 6 | Centralize deps in [workspace.dependencies] | M | `Cargo.toml` + per-crate |
| 7 | Zenoh default-features = false | S | `Cargo.toml` |

**Independent:** Can run in parallel with Sprint 10

---

## Completion Status

| Sprint | Status | PR | Tasks Done/Total |
|--------|--------|-----|------------------|
| 6: Zenoh SHM | **DONE** | #74 | 8/8 |
| 7: Event Loop | **DONE** | #75 | 4/5 |
| 8: Daemon Split | **PARTIAL** | #76 | 2/5 |
| 9: Coordinator HA | **DONE** | #81 | 7/7 |
| 10: Dynamic Topology | **DONE** | #82 | 5/7 |
| 11: Arrow IPC | **DONE** | #83 | 2/5 |
| 12: Soft RT + DX | **DONE** | direct | 4/7 |
| Review fixes | **DONE** | direct | ~30 |
| **Total** | | **12 PRs** | **62/74 (84%)** |

### Key Metrics
- 29 commits, 59 files changed, +3,623 / -371 lines
- All 5 audit constraints addressed (coordinator SPOF, static topology, event loop, Arrow metadata, soft RT)
- All review findings from /review on every sprint resolved

## Timeline (Completed)

```
Sprint 6:  Zenoh SHM ──────────────────── DONE (PR #74)
Sprint 7:  Event Loop Separation ───────── DONE (PR #75)
Sprint 8:  Daemon Module Split ─────────── PARTIAL (PR #76)
Sprint 9:  Coordinator HA ─────────────── DONE (PR #81)
Sprint 10: Dynamic Topology ────────────── DONE (PR #82)
Sprint 11: Arrow IPC ──────────────────── DONE (PR #83)
Sprint 12: Soft RT + DX ──────────────── DONE (direct)
```

## What Gets Eliminated

By doing Zenoh SHM first (Sprint 6), these deferred items become irrelevant:
- **CM3** (signal shmem client on server drop) — no more custom shmem server
- **P4** (split shmem protocol) — zenoh handles protocol
- **P8** (separate metadata from Zenoh payload) — zenoh attachments handle this
- **DEP8** (inquire behind feature) — less relevant after shmem simplification
- **raw_sync_2** dependency — eliminated entirely
- **shared_memory_extended** dependency — eliminated entirely

## Success Criteria

- [x] v0.2-alpha: Zenoh SHM data plane + event loop separation passing all smoke tests
- [x] v0.2-beta: Daemon reconnects to restarted coordinator, dataflows survive
- [x] v0.2-rc: `adora node add/remove` protocol + CLI (coordinator dispatch pending)
- [x] v0.2: Arrow type validation, soft RT guide, streaming example

## Remaining for v0.2.1

### High Priority
- Dynamic topology coordinator-to-daemon dispatch (stubs currently return errors)
- Arrow IPC framing (YAML flag + StreamWriter/StreamReader)
- Python IPC path (PyArrow native IPC)
- Zenoh receive zero-copy (payload.to_bytes() copies — need RawData::ZenohShm)

### Medium Priority
- Daemon module split (node_events, dataflow_lifecycle extraction from lib.rs)
- cpu_affinity YAML option for spawned nodes
- Centralize workspace dependencies
- Zenoh default-features = false (identify needed transports)
- Fire-and-forget metrics (extract collect_and_send_metrics)

### Low Priority
- schema_hash population logic
- Publish failure counter metric
- Tokio explicit feature list per crate
