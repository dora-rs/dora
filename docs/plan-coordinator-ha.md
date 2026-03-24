# Coordinator SPOF Hardening Plan

## Context

The coordinator is a single point of failure — a crash kills all daemons after 30s and loses all in-memory state. The ReDB persistent store exists but is opt-in and the recovery flow is naive (marks everything Failed). Daemons don't auto-reconnect.

Goal: incremental hardening so that a coordinator restart is survivable without losing running dataflows.

## Current State

| Aspect | Status |
|--------|--------|
| Persistent store (ReDB) | Exists, optional, behind feature flag |
| Startup recovery | Marks in-flight dataflows as Failed |
| Daemon reconnect | Not implemented — daemon exits on connection loss |
| Graceful degradation | Not implemented — daemon bails immediately |
| StatusReport | Sends `Vec<DataflowId>` only (no per-node info) |
| Re-registration | Rejects duplicate `machine_id` |

## Phase 1: Daemon Auto-Reconnect (Most Impactful)

### 1.1 Daemon reconnection loop

**File:** `binaries/daemon/src/lib.rs`

Wrap `register()` + `run_inner()` in a reconnection loop. On connection loss:
- Keep all `running` dataflow state (nodes are separate processes)
- Re-call `register()` with exponential backoff
- Re-create event streams, re-enter `run_inner`

### 1.2 Accept daemon re-registration

**File:** `binaries/coordinator/src/lib.rs`

Remove the error on duplicate `machine_id` registration. `DaemonConnections::add()` already replaces stale connections. A reconnecting daemon with the same `machine_id` should be accepted.

### 1.3 Persist daemon ID across reconnects

**Files:** `libraries/message/src/daemon_to_coordinator.rs`, `binaries/daemon/src/coordinator.rs`

Add `previous_daemon_id: Option<DaemonId>` to `DaemonRegisterRequest` (`#[serde(default)]` for backward compat). Coordinator reuses the ID if the previous connection is dead.

### 1.4 Graceful degradation

**File:** `binaries/daemon/src/lib.rs`

Instead of `bail!("lost connection to coordinator")`, set `coordinator_sender = None` and continue the event loop. Skip non-critical sends (logs, metrics, heartbeats). Buffer critical ones (build results, spawn results) for replay on reconnect.

## Phase 2: Dataflow State Reconstruction

### 2.1 Enrich DataflowRecord

**File:** `libraries/coordinator-store/src/lib.rs`

Add to `DataflowRecord`:
- `node_to_daemon: BTreeMap<String, String>` — per-node daemon assignment
- `uv: bool` — Python UV flag for restart

Bump schema version in `redb_store.rs`.

### 2.2 Enrich StatusReport

**File:** `libraries/message/src/daemon_to_coordinator.rs`

Change `StatusReport { running_dataflows: Vec<DataflowId> }` to include per-node info:
```rust
StatusReport {
    running_dataflows: Vec<DataflowStatusEntry>,
}
struct DataflowStatusEntry {
    dataflow_id: Uuid,
    running_nodes: Vec<NodeId>,
}
```

### 2.3 Recovering status + reconciliation

**File:** `binaries/coordinator/src/lib.rs`

On startup:
- Mark `Running`/`Pending` dataflows as `Recovering` (new status), NOT `Failed`
- Start a 60s timeout
- When daemons reconnect and send StatusReport, match against `Recovering` dataflows
- Reconstruct `RunningDataflow` from persisted descriptor + daemon's reported nodes
- After 60s, mark any still-`Recovering` dataflows as `Failed`

## Phase 3: Enable ReDB by Default

### 3.1 CLI defaults

**File:** `binaries/cli/Cargo.toml` — add `redb-backend` to default features

**File:** `binaries/cli/src/command/coordinator.rs` — change `--store` default from `"memory"` to `"redb"`

### 3.2 Local mode

Keep `InMemoryStore` for `adora run` — local mode is ephemeral by design.

## Implementation Order

1. Phase 3 (ReDB default) — trivial, do first
2. Phase 1.2 (accept re-registration) — small coordinator change, unblocks everything
3. Phase 1.1 + 1.3 (daemon reconnect loop) — daemon-side
4. Phase 1.4 (graceful degradation) — daemon keeps running when coordinator is down
5. Phase 2.1 (enrich DataflowRecord) — store schema
6. Phase 2.2 (enrich StatusReport) — message schema
7. Phase 2.3 (recovery + reconciliation) — coordinator startup

## Key Files

| File | Change |
|------|--------|
| `binaries/daemon/src/lib.rs` | Reconnection loop, graceful degradation |
| `binaries/coordinator/src/lib.rs` | Re-registration, recovery logic |
| `libraries/message/src/daemon_to_coordinator.rs` | DaemonRegisterRequest, StatusReport |
| `libraries/coordinator-store/src/lib.rs` | DataflowRecord fields, Recovering status |
| `libraries/coordinator-store/src/redb_store.rs` | Schema version bump |
| `binaries/cli/src/command/coordinator.rs` | Store default flip |

## Risks

- **Phase 1.4 (graceful degradation)** is riskiest — many daemon code paths assume coordinator is available. Need `if coordinator.is_some()` guards on logs, metrics, heartbeats.
- **Phase 2.3 (reconstruction)** has edge cases — partial daemon reconnect, nodes that died during outage. The 60s timeout handles this gracefully.
- **Backward compat**: `Option<DaemonId>` with `#[serde(default)]` ensures old/new daemon/coordinator interop.

## Non-Goals (v0.3+)

- Leader election / coordinator standby
- Raft consensus
- Automatic coordinator failover
- Multi-coordinator distributed state
