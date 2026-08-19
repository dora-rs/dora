# Migration Guide: dora 0.x → 1.0

> **Status (2026-04-16):** minimum-viable scaffold. The full migration guide with per-API before/after examples is tracked as follow-up work.
>
> Previous filename: `dora-compatibility.md` (which documented a fork→upstream compat layer that is obsolete under the tree-takeover consolidation strategy).
>
> **Cross-reference caveat:** the audit evidence files named below (`phase--1-audit-2026-04-16.md`, `audit-2026-03-21-closure.md`, `ownership-verification-2026-04-16.md`, `downstream-user-assessment-2026-04-16.md`) currently live on the `docs/consolidation-plan-review` branch (PR #286) and are not yet on `main`. They land with that PR. Filenames are written as plain text below so no link 404s if the merge order flips.

## What changed

dora 1.0 is the consolidation of the fork tree into the upstream `dora-rs/dora` repository. See [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) for the full context.

**Headline:** 1.0 is a **hard break** from 0.x. You cannot mix-and-match 0.x and 1.0 components in a running cluster. Plan a full-cluster restart for the upgrade.

## Before you upgrade — know the breaking changes

| Surface | 0.x behaviour | 1.0 behaviour | Evidence |
|---|---|---|---|
| Wire protocol (CLI ↔ coordinator) | tarpc over TCP with JSON framing | WebSocket with new message shapes | `phase--1-audit-2026-04-16.md` §4 (lands with PR #286) |
| Message-enum variants (`NodeEvent`, `DaemonCommunication`) | 0.x variant order | New variants inserted (`InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted`, `Shmem`) — bincode tags shifted | `phase--1-audit-2026-04-16.md` §3 (lands with PR #286) |
| CLI handshake | Optional `get_version()` RPC | Mandatory `ControlRequest::Hello` with semver check | `libraries/message/src/cli_to_coordinator.rs` |
| Recording file extension | `.adorec` | `.drec` (magic bytes were already `DORAREC\x00` / `DORAEND\x00`) | `libraries/recording/src/lib.rs` |
| Request-reply communication layer | `libraries/communication-layer/request-reply/` | Replaced by `send_service_request()` / `send_service_response()` helpers and service/action patterns | [`docs/patterns.md`](patterns.md) |
| Auth | Query-parameter token | Bearer header token, constant-time comparison | `libraries/message/src/auth.rs` |

Additional fixes and hardening items from the 2026-03-21 audit are closed in 1.0; see `audit-2026-03-21-closure.md` (lands with PR #286) for the full per-finding record.

## Upgrade path

1. **Update `Cargo.toml` dependencies.** Bump `dora-node-api`, `dora-operator-api`, `dora-cli` to `1.0`. The crate names are unchanged; no rename.
2. **Rebuild every component.** Daemons, coordinators, CLI, and every node/operator binary must be rebuilt against the 1.0 crates. Do not run 0.x binaries against 1.0 daemons.
3. **Update YAML descriptors if needed.** Most 0.x descriptors work unchanged, but a few fields have new defaults or renames — see [`yaml-spec.md`](yaml-spec.md).
4. **Rename existing recordings.** `mv capture.adorec capture.drec` — the file format itself is unchanged, only the extension differs.
5. **Update CLI usage.** Any tooling that pipes files into `dora replay` should use `.drec` going forward. Shell completions and aliases referencing `*.adorec` need updating.
6. **Coordinate the restart.** In a distributed deployment, bring down all 0.x daemons before starting 1.0 daemons. A fork CLI connecting to an upstream 0.x coordinator (or vice versa) will fail at the TCP handshake with a low-level framing error, not a graceful version-mismatch message.
7. **If you installed the `adora-rs` PyPI package** (from the fork-era 0.x releases): uninstall it and install `dora-rs` directly. 1.0 is a clean break — `adora-rs` is not republished, so `pip install adora-rs` resolves to the last fork-era 0.x release and will not pull 1.0. Rust users on crates.io are unaffected — the fork never published `adora-*` crates.

## New features in 1.0

Non-exhaustive; see the full 1.0 release notes for the complete list.

- Service pattern: `send_service_request()` / `send_service_response()` with correlation IDs
- Action pattern: goal / feedback / result metadata with cancellation
- Streaming chunks: `send_stream_chunk()` + `StreamSegment`
- Structured logging: `node.log()` / `node.log_info()` bridged from Python `logging`
- Restart awareness: `is_restart()` / `restart_count()`
- Input health: `InputTracker` / `InputState`
- Fault-tolerance snapshots + coordinator state catch-up
- `DoraNodeBuilder` with custom daemon port (upstream PR #1591 compatible)
- CUDA IPC via ctypes (no more `numba` dependency)

## Removed in 1.0

Features and code that existed in dora 0.x but are absent from 1.0. If you depended on one of these, the 0.x upgrade path doesn't cover it — contact the maintainers if you need it restored.

| 0.x surface | Status in 1.0 | Replacement / rationale |
|---|---|---|
| `dora destroy` CLI command | Removed | Use `dora down` instead — same function, one consolidated command |
| `dora version` subcommand (queries coordinator for version info) | Removed | `dora --version` (clap auto) shows the CLI version. Runtime version negotiation happens automatically via the `ControlRequest::Hello` handshake on every CLI↔coordinator connect. |
| `libraries/communication-layer/request-reply/` (TCP-backed `RequestReplyLayer` trait) | Removed | Superseded by the service/action patterns using Arrow metadata correlation IDs — `send_service_request()` / `send_service_response()` helpers on `DoraNode`. See [`docs/patterns.md`](patterns.md). |
| Coordinator TCP listener (`listener.rs`, `server.rs`, `tcp_utils.rs`) | Removed | Replaced by the WebSocket control plane (`ws_server.rs`, `ws_control.rs`, `ws_daemon.rs`). Per D-1a, this is a **hard protocol break** — 0.x daemons cannot interoperate with 1.0 coordinators. |
| Daemon `hot_reload.rs` module (293 lines) | Removed | Inlined into `binaries/daemon/src/lib.rs` as `send_reload()`. Same functionality, leaner file layout. |
| Daemon `state.rs` module (172 lines) | Removed | Refactored into `running_dataflow.rs` + `event_types.rs`. Same state handling, clearer separation of concerns. |
| Upstream `libraries/extensions/ros2-bridge/python/src/typed/deserialize/string.rs` | Moved | Functionality is in `libraries/extensions/ros2-bridge/arrow/src/deserialize/string.rs` and additionally supports `WStringDeserializer` (which was `todo!()` in 0.x). |

## Descriptor changes (dataflow YAML)

These landed during the 1.0 RC window. All are rejected with an error naming
the offending field rather than being silently ignored — `Descriptor` and
`Debug` both use `deny_unknown_fields`, which matters: a silently-dropped
`deploy` block would run every node on the local daemon while the YAML looks
like it pinned them to machines.

### `custom:` removed (#3158)

Use the top-level fields directly.

```yaml
# 0.x
- id: my-node
  custom:
    path: node.py
    args: --flag
    inputs:
      tick: dora/timer/millis/100
    outputs: [data]

# 1.0
- id: my-node
  path: node.py
  args: --flag
  inputs:
    tick: dora/timer/millis/100
  outputs: [data]
```

`custom.envs` has no direct replacement — use the node-level `env:` map.

### `_unstable_` key prefix dropped (#3220)

```yaml
# 0.x                    # 1.0
_unstable_deploy:        deploy:
  machine: m1              machine: m1
_unstable_debug:         debug:
  enable_debug_inspection: true
```

Both the dataflow-level and the per-node `deploy` key are affected.

### `publish_all_messages_to_zenoh` alias removed (#3158)

```yaml
# 0.x                                    # 1.0
debug:                                   debug:
  publish_all_messages_to_zenoh: true      enable_debug_inspection: true
```

### `communication:` block removed (#3158)

Delete it. `_unstable_local` and `_unstable_remote` were single-variant enums
whose only value was the default, so the block never changed behaviour.

## Rust API changes

### Arrow types are no longer in dora's public API (#3213)

The largest change for node authors. `dora-node-api` no longer names an Arrow
type, so dora can move to a newer Arrow major in a minor release without
breaking your build. See the Arrow version policy in
[`api-rust.md`](api-rust.md).

**`ArrowData` is now `DoraArray`**, with a private field rather than
`pub ArrayRef` and no `Deref` to `ArrayRef`:

```rust
// 0.x
Event::Input { id, data, .. } => {
    let arr: &ArrayRef = &data;          // via Deref
    let v: u32 = (&data).try_into()?;
}

// 1.0
Event::Input { id, data, .. } => {
    let v: u32 = (&data).try_into()?;    // unchanged
    // for raw Arrow access, enable the `arrow-v59` feature:
    let arr = data.as_array();
}
```

**`IntoArrow` lost its associated type.** If you implemented it by hand:

```rust
// 0.x
impl IntoArrow for MyType {
    type A = arrow::array::UInt32Array;
    fn into_arrow(self) -> Self::A { ... }
}

// 1.0
impl IntoArrow for MyType {
    fn into_arrow(self) -> DoraArray { ... }
}
```

**`pub use arrow;` is now feature-gated and version-suffixed.** A bare `arrow`
re-export changes meaning silently when dora bumps; `arrow_v59` cannot.

```toml
# 1.0 — only if you name Arrow types directly
dora-node-api = { version = "1", features = ["arrow-v59"] }
```

```rust
// 0.x                              // 1.0
use dora_node_api::arrow;           use dora_node_api::arrow_v59 as arrow;
```

Most nodes need neither: `value.into_arrow()` and `(&data).try_into()?` never
name an Arrow type, which is why the `dora new` templates were unchanged.

### `dora_core` re-export narrowed (#3221)

`dora_node_api::dora_core` now exposes only `config::{DataId, NodeId,
OperatorId}` and `uhlc`. The scaffolded path is unchanged:

```rust
use dora_node_api::dora_core::config::DataId;   // still works
```

If you reached `dora_node_api::dora_core::{descriptor, topics, build,
manifest, types}`, depend on `dora-core` directly instead — but note it is an
internal crate outside the 1.0 guarantee (see "Stability scope at 1.0" in
[`api-rust.md`](api-rust.md)).

## Still to do (this guide)

- ~~Per-API before/after code snippets~~ — done for the RC-window breaking changes above; older 0.x-era API deltas still uncovered
- Dedicated section on the C/C++ API (headers stable, call signatures unchanged)
- Dedicated section on the Python API
- ~~`dora migrate` subcommand usage~~ — **dropped per #297 resolution**. No migration tool ships in 1.0; manual steps above + release-note hard-break callout only.

## Related documents

- [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) — the full consolidation plan (on main)
- `phase--1-audit-2026-04-16.md` — wire-protocol audit evidence (lands with PR #286)
- `audit-2026-03-21-closure.md` — security/correctness closure (lands with PR #286)
- `ownership-verification-2026-04-16.md` — publish-path readiness (lands with PR #286)
- `downstream-user-assessment-2026-04-16.md` — who this matters for (lands with PR #286)
