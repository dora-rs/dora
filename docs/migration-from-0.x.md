# Migration Guide: dora 0.x → 1.0

> **Status (2026-08-29):** complete for the 1.0 release. Every language API (Rust, Python, C, C++), the descriptor surface, and the wire format now have a section with before/after examples. What is deliberately not covered is listed under [Still to do](#still-to-do-this-guide).
>
> Previous filename: `dora-compatibility.md` (which documented a fork→upstream compat layer that is obsolete under the tree-takeover consolidation strategy).

## What changed

dora 1.0 is the consolidation of the fork tree into the upstream `dora-rs/dora` repository. See [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) for the full context.

**Headline:** 1.0 is a **hard break** from 0.x. You cannot mix-and-match 0.x and 1.0 components in a running cluster, because the binary encoding itself changed — see [Wire format](#wire-format-a-0x-peer-cannot-talk-to-a-10-peer). Plan a full-cluster restart for the upgrade.

The 0.x tree is preserved under the `v0.x-final` tag. It shares no history with 1.0 — `git merge-base main v0.x-final` has no answer — so "removed in 1.0" below means "present at `v0.x-final`, absent on `main`", not "deleted by a commit you can bisect to".

## Before you upgrade — know the breaking changes

| Surface | 0.x behaviour | 1.0 behaviour | Evidence |
|---|---|---|---|
| Wire protocol (CLI ↔ coordinator) | tarpc over TCP with JSON framing | WebSocket with new message shapes | [`phase--1-audit-2026-04-16.md`](phase--1-audit-2026-04-16.md) §4 |
| Wire encoding | bincode | postcard — no mixed-version mode (#3153) | [Wire format](#bincode--postcard-3153) |
| HLC timestamp in JSON payloads | `id` is a `u128` number | `id` is a 16-byte array (uhlc 0.9, #3016) — binary plane unaffected | [uhlc 0.5 → 0.9](#uhlc-05--09-3016--the-json-plane-only) |
| Message-enum variants (`NodeEvent`, `DaemonCommunication`) | 0.x variant order | New variants inserted (`InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted`, `Shmem`) — positional tags shifted | [`phase--1-audit-2026-04-16.md`](phase--1-audit-2026-04-16.md) §3 |
| Wire-protocol enums in `dora-message` | exhaustively matchable | `#[non_exhaustive]` — a `_ =>` arm is required (#3151) | `libraries/message/src/` |
| CLI handshake | Optional `get_version()` RPC | Mandatory `ControlRequest::Hello` with semver check | `libraries/message/src/cli_to_coordinator.rs` |
| Topic data channel | subscription carried no encoding version | `protocol_version` in both directions; mismatch is refused (#3160) | [Topic subscriptions](#topic-subscriptions-are-refused-on-encoding-mismatch-3160) |
| Recording files | `.adorec`, `FORMAT_VERSION` 1 (bincode entries) | `.drec`, `FORMAT_VERSION` 2 (postcard entries); v1 is refused, not upgraded | `libraries/recording/src/lib.rs` |
| Request-reply communication layer | `libraries/communication-layer/request-reply/` | Replaced by `send_service_request()` / `send_service_response()` helpers and service/action patterns | [`docs/patterns.md`](patterns.md) |
| Auth | Query-parameter token | Bearer header token, constant-time comparison | `libraries/message/src/auth.rs` |
| Python interpreter floor | `requires-python = ">=3.8"` | `requires-python = ">=3.11"` (abi3-py311) | `apis/python/node/pyproject.toml` |

Additional fixes and hardening items from the 2026-03-21 audit are closed in 1.0; see [`audit-2026-03-21-closure.md`](audit-2026-03-21-closure.md) for the full per-finding record.

## Wire format: a 0.x peer cannot talk to a 1.0 peer

This is the single most important thing to know before upgrading, and no configuration recovers it: **there is no mixed-version mode.** The binary encoding changed during the rc window, so a 0.x node pointed at a 1.0 daemon does not degrade or negotiate down — it fails on the first frame.

### `bincode` → `postcard` (#3153)

bincode is unmaintained (RUSTSEC-2025-0141): development stopped at 1.3.3 and every version is flagged, so it was not something to carry into an encoding that 1.0 commits to. postcard is serde-based, so no message *type* changed — only the bytes — and unlike bincode it has a documented, stable wire spec, which is the property that matters under a stability guarantee. Messages also shrink 25–27 bytes each (`Metadata`: 34 → 27 B) from varint integers and length prefixes.

Four format versions moved with it. Each one fails loudly rather than misparsing, which is the whole point — both encodings are positional, so a tolerated mismatch would surface as corrupt data rather than an error:

| Surface | 0.x | 1.0 | What a stale peer or file gets |
|---|---|---|---|
| `Metadata::CURRENT_VERSION` (node ↔ daemon) | 1 | 2 | decode failure at register — see the caveat below |
| Coordinator store `SCHEMA_VERSION` (redb) | 4 | 5 | coordinator refuses to open the database |
| `.drec` `FORMAT_VERSION` | 1 | 2 | `dora replay` refuses the file |
| Topic data channel `protocol_version` | absent, or 1 | 2 | subscription refused at handshake (#3160) |

**Do not expect a clean version error from a 0.x node.** `NodeRegisterRequest` carries a `metadata_version` field so that layout drift *within* one encoding is caught at register with a legible message.

It cannot catch a change of the encoding itself: the register frame is encoded the same way as every other frame, so decoding fails on the frame that carries the check before the check can run. Expect a low-level deserialization error, not `message wire-format mismatch: node speaks metadata format v1`.

If you run a persistent coordinator store, the redb file from 0.x is not migrated. The coordinator names the file and tells you what to do:

```
redb schema version mismatch: database at `~/.dora/coordinator.redb` has v4,
but this binary expects v5. Delete the file and restart to create a fresh
database, or use `--store memory` to bypass persistence.
```

This only affects `--store redb`; the default `memory` backend keeps nothing across a restart and needs no action.

### `uhlc` 0.5 → 0.9 (#3016) — the JSON plane only

Worth stating precisely, because the natural assumption is wrong: **this bump did not change the binary plane.** uhlc 0.9 changes `ID`'s in-memory representation from `NonZeroU128` to `[u8; 16]`, but the two serialize identically — a `u128` is written little-endian, and 0.5 built that `u128` from the id bytes in little-endian order to begin with.

The golden vectors in `libraries/message/tests/uhlc_wire_format.rs` were run against both 0.5.2 and 0.9.0 with the real `dora-message` types, and every vector matches byte for byte. What 0.9 actually reversed is `ID`'s `Display` / `FromStr`, which dora never uses.

Daemon ↔ node messages, inter-daemon Zenoh samples and `.drec` entries were untouched by this bump, and it warranted no `Metadata::CURRENT_VERSION` change of its own.

What does change shape is the JSON plane. On the CLI ↔ coordinator and coordinator ↔ daemon WebSocket links, a timestamp's `id` goes from a bare `u128` number to a 16-element byte array:

```json
// 0.x
{"timestamp": {"time": 7346545054874578944, "id": 133075017253481751908959400507149664154}}

// 1.0
{"timestamp": {"time": 7346545054874578944,
               "id": [154, 63, 12, 113, 212, 40, 78, 182, 21, 195, 135, 42, 233, 80, 29, 100]}}
```

Both links are gated by the `dora_version` semver handshake, so a mismatch fails at parse time with a serde type error rather than being misread, and nothing persists a uhlc timestamp as JSON on disk. This reaches you only if you have your own tooling parsing those WebSocket payloads.

The new shape is strictly easier to consume. A real HLC id uses all 16 bytes, so under 0.5 the `u128` form exceeded every number `serde_json::Value` can hold (i64/u64/f64) — `serde_json::to_value` failed outright with "number out of range" on every timestamp, which is why dora's own components passed pre-serialized JSON fragments around instead.

The `[u8; 16]` form round-trips through `serde_json::Value` losslessly. If you were working around that limit with a bignum-aware parser or raw string handling, you no longer need to.

The bump was taken deliberately *before* the freeze rather than after it: it needed no source changes beyond an error type that all eight call sites only `Display`, and doing it afterwards would have cost a major bump.

### Topic subscriptions are refused on encoding mismatch (#3160)

The WebSocket topic data channel carries raw `Timestamped<InterDaemonEvent>` bytes to third-party subscribers, forwarded as-is with no envelope and no self-describing encoding. A subscriber speaking a different binary format therefore *misparses* those frames rather than failing to decode them — which before 1.0 meant silent corruption, since the channel had no version exchange at all.

The subscription handshake now carries `protocol_version` in both directions, and either side refuses on mismatch:

```json
{"TopicSubscribe":  {"dataflow_id": "...", "topics": [...], "protocol_version": 2}}
{"TopicSubscribed": {"subscription_id": "...", "protocol_version": 2}}
```

Both fields are `#[serde(default)]`, so a peer predating the handshake omits them, deserializes to `None`, and is rejected for the same reason a wrong number is. The rejection names both sides so you can tell which one is old:

```
topic data protocol mismatch: client speaks version 1, this side speaks 2.
Binary frames are positionally encoded, so subscribing would silently misparse
rather than fail. Upgrade whichever side is older.
```

**If you wrote your own topic subscriber**, send `protocol_version: dora_message::TOPIC_DATA_PROTOCOL_VERSION` (currently `2`) in `TopicSubscribe` and check the value echoed in `TopicSubscribed` before consuming frames.

The constant exists separately from the `Hello` semver handshake on purpose: third-party subscribers never send `Hello`, and `versions_compatible` is semver-caret, so a 1.0 and a 1.5 peer are "compatible" and `Hello` would wave through a future encoding change inside the 1.x series that only this version catches.

The full frame layout is in [`websocket-topic-data-channel.md`](websocket-topic-data-channel.md).

### 0.x recordings do not replay

`.drec` entries are `Timestamped<InterDaemonEvent>` payloads, so they moved encoding with everything else. The container framing and the magic bytes (`DORAREC\x00` / `DORAEND\x00`) are unchanged, which is exactly why the reader carries an explicit floor: without it a v1 header would pass the magic and version checks and then fail per entry, surfacing as a corruption error instead of a version one.

```
recording format version 1 is no longer supported (min supported: 2); it was
written by a dora release that encoded events with bincode. Re-record with this
version of dora.
```

There is no converter, and renaming `.adorec` to `.drec` does not produce a readable file. If a capture matters, keep a `v0.x-final` build around to replay it; otherwise re-record under 1.0.

## Upgrade path

1. **Update `Cargo.toml` dependencies.** Bump `dora-node-api`, `dora-operator-api`, `dora-cli` to `1.0`. The crate names are unchanged; no rename.
2. **Rebuild every component.** Daemons, coordinators, CLI, and every node/operator binary must be rebuilt against the 1.0 crates. Do not run 0.x binaries against 1.0 daemons.
3. **Check your Python version.** 1.0 wheels need CPython 3.11 or later. If you are on 3.8–3.10, upgrade the interpreter before upgrading dora — see [Python API changes](#python-api-changes).
4. **Update YAML descriptors.** See [Descriptor changes](#descriptor-changes-dataflow-yaml) — several 0.x keys are now rejected by name rather than ignored.
5. **Re-record, don't rename, existing recordings.** Renaming is not enough: `.drec` `FORMAT_VERSION` went 1 → 2 when entry payloads moved to postcard, and the reader's floor is 2, so a 0.x capture is refused outright rather than misread — see [0.x recordings do not replay](#0x-recordings-do-not-replay). The container framing and magic bytes (`DORAREC\x00` / `DORAEND\x00`) are unchanged, but the entries inside are not.
6. **Update CLI usage.** Any tooling that pipes files into `dora replay` should use `.drec` going forward. Shell completions and aliases referencing `*.adorec` need updating.
7. **Coordinate the restart.** In a distributed deployment, bring down all 0.x daemons before starting 1.0 daemons. A fork CLI connecting to an upstream 0.x coordinator (or vice versa) will fail at the TCP handshake with a low-level framing error, not a graceful version-mismatch message.
8. **If you installed the `adora-rs` PyPI package** (from the fork-era 0.x releases): uninstall it and install `dora-rs` directly. 1.0 is a clean break — `adora-rs` is not republished, so `pip install adora-rs` resolves to the last fork-era 0.x release and will not pull 1.0. Rust users on crates.io are unaffected — the fork never published `adora-*` crates.

## New features in 1.0

Non-exhaustive; see the full 1.0 release notes for the complete list.

- Service pattern: `send_service_request()` / `send_service_response()` with correlation IDs
- Action pattern: goal / feedback / result metadata with cancellation
- Streaming chunks: `send_stream_chunk()` + `StreamSegment`
- Structured logging: `node.log()` / `node.log_info()` bridged from Python `logging`
- Restart awareness: `is_restart()` / `restart_count()`, plus the `NodeRestarted` and `InputRecovered` events
- Runtime parameters: `dora param set` / `delete`, surfaced as `ParamUpdate` / `ParamDeleted` events
- Input health: `InputTracker` / `InputState`
- Fault-tolerance snapshots + coordinator state catch-up
- `DoraNodeBuilder` with custom daemon port (upstream PR #1591 compatible)
- Arrow IPC data plane: a self-describing official wire format with a ≤1-copy send path (#2366)
- Generic extension seam (`ExtensionRequest` / `ExtensionMessage`) so optional transports stay out of the frozen protocol (#3219)
- CUDA IPC via ctypes, no `numba` dependency — shipped in the `dora_tensor_pool` extension rather than the `dora` package (#3249)

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

These landed during the 1.0 RC window. All are rejected with an error naming the offending field rather than being silently ignored — `Descriptor` and `Debug` both use `deny_unknown_fields`, which matters: a silently-dropped `deploy` block would run every node on the local daemon while the YAML looks like it pinned them to machines.

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

Delete it. `_unstable_local` and `_unstable_remote` were single-variant enums whose only value was the default, so the block never changed behaviour.

## Rust API changes

### Fallible methods return `NodeResult`, not `eyre::Result`

Every public `DoraNode` and `EventStream` method changed its error type from `eyre::Report` to the concrete [`NodeError`](../apis/rust/node/src/error.rs) enum, aliased as `NodeResult<T>`.

```rust
// 0.x
use dora_node_api::DoraNode;
fn run() -> eyre::Result<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    Ok(())
}

// 1.0
use dora_node_api::{DoraNode, NodeResult};
fn run() -> NodeResult<()> {
    let (mut node, mut events) = DoraNode::init_from_env()?;
    Ok(())
}
```

Most code does not have to change: `NodeError` implements `std::error::Error + Send + Sync + 'static`, so `?` still converts into `eyre::Result`, `anyhow::Result`, and `Box<dyn Error>`. What breaks is code that *names* the type — an explicit `let r: eyre::Result<()> = node.send_output(...)`, a function whose signature promised `eyre::Report`, or a `match` on the error.

`NodeError` has four variants: `Init`, `Output`, `Data`, and `Internal(eyre::Report)`. The pattern-aware receive helpers (`recv_service_response`, `recv_action_result`) return `PatternError` instead, with `Timeout` / `ServerRestarted` / `StreamEnded` / `StreamError`.

### Arrow types are no longer in dora's public API (#3213)

The largest change for node authors. `dora-node-api` no longer names an Arrow type, so dora can move to a newer Arrow major in a minor release without breaking your build. See the Arrow version policy in [`api-rust.md`](api-rust.md).

**`ArrowData` is now `DoraArray`**, with a private field rather than `pub ArrayRef` and no `Deref` to `ArrayRef`:

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

**`pub use arrow;` is now feature-gated and version-suffixed.** A bare `arrow` re-export changes meaning silently when dora bumps; `arrow_v59` cannot.

```toml
# 1.0 — only if you name Arrow types directly
dora-node-api = { version = "1", features = ["arrow-v59"] }
```

```rust
// 0.x                              // 1.0
use dora_node_api::arrow;           use dora_node_api::arrow_v59 as arrow;
```

If your own crate is pinned to Arrow 58 and you are not ready to move, use `arrow-v58` instead — it costs one Arrow C Data Interface hop (no buffer copy) rather than the free borrow `arrow-v59` gets. The support window and which major is free are documented in [`api-rust.md`](api-rust.md).

Most nodes need neither: `value.into_arrow()` and `(&data).try_into()?` never name an Arrow type, which is why the `dora new` templates were unchanged.

### `dora_core` re-export narrowed (#3221)

`dora_node_api::dora_core` now exposes only `config::{DataId, NodeId, OperatorId}` and `uhlc`. The scaffolded path is unchanged:

```rust
use dora_node_api::dora_core::config::DataId;   // still works
```

If you reached `dora_node_api::dora_core::{descriptor, topics, build, manifest, types}`, depend on `dora-core` directly instead — but note it is an internal crate outside the 1.0 guarantee (see "Stability scope at 1.0" in [`api-rust.md`](api-rust.md)).

### `flume` is no longer re-exported (#3237)

`pub use flume;` and `pub use flume::Receiver;` were pure convenience — no flume type appears in any public signature. Removing them takes flume out of the frozen 1.0 surface, so a future flume major is not a dora breaking change.

```rust
// 0.x
use dora_node_api::flume::{self, Receiver};

// 1.0 — add flume to your own Cargo.toml
use flume::{self, Receiver};
```

The same reasoning does **not** apply to `uuid`: `pub type DataflowId = uuid::Uuid`, so uuid stays in the frozen surface and is still re-exported as `dora_node_api::uuid`.

### `integration_testing` no longer exposes tokio (#3239)

The module re-exported three tokio items, which would have frozen a tokio major into the 1.0 guarantee. They are now newtypes owned by dora, so tests need no channel dependency of their own and dora can swap channel libraries without a breaking change.

```rust
// 0.x                                    // 1.0
unbounded_channel()                       output_channel()
UnboundedSender<OutputJson>               OutputSender
UnboundedReceiver<OutputJson>             OutputReceiver
```

The module itself is *not* feature-gated: `init_from_env` reads `DORA_TEST_WITH_INPUTS` / `DORA_TEST_WRITE_OUTPUTS_TO` / `DORA_TEST_NO_OUTPUT_TIME_OFFSET` on the normal release path, and env-var testing exists precisely to exercise the same executable the dataflow runs.

### `ArrowTypeInfo` left the send path (#2366)

Samples are now self-describing Arrow IPC streams, so the caller no longer supplies type information.

**`send_output_sample` lost its `type_info` parameter:**

```rust
// 0.x
node.send_output_sample(output_id, type_info, parameters, Some(sample))?;

// 1.0
node.send_output_sample(output_id, parameters, Some(sample))?;
```

**`send_typed_output` is gone entirely.** It existed only to pair `ArrowTypeInfo` with a fill closure. Use `send_output` (which encodes for you), or `send_output_raw` if you need the closure:

```rust
// 0.x
node.send_typed_output(output_id, type_info, parameters, len, |buf| { ... })?;

// 1.0
node.send_output_raw(output_id, parameters, len, |buf| { ... })?;
```

`send_output`, `send_output_raw` and `send_output_bytes` are otherwise unchanged apart from the `NodeResult` return type.

**`arrow_utils` is now the IPC codec.** The 0.x helpers `copy_array_into_sample`, `required_data_size`, `into_arrow_array` and `buffer_into_arrow_array` are gone; the module now offers `encode_arrow_ipc`, `decode_arrow_ipc` and `decode_arrow_ipc_zero_copy`. If you were hand-rolling the buffer layout, call `send_output` or `DoraNode::encode_arrow` instead.

### Wire-protocol enums are `#[non_exhaustive]` (#3151)

`DaemonRequest`, `DaemonReply`, `InterDaemonEvent`, `CoordinatorRequest` and `DaemonEvent` on the published `dora-message` crate now carry `#[non_exhaustive]`, so future variant additions are not semver-major. Exhaustive matches no longer compile.

```rust
// 0.x
match request {
    DaemonRequest::Subscribe => { ... }
    DaemonRequest::SendMessage { .. } => { ... }
    // ...every variant listed
}

// 1.0 — add a fallback arm
match request {
    DaemonRequest::Subscribe => { ... }
    DaemonRequest::SendMessage { .. } => { ... }
    _ => { /* unknown request from a newer peer */ }
}
```

Pick the fallback to match the role: the coordinator warns and continues (a newer daemon must not be able to tear down its connection), the daemon replies with an explicit error (so a newer node fails loudly instead of hanging), and read-only observability paths skip.

`dora_node_api::Event` was already `#[non_exhaustive]` in 0.x, so the four new variants (`InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted`) do not break existing matches.

### Pool and pinned-memory requests replaced by the extension seam (#3219)

Naming a protocol after one transport would freeze that transport's architecture into the 1.0 surface. Three groups of variants collapsed onto opaque carriers:

```rust
InterDaemonEvent::ExtensionMessage { dataflow_id, namespace, target_machine, payload }
DaemonRequest::ExtensionRequest    { namespace, payload }
DaemonReply::ExtensionReply        { payload }
```

Also removed: `DaemonRequest::RegisterPinnedMemory`, `DaemonRequest::ReadPinnedMemory`, `DaemonReply::PinnedMemoryMetadata`, and the two `DoraNode` methods that fronted them. `RegisterPinnedMemory` was never sent by any node-API method and never dispatched; `read_pinned_memory` had no callers and no daemon arm, so it could only ever come back as "unsupported request from node".

## Python API changes

### CPython 3.11 is the floor (was 3.8)

`requires-python` moved from `>=3.8` to `>=3.11`. Wheels are built `abi3-py311`, so one wheel covers every later CPython without waiting for a dora release — 3.11 is a floor dora 1.x will not raise, not a version it is pinned to.

Free-threaded builds (`python3.13t`, `python3.14t`) are **not** supported: abi3 does not cover them and no wheels are published. See the [Python version policy](api-rust.md#python-version-policy) for the full guarantee.

If you are on 3.8–3.10, upgrade the interpreter first. There is no 1.0 wheel for those versions and no compatibility shim.

### `numpy` is now a hard dependency

0.x declared `pyarrow>=14.0.1` and `pyyaml>=6.0`. 1.0 adds `numpy>=1.20`. Pip installs it for you; the item matters only if you vendor wheels or pin a lockfile by hand.

### `dora.cuda` moved to `dora_tensor_pool` (#3249)

The CUDA IPC helpers were about to be frozen into the 1.0 Python API even though nothing in dora imports them — their only consumers are CUDA examples that CI cannot run. They now live in the tensor-pool extension, behind the same seam as the rest of that transport.

```python
# 0.x
from dora.cuda import torch_to_ipc_buffer, ipc_buffer_to_ipc_handle, open_ipc_handle

# 1.0 — pip install libraries/extensions/tensor-pool/python
from dora_tensor_pool import torch_to_ipc_buffer, ipc_buffer_to_ipc_handle, open_ipc_handle
```

The function names, signatures and `IpcHandle` type are unchanged — only the import path moves. `get_tensor_info` / `tensor_from_info` are exported from the same package.

**These helpers are not part of the 1.0 API** and carry no compatibility guarantee. That is deliberate: removing a documented symbol after 1.0 would need a major bump, whereas adding one back in 1.1 is additive.

### Everything else in the `dora` package is additive

Comparing the type stubs at `v0.x-final` against 1.0, `Node` and the ROS2 classes gained about 35 methods between them. No method was removed and none lost a parameter:

- Logging: `log`, `log_trace`, `log_debug`, `log_info`, `log_warn`, `log_error`
- Restart awareness: `is_restart`, `restart_count`
- Runtime parameters: `get_parameter`, `set_parameter`, `has_parameter`, `list_parameters`
- Service pattern: `send_response`, `take_request`
- Action pattern: `send_goal`, `take_goal`, `send_feedback`, `take_feedback`, `send_result`, `take_result`, `cancel`, `take_cancel`
- Tensor pool: `register_tensor_pool`, `read_tensor_pool`, `write_tensor_pool`, `free_tensor_pool`
- Extension seam: `extension_store`, `extension_load`, `extension_drop`, `drain_dropped_extension_keys`
- ROS2 services and actions: `Ros2ServiceClient` / `Ros2ServiceServer` / `Ros2ActionClient` / `Ros2ActionServer` and their `create_*` factories
- `Node.timestamp()`

`Ros2Context.__init__` gained an optional `domain_id`; omit it and the DDS domain comes from `ROS_DOMAIN_ID`, defaulting to 0. A `ROS_DOMAIN_ID` that is set but not a valid domain id now raises rather than being ignored.

`DataflowBuilder.Node.add_input` gained an optional `queue_policy` (`"drop_oldest"` or `"backpressure"`) and now validates both it and `queue_size` eagerly, raising `ValueError` instead of emitting YAML the daemon will reject later.

`from dora import start_runtime` still works. It is additionally exported by the separate `dora-rs-cli` wheel, which is where the `dora` command itself now ships; `dora/__init__.py` imports it defensively so `from dora import Node` works without the CLI installed.

## C API changes

**No call signature changed.** Every function in `apis/c/node/node_api.h` and `apis/c/operator/operator_types.h` has the same name, parameters and return type it had in 0.x. A 0.x C node recompiles against 1.0 unchanged.

What is new:

- **Documented threading contract.** Each function is annotated with whether it is safe to call concurrently. The short version: a context pointer must be used by at most one thread at a time (`dora_next_event`, `dora_send_output`, `dora_log` all mutate it), event pointers are read-only after creation so multiple threads may read fields from one event, and `free_dora_context` / `free_dora_event` take ownership.
- **CMake package config.** `apis/c/node/cmake/dora-api-config.cmake.in` and `dora-api-version.cmake.in` let you `find_package(dora-api)` instead of hand-writing include and link paths.
- **A null-data guard on `dora_send_operator_output`.** Passing `(NULL, 0)` for a zero-length message is now handled explicitly, matching `dora_send_output` in the node API, which had the check since introduction.

## C++ API changes

Unlike C, the C++ node bindings did lose surface. The operator bindings (`dora-operator-api-cxx`) are purely additive — `on_stop`, `on_input_closed` and `on_input_parse_error` were added and nothing was removed.

### Node initialization takes no id

```cpp
// 0.x
DoraNode init_dora_node();
DoraNode init_dora_node_from_id(rust::String node_id);
DoraNode init_dora_node_flexible(rust::String node_id);

// 1.0
DoraNode init_dora_node();
```

Only the env-var form survives, so a C++ node is always launched by the daemon and reads its identity from `DORA_NODE_CONFIG`. Dynamic C++ nodes that connected by explicit id have no 1.0 equivalent; the Rust API still has `DoraNode::init_from_node_id` and `init_flexible`, so a thin Rust shim is the workaround if you need one.

### The `DataSampleHandle` zero-copy send API is gone

```cpp
// 0.x — allocate, fill in place, hand the buffer over
auto sample = allocate_data_sample(dora_node.send_output, len);
uint8_t *ptr = data_sample_as_ptr(sample);
std::memcpy(ptr, payload, data_sample_len(sample));
send_data_sample(dora_node.send_output, "out", std::move(sample));

// 1.0 — send the bytes directly
send_output(dora_node.send_output, "out", rust::Slice<const uint8_t>(payload, len));
```

`allocate_data_sample`, `data_sample_as_ptr`, `data_sample_len`, `send_data_sample`, `send_data_sample_with_metadata` and the `DataSampleHandle` type are all absent from 1.0. `send_output` and `send_output_with_metadata` are unchanged and cover the same cases at the cost of one copy. The zero-copy path is still available from Rust via `allocate_data_sample` + `send_output_sample`.

### `node_id()` and `dataflow_id()` are gone

```cpp
// 0.x
auto id    = node_id(dora_node.send_output);
auto df_id = dataflow_id(dora_node.send_output);
```

There is no direct 1.0 replacement. `node_config_json(dora_node.send_output)` returns the node's `NodeRunConfig` (its inputs and outputs) as JSON and `dataflow_descriptor_json(...)` returns the whole descriptor, but neither yields the plain id string. A C++ node knows its own id from the YAML that launched it; read the `DORA_NODE_CONFIG` environment variable if you need it at runtime.

### Additions worth knowing

`try_next_event` and `next_event_timeout` (non-blocking and timed receive), `drain_events`, `event_as_input_with_metadata`, `event_as_node_failed`, `close_outputs`, the service and action helpers (`send_service_request`, `recv_service_response`, `send_service_response`, `recv_action_result`), and correlation-id accessors on `Metadata` (`request_id`, `goal_id`, `goal_status` and their setters). See [`api-cxx.md`](api-cxx.md).

## Still to do (this guide)

- Rust and Python **operator** API deltas — the node APIs are covered above; the operator surfaces are additive as far as the stub and bridge comparisons show, but have not been walked line by line
- CLI flag-level deltas beyond the two removed subcommands — [`cli-command-coverage.md`](cli-command-coverage.md) is the current inventory
- ROS2 bridge surface, which is explicitly unstable and outside the 1.0 guarantee
- ~~Per-API before/after code snippets~~ — done
- ~~Dedicated section on the C/C++ API~~ — done
- ~~Dedicated section on the Python API~~ — done
- ~~`dora migrate` subcommand usage~~ — **dropped per #297 resolution**. No migration tool ships in 1.0; manual steps above + release-note hard-break callout only.

## Related documents

- [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md) — the full consolidation plan
- [`api-rust.md`](api-rust.md) — Rust API reference, Arrow version policy, and the 1.0 stability scope
- [`api-python.md`](api-python.md) — Python API reference
- [`api-c.md`](api-c.md) / [`api-cxx.md`](api-cxx.md) — C and C++ API references
- [`websocket-topic-data-channel.md`](websocket-topic-data-channel.md) — topic-data frame layout and the `protocol_version` handshake
- [`phase--1-audit-2026-04-16.md`](phase--1-audit-2026-04-16.md) — wire-protocol audit evidence
- [`audit-2026-03-21-closure.md`](audit-2026-03-21-closure.md) — security/correctness closure
- [`ownership-verification-2026-04-16.md`](ownership-verification-2026-04-16.md) — publish-path readiness
- [`downstream-user-assessment-2026-04-16.md`](downstream-user-assessment-2026-04-16.md) — who this matters for
