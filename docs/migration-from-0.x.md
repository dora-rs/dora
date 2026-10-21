# Migrating from dora 0.x to 1.0

dora 1.0 does not interoperate with 0.x. The binary encoding changed and the control plane was rebuilt, so a 0.x node cannot talk to a 1.0 daemon and a 0.x CLI cannot talk to a 1.0 coordinator. There is no compatibility mode and no rolling upgrade: stop every component, upgrade all of them, start again.

This guide covers everything that changed on the way to 1.0, with a before/after example wherever code has to change. The [table](#breaking-changes-at-a-glance) is the checklist, the [upgrade steps](#upgrade-steps) are the order to do things in, and the sections after them explain each row.

## Which 0.x?

"0.x" covers three kinds of installation, and not every section applies to each of them.

- **Upstream dora 0.x**: the `dora-rs/dora` releases up to 0.5.0, that is `dora-node-api` 0.x on crates.io and `dora-rs` 0.x on PyPI. That tree is preserved under the `v0.x-final` tag, and "removed in 1.0" below means present there and absent on `main`. Every section applies. The persistent coordinator store and `dora record` did not exist yet, so for these installations the store and recording items describe new behaviour rather than stale files.
- **Fork-era builds**: the `dora-rs/adora` tree that 1.0 was consolidated from (see [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md)), including the `adora-rs` PyPI package. These already had the WebSocket control plane, a persistent coordinator store and `.adorec` recordings, so for them the wire-format section is mostly about files written by the old build.
- **1.0 release candidates**, rc.1 to rc.5. Nearly everything moved between rc.4 and rc.5. See [Coming from a release candidate](#coming-from-a-release-candidate).

## Breaking changes at a glance

| Surface | 0.x | 1.0 | Details |
|---|---|---|---|
| Binary wire encoding | bincode | postcard; a 0.x peer fails on its first frame ([#3153](https://github.com/dora-rs/dora/pull/3153)) | [Wire format](#wire-format-a-0x-peer-cannot-talk-to-a-10-peer) |
| Control plane | tarpc over TCP on 6012 + 6013, daemons on 53290, bound to `0.0.0.0` | one WebSocket port, 6013, bound to loopback by default | [Network and deployment](#network-and-deployment) |
| CLI ↔ coordinator version check | `get_version()` RPC comparing the message-format version | `Hello` frame comparing crate versions, semver-caret | [Network and deployment](#network-and-deployment) |
| Authentication | none (upstream); `?token=` query parameter (fork builds) | opt-in bearer token, `dora up --auth` | [Network and deployment](#network-and-deployment) |
| Coordinator state | in memory | persisted to `~/.dora/coordinator.redb` by default | [The coordinator store](#the-coordinator-store) |
| HLC timestamp in JSON payloads | `id` is a `u128` number | `id` is a 16-byte array (uhlc 0.9, [#3016](https://github.com/dora-rs/dora/pull/3016)); binary plane unchanged | [uhlc 0.5 → 0.9](#uhlc-05--09-3016-the-json-plane-only) |
| Topic data channel | no encoding version | `protocol_version` in both directions; mismatch refused ([#3160](https://github.com/dora-rs/dora/pull/3160)) | [Topic subscriptions](#topic-subscriptions-are-refused-on-encoding-mismatch-3160) |
| Recordings | `.adorec`, format v1 (fork builds) | `.drec`, format v2; v1 is refused, not converted | [Recordings](#0x-recordings-do-not-replay) |
| `dora version` | queried the coordinator | removed; `dora --version` prints the CLI version | [CLI changes](#cli-changes) |
| Dataflow YAML | `custom:`, `_unstable_deploy`, `_unstable_debug`, `publish_all_messages_to_zenoh`, `communication:` | rejected by name ([#3158](https://github.com/dora-rs/dora/pull/3158), [#3220](https://github.com/dora-rs/dora/pull/3220)) | [Descriptor changes](#descriptor-changes-dataflow-yaml) |
| Node, input and output ids | not validated on parse | restricted character set, checked on parse | [Identifiers are validated](#identifiers-are-validated) |
| Rust toolchain | 1.85 | 1.95 ([#3017](https://github.com/dora-rs/dora/pull/3017)) | [Rust 1.95 or newer](#rust-195-or-newer-3017) |
| `dora-node-api` error type | `eyre::Result` | `NodeResult` / `NodeError` | [Rust node API](#fallible-methods-return-noderesult-not-eyreresult) |
| Arrow in `dora-node-api` | `ArrowData`, `pub use arrow` | `DoraArray`; Arrow behind `arrow-v58` / `arrow-v59` features ([#3213](https://github.com/dora-rs/dora/pull/3213)) | [Rust node API](#arrow-types-are-no-longer-in-doras-public-api-3213) |
| `send_output` data parameter | `impl Array` | `impl IntoArrow` | [Rust node API](#send_output-takes-intoarrow) |
| Wire-protocol enums in `dora-message` | exhaustively matchable | `#[non_exhaustive]`, a `_ =>` arm is required ([#3151](https://github.com/dora-rs/dora/pull/3151)) | [Rust node API](#wire-protocol-enums-are-non_exhaustive-3151) |
| Descriptor structs in `dora-message` | struct-literal constructible | `#[non_exhaustive]`; construct with `Node::new`, `Descriptor::new`, `Default::default()` ([#3387](https://github.com/dora-rs/dora/pull/3387)) | [Rust node API](#descriptor-structs-are-non_exhaustive-3387) |
| `dora-operator-api` | `Event::Input { id, data }`, `send(String, impl Array)` | `Event::Input { id, metadata, data }`, `send(&str, impl IntoArrow)` | [Rust operator API](#rust-operator-api-changes) |
| Python | CPython 3.8+; `dora.cuda` | CPython 3.11+; `dora_tensor_pool` ([#3249](https://github.com/dora-rs/dora/pull/3249)) | [Python API](#python-api-changes) |
| C++ node API | `init_dora_node_from_id`, `DataSampleHandle`, `node_id()`, `dataflow_id()` | removed | [C++ API](#c-api-dora-node-api-cxx) |
| C++ operators | `new_operator` + `on_input` | also `on_input_closed`, `on_stop`, `on_input_parse_error`; link error without them | [C++ operators](#c-operators-must-implement-three-more-callbacks) |
| Crates | `dora-runtime`, `communication-layer-request-reply`, PyO3 crates on crates.io | split, removed, unpublished | [Crates](#crates-that-moved-or-left-cratesio) |

## Upgrade steps

1. **Stop everything.** `dora down` on every machine (`dora destroy` still works as an alias). 1.0 and 0.x daemons cannot share a coordinator, so there is no order in which a rolling upgrade works.
2. **Update the toolchains.** Rust 1.95 or newer for anything built from source; CPython 3.11 or newer for Python nodes.
3. **Update dependencies.** Bump `dora-node-api` and `dora-operator-api` to `1` in `Cargo.toml`, then `cargo install dora-cli --locked` or `pip install -U dora-rs dora-rs-cli`. If you installed the fork-era `adora-rs` package, uninstall it; it is not republished, so it will never resolve to 1.0. Some crates were split, removed or unpublished; see [Crates](#crates-that-moved-or-left-cratesio).
4. **Rebuild every node and operator** against the 1.0 crates, following the API sections below. Do not run 0.x binaries against 1.0 daemons.
5. **Update the YAML.** See [Descriptor changes](#descriptor-changes-dataflow-yaml); several 0.x keys are rejected by name rather than ignored.
6. **Check the network.** The control plane is one WebSocket port, 6013, and the coordinator binds loopback unless told otherwise; see [Network and deployment](#network-and-deployment).
7. **Re-record recordings.** A v1 capture is refused, not converted; see [0.x recordings do not replay](#0x-recordings-do-not-replay).
8. **Update scripts** that pipe `*.adorec` into `dora replay`, call `dora version`, pass a node name to `dora logs` positionally, or hard-code port 6012 or 53290.
9. **Start again.** `dora up`, or `dora up --recreate-store` if a fork-era or rc build left a coordinator store on the machine (see [The coordinator store](#the-coordinator-store)); then `dora validate <dataflow>.yml` and `dora start`.

## Wire format: a 0.x peer cannot talk to a 1.0 peer

A 0.x node pointed at a 1.0 daemon does not negotiate down, it fails on the first frame. A 0.x daemon and a 0.x CLI fail as well, at the control plane rather than on the encoding; see [Network and deployment](#network-and-deployment).

### bincode → postcard ([#3153](https://github.com/dora-rs/dora/pull/3153))

Four format versions moved with the encoding, each failing loudly rather than misparsing: both encodings are positional, so a tolerated mismatch would surface as corrupt data instead of an error.

| Surface | 0.x | 1.0 | What a stale peer or file gets |
|---|---|---|---|
| `Metadata::CURRENT_VERSION` (node ↔ daemon; also checked daemon ↔ coordinator) | 0 | 2 | decode failure at register, see below |
| Coordinator store `SCHEMA_VERSION` (redb) | none (upstream); older (fork builds) | 5 | coordinator refuses to open the database |
| `.drec` `FORMAT_VERSION` | 1 | 2 | `dora replay` refuses the file |
| Topic data channel `protocol_version` | absent | 2 | subscription refused at handshake ([#3160](https://github.com/dora-rs/dora/pull/3160)) |

Release candidates wrote other values; see [Coming from a release candidate](#coming-from-a-release-candidate).

bincode is unmaintained ([RUSTSEC-2025-0141](https://rustsec.org/advisories/RUSTSEC-2025-0141.html)): development stopped at 1.3.3 and every version is flagged, so it was not an encoding to commit to for the life of 1.x. postcard is serde-based, so no message *type* changed, only the bytes, and unlike bincode it has a documented, stable wire specification, which is what matters once the format is frozen.

### What a stale peer sees

A 0.x node against a 1.0 daemon fails with a low-level deserialization error, not with `message wire-format mismatch: node speaks metadata format v0`. The `metadata_version` field in `NodeRegisterRequest` catches layout drift within one encoding, but the frame that carries it is itself encoded in the new format, so decoding fails before the check runs.

The daemon ↔ coordinator link is JSON, so the same field in `DaemonRegisterRequest` does produce that legible message there. An upstream 0.x daemon never gets that far: it dials port 53290, where nothing listens.

A 0.x CLI against a 1.0 coordinator connects to port 6013, where the tarpc `get_version` call it makes first is answered by a WebSocket server. The 0.x CLI reports that as `Failed to query coordinator version. The coordinator may be running an older version of dora that is incompatible with this CLI`, which blames the wrong side; the coordinator is newer, not older.

### The coordinator store

The coordinator keeps its state (dataflows, their results, `dora param` values) in a redb file so that it survives a restart. `redb` is the default `--store` backend and `dora up` uses it, so if an earlier build ran on this machine there is a `~/.dora/coordinator.redb` with an older schema, and the coordinator refuses to open it:

```
redb schema version mismatch: database at `~/.dora/coordinator.redb` has v3,
but this binary expects v5. Delete the file and restart to create a fresh
database, or use `--store memory` to bypass persistence.
```

Run `dora up --recreate-store`: it moves the old store aside as `coordinator.redb.backup` (numbered if one already exists) and starts fresh. `dora up` prints this hint under the error; `dora coordinator` run directly shows only the message above. The flag handles the default store path only, so a custom `--store redb:<path>` needs a manual move, and it is refused while `DORA_COORDINATOR_ADDR` points at a non-loopback address, so run it on the coordinator's machine with that variable unset. Running dataflows are re-registered by their daemons when they reconnect; persisted `dora param` values and the results of past runs stay in the `.backup` file only. Only `--store memory` deployments have nothing to do here.

Which schema an old file has depends on the build that wrote it (the [release candidate table](#coming-from-a-release-candidate) lists them); the remedy is the same for all. Upstream 0.x had no store, so a machine that only ever ran upstream 0.x has no file and no error.

### uhlc 0.5 → 0.9 ([#3016](https://github.com/dora-rs/dora/pull/3016)): the JSON plane only

**This bump did not change the binary plane.** uhlc 0.9 changes `ID`'s in-memory representation from `NonZeroU128` to `[u8; 16]`, but the two serialize identically; `libraries/message/tests/uhlc_wire_format.rs` pins the encoding and matches byte for byte under 0.5.2 and 0.9.0. Daemon ↔ node messages, inter-daemon Zenoh samples and `.drec` entries were untouched.

What does change shape is the JSON plane, and this reaches you only if you have your own tooling parsing the CLI ↔ coordinator or coordinator ↔ daemon WebSocket payloads. A timestamp's `id` goes from a bare number to a 16-element byte array:

```json
// 0.x
{"timestamp": {"time": 7346545054874578944, "id": 133075017253481751908959400507149664154}}

// 1.0
{"timestamp": {"time": 7346545054874578944,
               "id": [154, 63, 12, 113, 212, 40, 78, 182, 21, 195, 135, 42, 233, 80, 29, 100]}}
```

Both links are gated by the `dora_version` handshake, so a mismatch fails at parse time with a serde type error rather than being misread, and nothing persists a uhlc timestamp as JSON on disk.

The new shape is easier to consume. A real HLC id uses all 16 bytes, so under 0.5 the `u128` form exceeded every number `serde_json::Value` can hold (i64, u64, f64) and `serde_json::to_value` failed with "number out of range" on every timestamp. The `[u8; 16]` form round-trips through `serde_json::Value` losslessly; if you were working around that limit with a bignum-aware parser or raw string handling, you no longer need to.

### Topic subscriptions are refused on encoding mismatch ([#3160](https://github.com/dora-rs/dora/pull/3160))

**If you wrote your own topic subscriber**, send `protocol_version: dora_message::TOPIC_DATA_PROTOCOL_VERSION` (currently `2`) in `TopicSubscribe` and check the value echoed in `TopicSubscribed` before consuming frames. It is checked separately from the `Hello` version handshake, which third-party subscribers never send, so keep sending it even against a same-version coordinator.

The WebSocket topic data channel carries raw `Timestamped<InterDaemonEvent>` bytes to third-party subscribers, with no envelope and no self-describing encoding, so a subscriber speaking a different binary format misparses those frames rather than failing to decode them. Before 1.0 the channel had no version exchange at all. Now both directions carry `protocol_version`:

```json
{"TopicSubscribe":  {"dataflow_id": "...", "topics": [...], "protocol_version": 2}}
{"TopicSubscribed": {"subscription_id": "...", "protocol_version": 2}}
```

A peer predating the handshake omits the field and is rejected the same way a wrong number would be; that is the case an upgrader hits, since `2` is the only value the constant has ever had. The rejection names both sides so you can tell which one is old:

```
topic data protocol mismatch: client predates the topic data protocol handshake
(bincode-era frames), this side speaks 2. Binary frames are positionally encoded,
so subscribing would silently misparse rather than fail. Upgrade whichever side is older.
```

The full frame layout is in [`websocket-topic-data-channel.md`](websocket-topic-data-channel.md).

### 0.x recordings do not replay

`.drec` entries are `Timestamped<InterDaemonEvent>` payloads, so they moved encoding with everything else. The container framing and the magic bytes (`DORAREC\x00` / `DORAEND\x00`) are unchanged, so a v1 file is refused by version rather than reported as corrupt:

```
recording format version 1 is no longer supported (min supported: 2); it was
written by a dora release that encoded events with bincode. Re-record with this
version of dora.
```

There is no converter, and renaming a fork-era `.adorec` to `.drec` does not produce a readable file. If a capture matters, keep the build that wrote it around to replay it; otherwise re-record under 1.0.

## Network and deployment

**One port instead of three.** 0.x ran a tarpc control server on 6012, its RPC channel on 6013 (`control_port + 1`), and a separate TCP listener for daemons on 53290. 1.0 serves the CLI, the daemons and the topic data channel over a single WebSocket port, 6013. The daemon's local listen port (53291) and the Zenoh peer port (5456) are unchanged. `--coordinator-port` defaults to 6013 on every command that has one, `dora daemon` included. Firewall rules for 6012 and 53290 can go.

**The coordinator binds loopback by default.** 0.x `dora coordinator` bound `0.0.0.0`. In 1.0 both `dora coordinator` and `dora up` bind `127.0.0.1` unless you pass `--interface <ip>` or set `DORA_COORDINATOR_INTERFACE`, so a distributed setup that relied on the old default now sees its daemons time out on connect. This is the bind address; `DORA_COORDINATOR_ADDR` is the address the CLI and daemons connect to.

**Every coordinator-connected command reads `DORA_COORDINATOR_ADDR` and `DORA_COORDINATOR_PORT`.** In 0.x the address was a per-command flag only; those flags still exist.

**The version check moved from an RPC to the handshake.** 0.x called `get_version()` on every command and compared `dora_message::VERSION`, the message-format version. 1.0 sends `ControlRequest::Hello` with the CLI's crate version before anything else, and daemons carry theirs in `DaemonRegisterRequest`; the coordinator refuses a semver-incompatible peer with a message naming both versions. Within 1.x the check is caret-compatible, so a 1.0 CLI can drive a 1.3 coordinator.

**Authentication is opt-in and header-based.** `dora up --auth` (or `dora coordinator --auth`) generates a token, writes it to `.dora-token` in the working directory and under the user config directory, and the CLI and daemons send it as `Authorization: Bearer <token>`. Upstream 0.x had no authentication. Fork-era builds sent the token as a `?token=` query parameter; the coordinator still parses that parameter but ignores it, so a fork-era client cannot authenticate against a 1.0 coordinator that has auth enabled.

**Coordinator state survives restarts.** With the default `redb` store, `dora list` keeps showing finished dataflows until `dora clean` removes them. 0.x forgot everything on restart; `dora coordinator --store memory` restores that behaviour. The schema-mismatch error an old store produces is covered under [The coordinator store](#the-coordinator-store).

[`distributed-deployment.md`](distributed-deployment.md) covers multi-machine setups, including `dora cluster`.

## CLI changes

- `dora version` is removed. `dora --version` prints the CLI's own version; the coordinator's is checked automatically on every connection.
- `dora destroy` and `dora check` keep working as aliases of `dora down` and `dora status`, which are the documented names.
- `dora logs <dataflow> --node <name>` replaces the 0.x positional `dora logs <dataflow> <node>`, which is rejected with a hint pointing at `--node`. `--all-nodes`, `--since`, `--until`, `--grep`, `--level` and `--log-format` are new.
- `dora stop <name-or-uuid>` accepts a dataflow name positionally; `--name` still works.
- `dora start` attaches by default in both versions; `--detach` runs the dataflow in the background, as before.
- `dora replay` reads `.drec` files. Shell completions and aliases that mention `*.adorec` need updating.
- New in 1.0: `cluster`, `restart`, `clean`, `param`, `record`, `replay`, `trace`, `doctor`, `expand`, `validate` and `hub`; `top` is a top-level shortcut for the existing `inspect top`. `daemon`, `runtime` and `coordinator` still exist but are hidden from `--help`; `dora up` and `dora run` are the intended entry points.
- `pip install dora-rs-cli` installs the `dora` command, as it did in 0.x. The `dora-rs` wheel alone does not include it.

[`cli.md`](cli.md) is the full reference.

## Descriptor changes (dataflow YAML)

All of these are rejected with an error naming the offending field rather than silently ignored:

```
unknown field `custom`, expected one of `id`, `name`, `description`, `path`, …
```

`Descriptor` and `Node` use `deny_unknown_fields`, as they did in 0.x, which matters here: a silently dropped `deploy` block would run every node on the local daemon while the YAML looks like it pinned them to machines. `Debug` is newly strict, so an unknown key inside `debug:` that 0.x ignored is an error too.

### `custom:` removed ([#3158](https://github.com/dora-rs/dora/pull/3158))

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

`custom.envs` has no direct replacement; use the node-level `env:` map.

### `_unstable_` prefix dropped, `publish_all_messages_to_zenoh` renamed ([#3220](https://github.com/dora-rs/dora/pull/3220), [#3158](https://github.com/dora-rs/dora/pull/3158))

Both the dataflow-level and the per-node `deploy` key lose the prefix, and the one key inside `debug` was renamed at the same time.

```yaml
# 0.x
_unstable_deploy:
  machine: m1
_unstable_debug:
  publish_all_messages_to_zenoh: true

# 1.0
deploy:
  machine: m1
debug:
  enable_debug_inspection: true
```

### `communication:` block removed ([#3158](https://github.com/dora-rs/dora/pull/3158))

Delete it. `_unstable_remote` only ever had one value, `tcp`. `_unstable_local` could select `unixdomain` in 0.5.0 and earlier; that transport was removed before `v0.x-final` and the daemon ↔ node channel in 1.0 is TCP, so a dataflow that set it loses the option rather than a behaviour it can keep.

### Identifiers are validated

0.x did not validate identifiers when parsing a descriptor. 1.0 checks every node, input and output id:

- **Node ids** may contain only `[a-zA-Z0-9_.-]`, must not be empty, must not start with `.` (they are joined into filesystem paths such as `.dora/python-envs/<id>`), and must not be exactly `dora`, which is reserved for the built-in `dora/timer/...` and `dora/logs` inputs.
- **Output and input ids** may contain only `[a-zA-Z0-9_./-]`, with no empty segment around a `/`.

A node id with a space or an `@`, which 0.x ran, is now a parse error. In Rust, `NodeId::from(String)` and `.into()` panic on an invalid id; use `parse::<NodeId>()` where the id comes from input.

[`yaml-spec.md`](yaml-spec.md) documents the full 1.0 schema, including the keys that are new (`module:`, `hub:`, `type_rules:`, restart tuning, log routing, `cpu_affinity`, and others). Those are additive and need no migration.

## Rust node API changes

### Rust 1.95 or newer ([#3017](https://github.com/dora-rs/dora/pull/3017))

The workspace `rust-version` moved from 1.85 (0.5.0) to 1.95, for redb 4 and sysinfo 0.39. This only matters if you build dora, or a crate that depends on it, from source; the published binaries and wheels are unaffected.

### Fallible methods return `NodeResult`, not `eyre::Result`

Most code does not have to change: `NodeError` implements `std::error::Error + Send + Sync + 'static`, so `?` still converts into `eyre::Result`, `anyhow::Result` and `Box<dyn Error>`. What breaks is code that *names* the type: an explicit `let r: eyre::Result<()> = node.send_output(...)`, a function whose signature promised `eyre::Report`, or a `match` on the error.

Every public `DoraNode` and `EventStream` method changed its error type from `eyre::Report` to the concrete [`NodeError`](../apis/rust/node/src/error.rs) enum, aliased as `NodeResult<T>`:

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

`NodeError` has four variants: `Init`, `Output`, `Data` and `Internal(eyre::Report)`. The pattern-aware receive helpers (`recv_service_response`, `recv_action_result`) return `PatternError` instead, with `Timeout`, `ServerRestarted`, `StreamEnded` and `StreamError`.

### Arrow types are no longer in dora's public API ([#3213](https://github.com/dora-rs/dora/pull/3213))

Most nodes need no change here: `value.into_arrow()` and `(&data).try_into()?` never name an Arrow type, which is why the `dora new` templates needed no Arrow-related change. If you do name one, read on.

`dora-node-api` no longer names an Arrow type, so dora can move to a newer Arrow major in a minor release without breaking your build. The policy is in [`api-rust.md`](api-rust.md).

**`ArrowData` is now `DoraArray`**, with a private field rather than `pub ArrayRef`, and no `Deref` to `ArrayRef`:

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

**Arrow itself is behind a feature.** `pub use arrow;` became `arrow_v59`, enabled by the `arrow-v59` feature:

```toml
# 1.0, only if you name Arrow types directly
dora-node-api = { version = "1", features = ["arrow-v59"] }
```

```rust
// 0.x                              // 1.0
use dora_node_api::arrow;           use dora_node_api::arrow_v59 as arrow;
```

If your own crate is pinned to Arrow 58 and you are not ready to move, use `arrow-v58` instead; it converts through the Arrow C Data Interface rather than borrowing. The support window is documented in [`api-rust.md`](api-rust.md).

### `send_output` takes `IntoArrow`

The data parameter of `send_output` changed from `impl arrow::array::Array` to `impl IntoArrow`. Primitive values, `Vec<T>`, strings and `DoraArray` implement it out of the box. Arrow arrays implement it too, but only with the `arrow-v59` feature, because those impls have to name an Arrow major:

```rust
// 0.x
node.send_output(id, params, UInt32Array::from(vec![1, 2, 3]))?;

// 1.0, no Arrow type named, no feature needed
node.send_output(id, params, vec![1u32, 2, 3])?;

// 1.0, passing an Arrow array still works with `features = ["arrow-v59"]`
node.send_output(id, params, UInt32Array::from(vec![1, 2, 3]))?;
```

### `dora_core` re-export narrowed ([#3221](https://github.com/dora-rs/dora/pull/3221))

`dora_node_api::dora_core` now exposes only `config::{DataId, NodeId, OperatorId}` and `uhlc`. The scaffolded path is unchanged:

```rust
use dora_node_api::dora_core::config::DataId;   // still works
```

If you reached `dora_node_api::dora_core::{descriptor, topics, build, manifest, types}`, depend on `dora-core` directly instead, keeping in mind that it is an internal crate outside the 1.0 guarantee (see "Stability scope at 1.0" in [`api-rust.md`](api-rust.md)).

### `flume` is no longer re-exported ([#3237](https://github.com/dora-rs/dora/pull/3237))

`pub use flume;` and `pub use flume::Receiver;` were pure convenience; no flume type appears in any public signature. Removing them takes flume out of the frozen 1.0 surface, so a future flume major is not a dora breaking change.

```rust
// 0.x
use dora_node_api::flume::{self, Receiver};

// 1.0, add flume to your own Cargo.toml
use flume::{self, Receiver};
```

The same reasoning does **not** apply to `uuid`: `pub type DataflowId = uuid::Uuid`, so uuid stays in the frozen surface and is still re-exported as `dora_node_api::uuid`.

### `integration_testing` no longer exposes tokio ([#3239](https://github.com/dora-rs/dora/pull/3239))

The module re-exported three tokio items, which would have frozen a tokio major into the 1.0 guarantee. They are now newtypes owned by dora, so tests need no channel dependency of their own.

```rust
// 0.x                                    // 1.0
unbounded_channel()                       output_channel()
UnboundedSender<OutputJson>               OutputSender
UnboundedReceiver<OutputJson>             OutputReceiver
```

The module is not feature-gated, so no Cargo change is needed.

### `ArrowTypeInfo` left the send path ([#2366](https://github.com/dora-rs/dora/pull/2366))

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

`send_output_raw` and `send_output_bytes` are otherwise unchanged apart from the `NodeResult` return type.

**`arrow_utils` is now the IPC codec.** The 0.x helpers `copy_array_into_sample`, `required_data_size` and `buffer_into_arrow_array` are gone; the module now offers `encode_arrow_ipc`, `decode_arrow_ipc` and `decode_arrow_ipc_zero_copy`. If you were hand-rolling the buffer layout, call `send_output` or `DoraNode::encode_arrow` instead.

### Wire-protocol enums are `#[non_exhaustive]` ([#3151](https://github.com/dora-rs/dora/pull/3151))

`DaemonRequest`, `DaemonReply`, `InterDaemonEvent`, `CoordinatorRequest`, `DaemonEvent` and (since [#3364](https://github.com/dora-rs/dora/pull/3364)) `RegisterResult` on the published `dora-message` crate now carry `#[non_exhaustive]`, so future variant additions are not semver-major. Exhaustive matches no longer compile.

```rust
// 0.x
match request {
    DaemonRequest::Subscribe => { ... }
    DaemonRequest::SendMessage { .. } => { ... }
    // ...every variant listed
}

// 1.0, add a fallback arm
match request {
    DaemonRequest::Subscribe => { ... }
    DaemonRequest::SendMessage { .. } => { ... }
    _ => { /* unknown request from a newer peer */ }
}
```

In the fallback arm, reply with an explicit error on a request you were expected to answer, and skip on read-only or observability paths.

`dora_node_api::Event` was already `#[non_exhaustive]` in 0.x, so the four new variants (`InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted`) do not break existing matches.

### Descriptor structs are `#[non_exhaustive]` ([#3387](https://github.com/dora-rs/dora/pull/3387))

`Descriptor`, `Node`, `CustomNode`, `ResolvedNode`, `Deploy`, `Debug`, `TypeRuleDef`, `OperatorConfig`, `SingleOperatorDefinition` and `NodeRunConfig` on the published `dora-message` crate now carry `#[non_exhaustive]`, so adding a descriptor key later is a minor release rather than a major one. Struct literals no longer compile from outside the crate, and `..Default::default()` is not an escape hatch: `#[non_exhaustive]` bans functional-update syntax across crate boundaries too, for the types that implement `Default` (`Deploy`, `Debug`, `NodeRunConfig`; `Node`, `Descriptor`, `CustomNode` and `ResolvedNode` never did, so in 0.x they were always spelled out field by field).

Every field stays `pub`, so the migration is to construct from the entry point and then assign:

```rust
// 0.x, a struct literal naming every field
let node = Node {
    id: "camera".parse()?,
    path: Some("./camera".to_owned()),
    name: None,
    description: None,
    // … and every other field, explicitly
};

// 1.0, construct, then assign
let mut node = Node::new("camera".parse()?);
node.path = Some("./camera".to_owned());
```

| Type | Entry point |
|---|---|
| `Descriptor` | `Descriptor::new(nodes)` |
| `Node` | `Node::new(id)` |
| `CustomNode` | `CustomNode::new(path)` |
| `ResolvedNode` | `ResolvedNode::new(id, kind)` |
| `Deploy`, `Debug`, `NodeRunConfig` | `Default::default()` |
| `TypeRuleDef` | `TypeRuleDef::new(from, to)` |
| `OperatorConfig`, `SingleOperatorDefinition` | deserialization only |

Build the `NodeId` with `parse()` rather than `.into()`, which panics on an invalid id (see [Identifiers are validated](#identifiers-are-validated)); this matters on the `AddNode` path, where the id comes from a request.

This affects Rust code that builds descriptors programmatically; the dynamic-topology `AddNode` surface is the common case. Reading and mutating an existing descriptor is unchanged, and there is no YAML, JSON-schema or wire-format change. The descriptor *enums* (`RestartPolicy`, `NodeSource`, `EnvValue`, `OutputFraming`, …) are deliberately not marked, so matches on them still compile unchanged; a new variant there stays a major change (the note on `RestartPolicy` in `descriptor.rs` explains the trade).

### Pool and pinned-memory requests replaced by the extension seam ([#3219](https://github.com/dora-rs/dora/pull/3219))

Naming a protocol after one transport would freeze that transport's architecture into the 1.0 surface. Three groups of variants collapsed onto opaque carriers:

```rust
InterDaemonEvent::ExtensionMessage { dataflow_id, namespace, target_machine, payload }
DaemonRequest::ExtensionRequest    { namespace, payload }
DaemonReply::ExtensionReply        { payload }
```

Also removed: `DaemonRequest::RegisterPinnedMemory`, `DaemonRequest::ReadPinnedMemory`, `DaemonReply::PinnedMemoryMetadata`, and the two `DoraNode` methods that fronted them. Neither ever had a working implementation, so nothing can be relying on them.

## Rust operator API changes

`dora-operator-api` ships outside the 1.0 guarantee (see "Stability scope at 1.0" in [`api-rust.md`](api-rust.md)), but a 0.x operator still has to be ported. Python and C operators are unchanged; C++ operators have their own [section](#c-operators-must-implement-three-more-callbacks).

**`Event::Input` gained a `metadata` field**, and `data` is a `DoraArray` rather than `ArrowData`. `Event` was already `#[non_exhaustive]`, but that attribute does not cover the fields inside a variant, so a pattern that names `id` and `data` without `..` no longer compiles:

```rust
// 0.x
Event::Input { id, data } => { ... }

// 1.0
Event::Input { id, data, .. } => { ... }
Event::Input { id, metadata, data } => { ... }   // if you want the OpenTelemetry context
```

**`DoraOutputSender::send` takes `&str` and `impl IntoArrow`**, not `String` and `impl Array`:

```rust
// 0.x
output_sender.send("count".to_owned(), UInt64Array::from(vec![n]))?;

// 1.0
output_sender.send("count", n.into_arrow())?;
```

**The re-exports narrowed**, for the same reason as on the node API. `pub use dora_arrow_convert::*` became `pub use dora_arrow_convert::{DoraArray, IntoArrow, into_vec}`, and Arrow itself sits behind the `arrow-v59` feature as `dora_operator_api::arrow_v59` (`default = []`). Passing an Arrow array to `send`, or calling `data.as_array()`, needs the feature; `n.into_arrow()` and `u32::try_from(&data)` do not.

`InputParseError`, `InputClosed` and `Stop` are unchanged, as are `DoraStatus` and the `register_operator!` macro.

## Python API changes

### CPython 3.11 is the floor (was 3.8)

`requires-python` moved from `>=3.8` to `>=3.11` and the wheels are built `abi3-py311`. If you are on 3.8 to 3.10, upgrade the interpreter first; there is no 1.0 wheel for those versions and no compatibility shim. Free-threaded builds (`python3.13t`, `python3.14t`) are not supported. The [Python version policy](api-rust.md#python-version-policy) has the full guarantee.

### `numpy` is now a hard dependency

0.x declared `pyarrow>=14.0.1` and `pyyaml>=6.0`. 1.0 adds `numpy>=1.20`. Pip installs it for you; the item matters only if you vendor wheels or pin a lockfile by hand.

### `dora.cuda` moved to `dora_tensor_pool` ([#3249](https://github.com/dora-rs/dora/pull/3249))

The CUDA IPC helpers moved out of the frozen 1.0 Python API into the tensor-pool extension, so they can keep changing after 1.0.

```python
# 0.x
from dora.cuda import torch_to_ipc_buffer, ipc_buffer_to_ipc_handle, open_ipc_handle

# 1.0, pip install libraries/extensions/tensor-pool/python
from dora_tensor_pool import torch_to_ipc_buffer, ipc_buffer_to_ipc_handle, open_ipc_handle
```

The function names, signatures and `IpcHandle` type are unchanged; only the import path moves. `get_tensor_info` / `tensor_from_info` are exported from the same package.

**These helpers are not part of the 1.0 API** and carry no compatibility guarantee: removing a documented symbol after 1.0 would need a major bump, adding one back in 1.1 would not.

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

`from dora import start_runtime` still works.

### Python operators are unchanged

A Python operator is still a class with `on_event(self, event, send_output) -> DoraStatus`, and `from dora import DoraStatus` is unchanged. The runtime forwards the same four events it did in 0.x (`INPUT`, `INPUT_CLOSED`, `STOP` and reload); the one difference in the event dict is that a reload now arrives as `type: "RELOAD"`, where 0.x labelled it `"UNKNOWN"`.

## C API (`dora-node-api-c`)

**No call signature changed.** Every function in `apis/c/node/node_api.h` and `apis/c/operator/operator_types.h` has the same name, parameters and return type it had in 0.x, so a 0.x C node or C operator recompiles against 1.0 unchanged.

What is new:

- **Documented threading contract.** Each function is annotated with whether it is safe to call concurrently. The short version: a context pointer must be used by at most one thread at a time (`dora_next_event`, `dora_send_output`, `dora_log` all mutate it), event pointers are read-only after creation so multiple threads may read fields from one event, and `free_dora_context` / `free_dora_event` take ownership.
- **CMake package config.** `apis/c/node/cmake/dora-api-config.cmake.in` and `dora-api-version.cmake.in` let you `find_package(dora-api)` instead of hand-writing include and link paths.
- **A null-data guard on `dora_send_operator_output`.** Passing `(NULL, 0)` for a zero-length message is now handled explicitly, matching `dora_send_output` in the node API.

## C++ API (`dora-node-api-cxx`)

Unlike C, the C++ node bindings did lose surface, and the C++ operator bindings gained callbacks that an existing operator has to define.

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
// 0.x: allocate, fill in place, hand the buffer over
auto sample = allocate_data_sample(dora_node.send_output, len);
uint8_t *ptr = data_sample_as_ptr(sample);
std::memcpy(ptr, payload, data_sample_len(sample));
send_data_sample(dora_node.send_output, "out", std::move(sample));

// 1.0: send the bytes directly
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

### C++ operators must implement three more callbacks

The `cxx::bridge` in `dora-operator-api-cxx` now calls `on_input_closed`, `on_stop` and `on_input_parse_error` in addition to `new_operator` and `on_input`. 0.x dropped `InputClosed` and `Stop` on the floor for C++ operators; 1.0 delivers them ([#1849](https://github.com/dora-rs/dora/pull/1849), [#1879](https://github.com/dora-rs/dora/issues/1879)). Because the functions are declared in the bridge, an operator that does not define them fails to **link**, not to compile: the linker reports an undefined reference to `on_input_closed` and the other two. To keep the 0.x behaviour, add stubs that ignore the event:

```cpp
DoraOnInputResult on_input_closed(Operator &op, rust::Str id, OutputSender &output_sender) {
    return {rust::String(), false};
}

DoraOnInputResult on_stop(Operator &op, OutputSender &output_sender) {
    return {rust::String(), false};
}

DoraOnInputResult on_input_parse_error(Operator &op, rust::Str id, rust::Str error,
                                       OutputSender &output_sender) {
    return {rust::String(), false};
}
```

Each receives the same per-event `OutputSender` as `on_input`, so an operator can also flush state or emit a final output from them. `examples/c++-dataflow/operator-rust-api/operator.cc` is a complete reference.

## Crates that moved or left crates.io

- **`dora-runtime` is split.** The operator runtime is now `dora-runtime-api` (the language-neutral SDK), `dora-runtime-shared-lib` (the C ABI backend, shipped inside the `dora` binary as `dora runtime`) and `dora-runtime-python` (the PyO3 backend, shipped inside the `dora-rs` wheel as `dora.start_runtime()`). Nothing that only ran operators through the CLI or the wheel has to change.
- **`communication-layer-request-reply` is removed.** The TCP-backed `RequestReplyLayer` trait is superseded by the service and action patterns: `send_service_request()` / `send_service_response()` on `DoraNode`, correlated through Arrow metadata. See [`patterns.md`](patterns.md).
- **The PyO3-linking crates are no longer published** ([#3303](https://github.com/dora-rs/dora/pull/3303)): `dora-node-api-python`, `dora-operator-api-python`, `dora-ros2-bridge-python` and `dora-runtime-python` reach users only as wheels. `pyo3-ffi` declares `links = "python"`, so publishing them would have pinned every downstream PyO3 extension to dora's exact pyo3 minor for the life of 1.x. If you depended on one from crates.io, build it from the tree instead; it now uses PyO3 0.29.
- **Depend on `dora-node-api = "1"` and `dora-operator-api = "1"` only.** `dora-core`, `dora-daemon`, `dora-coordinator` and the other internal crates are published only so that the public crates can build; depending on them directly is unsupported (see "Stability scope at 1.0" in [`api-rust.md`](api-rust.md)).

## Coming from a release candidate

Everything above applies to rc.1 through rc.4, since the rc window was where the 0.x surface was cut down. In particular the four format versions moved between rc.4 and rc.5, so an rc.4 deployment hits the same wire-format refusals as a 0.x one:

| | rc.1 to rc.3 | rc.4 | rc.5 and 1.0 |
|---|---|---|---|
| Binary encoding | bincode | bincode | postcard |
| `Metadata::CURRENT_VERSION` | 0 in rc.1, 1 from rc.2 | 1 | 2 |
| Coordinator store `SCHEMA_VERSION` | 2 | 3 | 5 |
| `.drec` `FORMAT_VERSION` | 1 | 1 | 2 |
| Topic data `protocol_version` | absent | absent | 2 |

The API changes merged between rc.4 (22 July) and rc.5 (27 August): Arrow out of the public API ([#3213](https://github.com/dora-rs/dora/pull/3213)), `NodeResult`, the narrowed re-exports ([#3221](https://github.com/dora-rs/dora/pull/3221), [#3237](https://github.com/dora-rs/dora/pull/3237), [#3239](https://github.com/dora-rs/dora/pull/3239)), `#[non_exhaustive]` wire enums ([#3151](https://github.com/dora-rs/dora/pull/3151)), the extension seam ([#3152](https://github.com/dora-rs/dora/pull/3152), [#3219](https://github.com/dora-rs/dora/pull/3219)), `dora.cuda` → `dora_tensor_pool` ([#3249](https://github.com/dora-rs/dora/pull/3249)), the descriptor removals ([#3158](https://github.com/dora-rs/dora/pull/3158), [#3220](https://github.com/dora-rs/dora/pull/3220)), identifier validation, MSRV 1.95 ([#3017](https://github.com/dora-rs/dora/pull/3017)), uhlc 0.9 ([#3016](https://github.com/dora-rs/dora/pull/3016)), the topic handshake ([#3160](https://github.com/dora-rs/dora/pull/3160)), and the unpublished PyO3 crates ([#3303](https://github.com/dora-rs/dora/pull/3303)). One descriptor rule is rc-only because `module:` nodes did not exist in 0.x: a module node may carry only `module`, `inputs`, `params`, `build` and the fields every node shares (`env`, `deploy`, and so on). Source and runtime fields (`path`, `args`, `git`, `hub`, `operators`, `outputs`, …) are rejected, where earlier rcs accepted and silently discarded them ([#2911](https://github.com/dora-rs/dora/pull/2911), completed by [#3281](https://github.com/dora-rs/dora/pull/3281) after rc.5).

If you ran **rc.5**, almost none of this applies: rc.5 has the 1.0 wire format, store schema, recording format and APIs. Three things did move after it. The module-node rule above was finished by [#3281](https://github.com/dora-rs/dora/pull/3281), so an rc.5 module node that also sets `operators:`, `operator:` or `ros2:`, or a nested module node carrying runtime fields, is now rejected, while `build:` on a module node, which rc.5 rejected, is accepted. The descriptor structs in `dora-message` became `#[non_exhaustive]` ([#3387](https://github.com/dora-rs/dora/pull/3387)), so a struct literal of `Node`, `Descriptor` or the others no longer compiles; see [Descriptor structs are `#[non_exhaustive]`](#descriptor-structs-are-non_exhaustive-3387). And `RegisterResult` became `#[non_exhaustive]` too, with a new `peer_zenoh_endpoints` field on `Ok` ([#3364](https://github.com/dora-rs/dora/pull/3364)), which only matters if you match on it. Everything else between rc.5 and 1.0 is bug fixes, documentation and CI.

## New in 1.0

Relative to upstream 0.5.0; fork-era builds already had several of these. Not exhaustive; see the release notes for the full list.

- Service pattern: `send_service_request()` / `send_service_response()` with correlation ids
- Action pattern: goal / feedback / result metadata with cancellation
- Streaming chunks: `send_stream_chunk()` + `StreamSegment`
- Structured logging: `node.log()` / `node.log_info()`, bridged from Python `logging`
- Restart awareness: `is_restart()` / `restart_count()`, plus the `NodeRestarted` and `InputRecovered` events
- Runtime parameters: `dora param set` / `delete`, surfaced as `ParamUpdate` / `ParamDeleted` events
- Input health: `InputTracker` / `InputState`
- Fault-tolerance snapshots and coordinator state catch-up
- Arrow IPC data plane: a self-describing wire format with a ≤1-copy send path ([#2366](https://github.com/dora-rs/dora/pull/2366))
- Generic extension seam (`ExtensionRequest` / `ExtensionMessage`) so optional transports stay out of the frozen protocol ([#3219](https://github.com/dora-rs/dora/pull/3219))
- CUDA IPC via ctypes with no `numba` dependency, shipped in the `dora_tensor_pool` extension rather than the `dora` package ([#3249](https://github.com/dora-rs/dora/pull/3249))
- Record and replay (`dora record`, `dora replay`), `dora cluster`, `dora clean`, `dora doctor`, `dora validate`, `dora expand`, module dataflows, the node hub, and the MAVLink 2 bridge

## Not covered here

- The ROS2 bridge, the hub and the MAVLink bridge ship outside the 1.0 guarantee and may change in a minor release; their surfaces are not compared here. The tensor pool is covered only where it touched the frozen API (`dora.cuda`).
- CLI flags below subcommand level, beyond the ones named above. [`cli.md`](cli.md) is the reference.
- There is no `dora migrate` command. The steps above are the migration.

## Related documents

- [`api-rust.md`](api-rust.md): Rust API reference, Arrow version policy, and the 1.0 stability scope
- [`api-python.md`](api-python.md): Python API reference
- [`api-c.md`](api-c.md) / [`api-cxx.md`](api-cxx.md): C and C++ API references
- [`cli.md`](cli.md), [`yaml-spec.md`](yaml-spec.md): the 1.0 command and descriptor references
- [`distributed-deployment.md`](distributed-deployment.md): multi-machine setups
- [`websocket-control-plane.md`](websocket-control-plane.md), [`websocket-topic-data-channel.md`](websocket-topic-data-channel.md): the control plane and the topic-data `protocol_version` handshake
- [`patterns.md`](patterns.md): the service, action and streaming patterns that replace the request-reply layer
- [`plan-dora-1.0-consolidation.md`](plan-dora-1.0-consolidation.md): how the fork and upstream trees were merged
- [`phase--1-audit-2026-04-16.md`](phase--1-audit-2026-04-16.md): the wire-protocol audit this guide's protocol claims come from
- [`audit-2026-03-21-closure.md`](audit-2026-03-21-closure.md): the security and correctness findings closed in 1.0
