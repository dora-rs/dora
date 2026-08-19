# Extensions: the transport seam

Some transports do not belong in dora's stable core — whether or not they ship
in this repository. A CUDA tensor-pool needs `libcudart`,
hand-parsed shared-memory headers, a seqlock and GPU-specific transport
selection — none of which the framework should carry, and none of which it can
usefully test without GPU runners. But such a transport still needs one thing
only the daemon can provide: **someone to clean up after a node that crashed.**

The extension seam is that one thing and nothing more. It lets a transport ship
in-tree and be built (behind a feature flag) without any part of it entering
the 1.0 compatibility surface.

## What dora provides

A dataflow-scoped table of opaque byte values, keyed by `(namespace, key)`.
dora never interprets any of the three — it brokers lifetime only.

```python
# Producer
node.extension_store("my-transport", "frame-7", descriptor_bytes)
node.send_output("handle", pa.array(["frame-7"]))

# Consumer
descriptor = node.extension_load("my-transport", "frame-7")
...
node.extension_drop("my-transport", "frame-7")

# Anywhere the extension runs: release what dropped keys referred to
for key in node.drain_dropped_extension_keys("my-transport"):
    my_cache.pop(key, None)
```

Rust nodes get the same four operations on `DoraNode`
(`extension_store` / `extension_load` / `extension_drop`) plus
`dora_node_api::event_stream::extensions::drain_dropped_keys`.

### Guarantees

| | |
|---|---|
| **Reclamation on exit** | When the storing node exits — cleanly or by crashing — its entries are dropped and its readers notified. This is the reason the table is daemon-side; a node cannot do it for itself. |
| **Reclamation on finish** | Anything left when the dataflow finishes is released, so a long-lived `dora up` daemon does not accumulate entries. |
| **Notification** | Every node that stored *or read* a key is told when it goes away, so per-process resources keyed on it can be freed promptly instead of at process exit. |
| **Ownership** | Only the storing node may overwrite a key. A second node cannot redirect an existing key's readers. |
| **Scoping** | Entries are per-dataflow and per-namespace. Two dataflows, or two extensions in one process, cannot see or clobber each other. |
| **Idempotent drop** | Dropping an absent key succeeds and broadcasts nothing, so a retry after a lost reply is safe. |

### Limits

- **8192 entries per dataflow.** A store-per-frame loop that never drops will
  hit the cap and get an error rather than exhausting the daemon.
- **4096 pending drop notifications per process**, oldest evicted. Missing one
  means the extension frees that resource on its own teardown rather than
  promptly — degraded, not incorrect.
- **Values are copied** through the daemon. This is a control-plane channel for
  descriptors, not a data plane. Put the bulk payload in shared memory or a
  regular dora output and keep the descriptor here.
- **A dropped notification is best-effort.** If the target's channel is full or
  closed the daemon logs it rather than blocking the drop.

## Reaching an extension that runs in the daemon

Some transports need a half that runs *in* the daemon — a peer daemon to talk
to, a machine to resolve, a listener to own. Three more opaque carriers cover
that, and they are the whole of it:

| | |
|---|---|
| node → its own daemon's extension | `DaemonRequest::ExtensionRequest { namespace, payload }`, answered by `DaemonReply::ExtensionReply { payload }` |
| daemon → the same extension on its peers | `InterDaemonEvent::ExtensionMessage { dataflow_id, namespace, target_machine, payload }`, on `dataflow_extension_topic(dataflow_id, namespace)` |
| what the extension may ask of the daemon | the `DaemonServices` trait: machine id, zenoh session, clock, the two extension-table operations, machine resolution, and a sink for decoded peer messages |

dora reads exactly two fields: `namespace`, to pick an extension, and
`target_machine`, to drop a dataflow-scope broadcast on daemons it does not
name. The payload is bytes it never parses.

## What dora deliberately does not provide

No shared-memory helpers, no CUDA, no wire vocabulary for any particular
transport. Every protocol variant above is generic: a second extension needs no
new variant, no new topic, and no change to any of the three carriers.

**The daemon's dispatch is not yet generic, though.** `Daemon` holds one
concrete `PoolState` field and hands *every* namespace to it; the tensor-pool
then filters on its own namespace and answers "no extension registered under
{ns}" for anything else — a claim it is not actually in a position to make. A
second extension would therefore need a field on `Daemon`, its own
`#[cfg(feature)]` at each lifecycle site, and an if/else across namespaces.

The generic form is a registry — `HashMap<String, Box<dyn DaemonExtension>>` —
which turns dispatch into a lookup and moves that error to where it is true.
It was left undone deliberately: `DaemonServices` and the handlers are `async`,
so dyn-compatibility means boxed futures throughout the trait, and building
that for a hypothetical second consumer is the speculative generality this
document otherwise argues against. Build it when the second extension arrives —
the protocol will not need to change, only the routing.

If you find yourself wanting dora to grow a variant named after your transport,
that is the signal the seam is being used wrong. It has happened once already:
[#3079](https://github.com/dora-rs/dora/pull/3079) landed twelve pool-named
wire variants, an unconditional dependency on the extension crate, and 47
`unsafe` sites in `daemon/src/lib.rs`. Everything in this document is what
undoing that restored.

## Why not just send a dataflow message?

You should, for the descriptor's *delivery* — that is what `send_output` is
for, and the example above does exactly that. What a message cannot do is
survive the sender crashing: the receiver keeps a mapping to memory nobody
owns, and nothing ever tells it otherwise. dora-rs/dora#2881 is that failure
mode with a real transport attached.

Use dataflow messages to hand the key around; use the extension table for the
descriptor whose lifetime has to outlive a crash.

## The first consumer

The pinned/CUDA tensor-pool transport lives at
[`libraries/extensions/tensor-pool`](../libraries/extensions/tensor-pool) and
uses exactly these four operations. It is opt-in (`--features tensor-pool`, off
by default) and **outside the 1.0 compatibility guarantees** — that combination
is the point: a transport can ship in-tree, be built and used, and still not
freeze anything into dora's stable surface.

Its daemon half — mirror segments, the DORADMA header, the seqlock, the
direct-TCP data plane, CUDA IPC — lives in that same crate, behind its own
`daemon` feature, and reaches the daemon only through `DaemonServices`. The
daemon's side of the seam is one file,
[`binaries/daemon/src/pool_extension.rs`](../binaries/daemon/src/pool_extension.rs):
the capability implementations and the lifecycle call sites, and nothing else.

That is the shape this seam exists to produce: an extension is a package that
uses these operations, not a fork of the framework. The test for whether it is
holding is mechanical — a default `cargo tree -p dora-daemon` contains no
extension crate, and `dora-daemon` contains no `unsafe` beyond the seven
platform sites in `shutdown.rs`, `running_dataflow.rs` and
`spawn/prepared.rs`.
