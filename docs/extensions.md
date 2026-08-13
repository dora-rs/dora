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

## What dora deliberately does not provide

No shared-memory helpers, no CUDA, no wire vocabulary for any particular
transport. The protocol variants are `ExtensionStore` / `ExtensionLoad` /
`ExtensionDrop` — deliberately generic, so that adding a second extension needs
no change to dora at all.

If you find yourself wanting dora to grow a variant named after your transport,
that is the signal the seam is being used wrong.

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

It was previously integrated directly: ~3,000 lines inside the Python binding
with 64 `unsafe` sites, 950 lines of daemon lifecycle logic, and pool-specific
wire-protocol variants. Reworking it onto this channel removed all of that from
dora proper while keeping the feature usable.

That is the shape this seam exists to produce: an extension is a package that
uses these four operations, not a fork of the framework.
