# Extensions: the out-of-tree transport seam

Some transports do not belong in dora. A CUDA memory-pool needs `libcudart`,
hand-parsed shared-memory headers, a seqlock and GPU-specific transport
selection — none of which the framework should carry, and none of which it can
usefully test without GPU runners. But such a transport still needs one thing
only the daemon can provide: **someone to clean up after a node that crashed.**

The extension seam is that one thing and nothing more.

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

## Prior art in this repo

The pinned/CUDA memory-pool transport was built *inside* dora and removed
before 1.0 — it had grown to 3,000 lines in the Python binding with 64 `unsafe`
sites, plus 950 lines of daemon-side lifecycle logic, and no CI could exercise
its GPU paths. It now lives in `external/dora-pool`, whose README records both
the design questions it never answered (dora-rs/dora#1872) and the constraints
on bringing it back.

That is the shape this seam exists to prevent: a transport should be a package
that uses these four operations, not a fork of the framework.
