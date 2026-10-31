# Dora C++ Extension Seam Example

This example uses dora's **extension seam** from C++: the daemon-side table of
opaque bytes that a transport living outside dora uses to share descriptors
and get them cleaned up when a node exits. See
[`docs/extensions.md`](../../docs/extensions.md) for what the seam guarantees,
and dora-rs/dora#2686 for the C++ parity request.

## Running

```bash
cargo run --example cxx-extension-seam
```

Requires `clang++` with C++20 support. The harness builds
`dora-node-api-cxx`, compiles the two nodes into `build/`, and then runs
`dataflow.yml`.

## What each node does

| Node | Source | Role |
|------|--------|------|
| `producer` | `nodes/producer.cc` | Stores a descriptor under a key, sends only the key, waits for the drop notification |
| `consumer` | `nodes/consumer.cc` | Loads the descriptor, checks it, drops it, and checks that a second load finds nothing |

The producer polls `drain_dropped_extension_keys` on every timer tick, because
drop notifications are not delivered as events. Either node exits with an
error if a step fails, so the run fails too. A run that is still going after
60 seconds (a node hung) also fails.

## Expected output

```
[producer] stored frame-1 and sent its key
[consumer] loaded 4 bytes for frame-1
[consumer] dropped frame-1
[producer] frame-1 was dropped, releasing it
```
