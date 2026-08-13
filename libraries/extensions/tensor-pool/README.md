# tensor-pool — pinned-host / CUDA tensor transport

Zero-copy handoff of large tensors between dora nodes: the producer registers a
shared-tensor pool once and overwrites it per frame, so a repeated 8 MB camera
frame does not travel through the normal message path every tick. CUDA sources
and sinks get DMA and IPC paths on top; a CPU-only machine gets plain shared
memory.

## ⚠️ Not covered by dora's 1.0 compatibility guarantees

**This is an opt-in extension. It is not part of the 1.0 API surface.**

- **It may break in any release**, including a patch. The four Python methods,
  the segment layout and the descriptor format are all free to change.
- **It has known open defects** — see below. They are real, reproducible, and
  not scheduled against 1.0.
- **Its GPU paths have no automated coverage.** dora's CI has no GPU runners.
  Only the CPU path is exercised, and only in a nightly job.

Everything else in dora — the node APIs, the CLI, the descriptor format, the
wire protocol — carries the normal 1.0 stability promise. This does not.

Use it if the throughput matters more to you than the stability, and pin your
dora version if you do.

## Building with it

Off by default, so a standard build neither compiles nor exposes it.

```bash
# Python wheel with the transport
maturin develop -m apis/python/node/Cargo.toml --features tensor-pool

# Daemon with orphaned-segment reclamation
cargo build -p dora-daemon --features tensor-pool
```

Both flags are independent. Without the daemon feature the transport still
works; shared-memory segments left by a node that crashed are reclaimed when
the dataflow finishes rather than at the next start.

```python
from dora import Node
from dora.cuda import get_tensor_info, tensor_from_info   # helpers, torch-only

node = Node()
pool_id = node.register_tensor_pool(get_tensor_info(tensor), device="cuda:0")
node.send_output("frame", pool_id)          # hand the id along as normal data
...
node.write_tensor_pool(pool_id, get_tensor_info(next_tensor))
node.free_tensor_pool(pool_id)
```

Runnable dataflows are in [`examples/`](examples/), covering CPU↔CPU, CPU↔CUDA,
CUDA↔CUDA and the negative lifecycle cases.

## Known open defects

| Issue | Symptom |
|---|---|
| [#3015](https://github.com/dora-rs/dora/issues/3015) | Pool ids collide across node restarts — a restarted node cannot re-register its pool |
| [#2935](https://github.com/dora-rs/dora/issues/2935) | Cross-process free cleanup can be silently skipped |
| [#2890](https://github.com/dora-rs/dora/issues/2890) | The seqlock overflow fix (#2866) is incomplete: two inline end-write paths still use non-wrapping `old_gen + 1` |

[#2881](https://github.com/dora-rs/dora/issues/2881) (pools not released when a
node crashes) is **fixed**, by #3014 plus the daemon-side reclamation described
below.

## How it reaches dora

Through the public [extension channel](../../../docs/extensions.md) and nothing
else. dora has no knowledge of pools, CUDA or the segment layout — it brokers
the lifetime of an opaque descriptor:

| This transport | dora |
|---|---|
| publish a pool descriptor | `extension_store("dora-tensor-pool", id, bytes)` |
| look one up | `extension_load("dora-tensor-pool", id, remove=…)` |
| withdraw one | `extension_drop("dora-tensor-pool", id)` |
| learn what went away | `drain_dropped_extension_keys("dora-tensor-pool")` |

The descriptor is JSON in `python/src/seam.rs`; dora never parses it. That
boundary is deliberate — it is what lets this extension change its own metadata
without touching dora's wire protocol, and what keeps `unsafe` pointer
arithmetic, the seqlock and the embedded `libcudart` bindings on this side of
the line.

**Keep it that way.** If a future change wants dora to grow a request named
after this transport, that is the signal something is being done wrong: widen
the generic channel instead.

## Layout

| Path | What |
|---|---|
| `src/` | `dora-tensor-pool` — daemon-side reclamation of orphaned `/dev/shm` segments |
| `python/src/transport.rs` | the transport: segment layout, seqlock, CUDA helpers, the four operations |
| `python/src/seam.rs` | descriptor encode/decode over the extension channel |
| `python/tensor_info_helpers.py` | `get_tensor_info` / `tensor_from_info` for torch tensors |
| `examples/` | dataflows, CPU and CUDA |
| `tests/` | smoke tests (need `torch`; not wired to the default suite) |

## History

Built in-tree (#2168, #2386, #2619), extracted before 1.0 because it had grown
to ~3,000 lines inside the Python binding with 64 `unsafe` sites and 950 lines
of daemon lifecycle logic, then brought back here — behind a feature flag,
reaching dora only through the extension channel, and explicitly outside the
1.0 guarantees.

The design questions its origin issue
([#1872](https://github.com/dora-rs/dora/issues/1872)) posed and that were
never answered still stand, and are the right starting point for anyone
reworking this:

1. Why a new transport rather than extending zenoh-shm or making the Arrow IPC
   path CUDA-aware?
2. What is the cross-platform story? (`/dev/shm` reclamation is Linux-only
   today.)
3. Lifecycle: bounded pool size, and what happens when a producer crashes
   mid-write.
4. API shape: four methods versus a flag on `send_output`.
5. Measurement against zenoh-shm **plus caller-side pinning**, on a real
   workload — not against non-pinned zenoh-shm.
