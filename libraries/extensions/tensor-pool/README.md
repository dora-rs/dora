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

# Daemon with this extension's daemon half
cargo build -p dora-daemon --features tensor-pool

# The `dora` binary, which hosts the in-process daemon for `dora run` / `dora up`
cargo build -p dora-cli --features tensor-pool
```

The wheel flag and the daemon flag are independent. Without the daemon half the
node-side transport still works within a single daemon; what you lose is
orphaned-segment reclamation at startup and the whole cross-machine path
(mirror segments, the direct-TCP data plane, cross-machine registration), since
those are what the daemon half implements.

```python
from dora import Node
from dora_tensor_pool import get_tensor_info, tensor_from_info   # helpers, torch-only

node = Node()
pool_id = node.register_tensor_pool(get_tensor_info(tensor), device="cuda:0")
node.send_output("frame", pool_id)          # hand the id along as normal data
...
node.write_tensor_pool(pool_id, get_tensor_info(next_tensor))
node.free_tensor_pool(pool_id)
```

Runnable dataflows are in [`examples/`](examples/), covering CPU↔CPU, CPU↔CUDA,
CUDA↔CUDA and the negative lifecycle cases.

### GPU pool placement and write completion

For a local CUDA receiver, `register_tensor_pool(info, device="cuda:N")`
allocates the pool on that receiver device and exports that allocation's IPC
handle. A CUDA writer on another device copies the complete payload into this
pool with `cudaMemcpyPeer` when P2P is available. Same-device writes use DtoD;
without P2P the existing pinned-host staging fallback is used. The copy and
receiver-device synchronization finish before the write generation is published.
An allocation, copy, or completion failure is surfaced rather than publishing a
successful frame. Device indices are process-local (`CUDA_VISIBLE_DEVICES`).
Each cross-device write includes a blocking full-payload copy and completion
synchronization, so its cost grows with payload size; subsequent reads access
receiver-local memory.

The receiver imports the handle once and can retain its view across same-host,
fixed-size writes, provided it acknowledges each frame before the next overwrite.
Printing `tensor.device` alone does not establish the physical allocation device
of a peer-accessible IPC mapping.

Because the *writer* allocates the pool on the receiver's device, a cross-device
writer creates a CUDA context on the receiver's GPU the first time it registers
or writes such a pool (the allocation, the IPC export, and enabling the
bidirectional peer pair all need one). This is a one-time, per-process cost — a
few hundred MB of device memory on each GPU in the pair — that persists for the
life of the writer process; it is not re-paid per frame. On a GPU shared with
other work, budget for it on both cards.

Regression checks:

```bash
# No CUDA hardware needed; exercises the actual embedded helpers with a fake driver.
python -m unittest discover -s libraries/extensions/tensor-pool/tests

# Needs a tensor-pool-enabled wheel/CLI and two visible P2P-capable GPUs.
dora run libraries/extensions/tensor-pool/tests/cuda_target_pool.yml
SOURCE_DEVICE=1 TARGET_DEVICE=0 dora run libraries/extensions/tensor-pool/tests/cuda_target_pool.yml
```

The GPU check queries the original allocation in the sender process, verifies
that both registration and writes preserve the caller's CUDA device, and checks
all bytes of four changing frames while retaining one receiver view. It is a
correctness check, not a throughput benchmark.

CPU-only Rust tests exercise the allocation orchestration used by registration,
including receiver-device selection, failure cleanup and caller-device
restoration. Fake-driver Python tests cover peer copies, host-staging fallback
and copy or synchronization failures without requiring CUDA hardware.

## Known open defects

| Issue | Symptom |
|---|---|
| [#2935](https://github.com/dora-rs/dora/issues/2935) | Cross-process free cleanup can be silently skipped |
| [#2890](https://github.com/dora-rs/dora/issues/2890) | The seqlock overflow fix (#2866) is incomplete: two inline end-write paths still use non-wrapping `old_gen + 1` |

[#2881](https://github.com/dora-rs/dora/issues/2881) (pools not released when a
node crashes) is **fixed**, by #3014 plus the daemon-side reclamation described
below.

[#3015](https://github.com/dora-rs/dora/issues/3015) (pool ids collide across
node restarts) is **fixed**: the per-process counter is now seeded from a
random `u64` (`PINNED_COUNTER` in `python/src/transport.rs`), so a restarted
node no longer re-derives its previous incarnation's shared-memory name. One
follow-up remains — with the collision gone, a pathological crash loop whose
receiver never frees now leaks a pool per restart up to the registry cap
instead of failing fast; the complete fix is an owner-death pool reclaim.

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

| call the daemon half | `extension_request("dora-tensor-pool", bytes)` |
| reach the same extension on a peer daemon | `InterDaemonEvent::ExtensionMessage`, published by the daemon half |

The descriptor is JSON in `python/src/seam.rs` and the request/peer messages are
postcard in `src/protocol.rs`; dora parses neither. That boundary is deliberate
— it is what lets this extension change its own metadata and its own protocol
without touching dora's wire format, and what keeps `unsafe` pointer arithmetic,
the seqlock, the segment layout and the embedded `libcudart` bindings on this
side of the line.

**Keep it that way.** If a future change wants dora to grow a request named
after this transport, that is the signal something is being done wrong: widen
the generic channel instead.

## Layout

| Path | What |
|---|---|
| `src/protocol.rs` | this transport's own messages, carried as opaque bytes by dora |
| `src/daemon/` | the daemon half (feature `daemon`): mirror segments, seqlock, direct-TCP data plane, cross-machine control flow, and `DaemonServices` — what it asks of the daemon |
| `src/lib.rs` | `TensorPoolManager` — the pool table and `/dev/shm` reclamation |
| `python/src/transport.rs` | the transport: segment layout, seqlock, CUDA helpers, the four operations |
| `python/src/seam.rs` | descriptor encode/decode over the extension channel |
| `python/dora_tensor_pool/` | `get_tensor_info` / `tensor_from_info` and the CUDA IPC helpers, importable as `dora_tensor_pool` |
| `examples/` | dataflows, CPU and CUDA |
| `tests/` | smoke tests (need `torch`; not wired to the default suite) |

## History

Built in-tree (#2168, #2386, #2619), extracted before 1.0 because it had grown
to ~3,000 lines inside the Python binding with 64 `unsafe` sites and 950 lines
of daemon lifecycle logic, then brought back here (#3152) — behind a feature
flag, reaching dora only through the extension channel, and explicitly outside
the 1.0 guarantees.

#3079 then re-integrated the cross-machine work directly into dora: 2.6k lines
and 47 `unsafe` sites in `daemon/src/lib.rs`, twelve pool-named wire variants,
an unconditional dependency from `dora-daemon`, and the daemon feature deleted
outright — while this file still claimed the transport reached dora "through the
public extension channel and nothing else". That was undone by putting the
daemon half here, behind `--features daemon`, and reducing dora's side to the
opaque carriers plus `DaemonServices`.

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
