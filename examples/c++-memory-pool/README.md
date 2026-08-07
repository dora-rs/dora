# C++ Memory Pool Example

A producer registers a pool of 16 × 1 KiB slots, fills one slot **in place**
per step through `dora::PoolWriteGuard`, and publishes four bytes on the topic
— the slot index, not the frame. That is the shape a camera pipeline uses, and
the reason the transport exists.

```bash
cargo run --example cxx-memory-pool
```

Linux only: the pool is a named POSIX segment in `/dev/shm`. The runner fails
the example if any `dora_pool_*` segment is left there afterwards, because the
daemon is the only thing that unlinks one.

## What the run verifies

The two nodes run in lockstep — the producer advances only on the consumer's
ack — so every check below is deterministic rather than timing-dependent.

| The producer does | The consumer must observe |
|---|---|
| maps `frames` and `banner` by **id** | a segment name of the form `dora_pool_<dataflow-uuid>_cxx-pool-sender_<id>`, which only the daemon could have supplied |
| writes `banner` once with `write_memory_pool` | `try_read_pool` → `Copied`, 32 bytes of `0xA5` |
| commits a slot (20 steps, wrapping the ring) | `Copied` on even steps, `PoolReadGuard::valid()` on odd ones, and **every** slot of the ring matching what was written to it |
| takes a guard on slot 5 and returns without `commit()` | `try_read_pool` → `Torn` (**abandoned cycle**, not a concurrent tear) and `PoolReadGuard::valid()` false — the half-filled slot is never handed over as data |
| commits slot 5 again | `Copied`, whole slot, poison gone: the abandoned cycle did not kill the pool |
| `free_memory_pool(frames)` | the daemon's free notification, `view_is_alive` false, `try_read_pool` → `Unavailable`, a `PoolReadGuard` that throws — and `banner` still reading `Copied` |

The consumer exits non-zero unless all three `PoolReadOutcome` values were
produced and observed. A run that only ever saw `Copied` has verified half the
surface.

Two pools rather than one for two reasons, neither of them an API restriction.
`write_memory_pool` writes the whole payload exactly, so a copy-path write to
`frames` would have to be all 16 KiB and would flatten the ring the consumer is
checking. And "freeing one pool leaves the other readable" needs a second pool
that outlives the free. (`write_memory_pool` also cannot run while a
`PoolWriteGuard` holds a cycle open on the *same* pool — but nothing here does:
the guard is scoped to one function, and the banner is written before the event
loop starts.)

## What this example is not

**It exercises no concurrency at all.** The two nodes run in lockstep — the
producer advances only on the consumer's ack — so the writer never writes while
the reader reads. The seqlock's contended path is therefore untouched: its
fences, its back-pressure behaviour, and multiple frames in flight are all
outside what this run covers. The `Torn` it produces comes from an *abandoned
write cycle*, whose generation is left odd deliberately, not from a race. Racing
readers and writers are covered by the unit tests in
`libraries/extensions/memory-pool/src/seqlock.rs`.

What this is, is a conformance harness for the C++ surface: it checks that the
guards, the read outcomes, the daemon round trip and the free path behave as
their headers document, on a real daemon — which is the one thing a unit test
cannot do, because C++ can only obtain a pool from a live daemon.

Consequently, some of it is scaffolding rather than a pattern to copy:

- **The `ack` topic and the lockstep it creates** exist so the assertions cannot
  race. A real consumer just reads on each notice and does not ack.
- **The ring model** (the consumer re-checking all 16 slots every frame) is an
  assertion, not a consumer's job.
- **Treating `Torn` as an error** is specific to lockstep, where it can only
  mean the producer abandoned a cycle. A real consumer retries on the next
  event, as `dora/memory_pool.hpp` shows; even here, the frame path retries
  once before failing.

## Transports

| transport | data region | receiver reaches the payload by |
|---|---|---|
| `shmem` | full | reading the mapping directly (CPU) |
| `unified` | full | `cudaHostRegister(..., cudaHostRegisterMapped)` + `cudaHostGetDevicePointer` |
| `ipc` | header only | `cudaIpcOpenMemHandle` — produced by the Python binding only |

`transport: "auto"` resolves to `unified` when `receiver_is_cuda` is set and to
`shmem` otherwise; `DORA_MEMORY_POOL_TRANSPORT` overrides `auto` and nothing
else. On an integrated GPU `unified` is the only mode that works, because
`cudaIpcGetMemHandle` is unsupported there.

## The CUDA consumer

`dataflow-cuda.yml` runs the **same producer binary** against
`nodes/pool-receiver-cuda.cc`, which reads the pool from a kernel instead of
from the CPU. That is the acceptance case of
[dora-rs#2686](https://github.com/dora-rs/dora/issues/2686): a C++ node
registers a memory pool and does zero-copy CPU→GPU transfer through the
host-pinned path on an integrated GPU, with no CUDA IPC.

The producer is not forked for it. It asks for `auto` with `receiver_is_cuda`
unset, so the dataflow sets `DORA_MEMORY_POOL_TRANSPORT: unified` in the
producer's `env:` block — the override applies to `auto` and to nothing else.
Only one consumer is wired up: the producer advances on the first ack matching
the step it is waiting for, so a second consumer would break the lockstep.

What the run adds on top of the CPU one:

| | |
|---|---|
| `view_transport` is `unified`, `view_ipc_present` is false, `is_integrated_gpu()` is true | the three predicates of the acceptance criterion, all asserted rather than printed |
| `cudaHostRegister(mapping base, segment bytes, cudaHostRegisterMapped)` **once per view**, then `+ view_payload_offset` | page-locking a real segment is slow; a per-frame registration would dominate the tick |
| a kernel compares all 16 slots to the model on every frame, under `dora::PoolReadGuard` | nothing copies the payload host→device; the only `cudaMemcpy` moves the kernel's 12-byte verdict back out of a `cudaMalloc` buffer. If the device alias did not address the pages the producer wrote, the comparison would fail |
| `cudaHostUnregister` at the free notification, and again in `~CudaMappedView` for the pool that is never freed | dropping the view unmaps the segment out from under a live registration exactly as an unlink does, so both release points need the unregistration first |

It is deliberately fatal on a non-integrated GPU. `unified` works on a discrete
GPU too — it is merely slower there than IPC — so a run that quietly happened on
one would print the same success line while demonstrating nothing about the case
the transport exists for.

### Building and running it

`build/pool_receiver_cuda` needs `nvcc`, so `cargo run --example
cxx-memory-pool` does not build it. From `examples/c++-memory-pool`, with
`CUDA=/usr/local/cuda` and `BRIDGE=../../target/cxxbridge/dora-node-api-cxx/install`:

```bash
cargo build -p dora-node-api-cxx -p dora-cli    # bridge + the `dora` binary
mkdir -p build

# The receiver is compiled as CUDA; the generated bridge glue is not.
$CUDA/bin/nvcc -std=c++17 -x cu -c nodes/pool-receiver-cuda.cc \
  -o build/pool-receiver-cuda.o -I $BRIDGE
g++ -std=c++17 -c $BRIDGE/dora-node-api.cc -o build/dora-node-api.o -I $BRIDGE
g++ build/pool-receiver-cuda.o build/dora-node-api.o \
  -L ../../target/debug -ldora_node_api_cxx \
  -L $CUDA/lib64 -lcudart -lm -lrt -ldl -lz -pthread \
  -o build/pool_receiver_cuda

# The producer comes from the CPU example's build; run it once first if
# build/pool_sender is not there.
../../target/debug/dora run dataflow-cuda.yml
```

The `dora run` exit code is the verdict — both nodes exit non-zero on any failed
assertion. Then check for leaks by hand, because `run.rs`'s `/dev/shm` gate only
wraps `dataflow.yml`:

```bash
ls /dev/shm | grep dora_pool_ || echo "clean after run"
```

Compiling the receiver and the bridge glue separately keeps `nvcc` away from the
generated `dora-node-api.cc`, which has no CUDA in it. Both objects are built as
C++17 — `nvcc` 11.4 goes no further, and mixing standards across the two would
be an ODR trap.
