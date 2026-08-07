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
