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
| takes a guard on slot 5 and returns without `commit()` | `try_read_pool` → `Torn` and `PoolReadGuard::valid()` false — the half-filled slot is never handed over as data |
| commits slot 5 again | `Copied`, whole slot, poison gone: the abandoned cycle did not kill the pool |
| `free_memory_pool(frames)` | the daemon's free notification, `view_is_alive` false, `try_read_pool` → `Unavailable`, a `PoolReadGuard` that throws — and `banner` still reading `Copied` |

The consumer exits non-zero unless all three `PoolReadOutcome` values were
produced and observed. A run that only ever saw `Copied` has verified half the
surface.

Two pools rather than one because `write_memory_pool` brackets its own write
cycle: it cannot run on a pool a `PoolWriteGuard` is holding open. `frames`
demonstrates the in-place path, `banner` the copy path.

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
