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
  once before failing. That retry is unreachable under lockstep — the producer
  is blocked on the ack while the read happens, so the first attempt can only
  tear if something is genuinely wrong. It is there to mirror what a real
  consumer does, not because this harness needs it, and the same is true of the
  copy in `pool-receiver-cuda.cc`.

None of the CUDA dataflows are run by `cargo run --example cxx-memory-pool`,
`scripts/smoke-all.sh` or CI: they need `nvcc` and a GPU, so they are built and
run by hand from the recipes below. They can therefore rot without anything
going red — if you change the C++ pool surface, run them.

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
| `view_transport` is `unified`, `view_ipc_present` is false, `is_integrated_gpu()` is true | the three predicates of the acceptance criterion, all asserted rather than printed. `unified` is a label recording the producer's intent, not a behaviour — see below |
| `cudaHostRegister(mapping base, segment bytes, cudaHostRegisterMapped)` **once per view**, then `+ view_payload_offset` | page-locking a real segment is slow; a per-frame registration would dominate the tick |
| a kernel compares all 16 slots to the model on every frame, under `dora::PoolReadGuard` | nothing copies the payload host→device; the only two `cudaMemcpy`s reset and read back the kernel's 12-byte verdict in a `cudaMalloc` buffer, and neither touches the payload. If the device alias did not address the pages the producer wrote, the comparison would fail — and it cannot be a snapshot taken at registration either, because the segment is registered at the hello step, before any frame exists, and 21 frames of changing content then arrive through that same never-refreshed mapping |
| `cudaHostUnregister` at the free notification, and again in `~CudaMappedView` for the pool that is never freed | dropping the view unmaps the segment out from under a live registration exactly as an unlink does, so both release points need the unregistration first |

It is deliberately fatal on a non-integrated GPU. `unified` works on a discrete
GPU too — it is merely slower there than IPC — so a run that quietly happened on
one would print the same success line while demonstrating nothing about the case
the transport exists for.

`transport == "unified"` is worth reading carefully, though: it asserts a
**label**, not a behaviour. The zero-copy path is byte-identical on a `shmem`
pool — the interop run below reads a Python-registered `shmem` segment through
the same device alias — so the string records what the producer declared, not
what makes the pool readable. The claim of #2686 still holds: the binding
supports `unified` and reaches the payload with no IPC. What makes *this* run
non-circular is the device-alias comparison, not the string.

### Building and running it

`build/pool_receiver_cuda` needs `nvcc`, so `cargo run --example
cxx-memory-pool` does not build it. From `examples/c++-memory-pool`, with
`CUDA=/usr/local/cuda` and `BRIDGE=../../target/cxxbridge/dora-node-api-cxx/install`:

```bash
cargo build -p dora-node-api-cxx -p dora-cli    # bridge + the `dora` binary
mkdir -p build
g++ -std=c++17 -c $BRIDGE/dora-node-api.cc -o build/dora-node-api.o -I $BRIDGE

# The producer. `cargo run --example cxx-memory-pool` also builds it, but only
# through `clang++ -std=c++20`, which a JetPack 5 image does not ship — so build
# it here with the same g++ as everything else.
g++ -std=c++17 -c nodes/pool-sender.cc -o build/pool-sender.o -I $BRIDGE
g++ build/pool-sender.o build/dora-node-api.o -L ../../target/debug \
  -ldora_node_api_cxx -lm -lrt -ldl -lz -pthread -o build/pool_sender

# The receiver is compiled as CUDA; the generated bridge glue is not.
$CUDA/bin/nvcc -std=c++17 -x cu -c nodes/pool-receiver-cuda.cc \
  -o build/pool-receiver-cuda.o -I $BRIDGE
g++ build/pool-receiver-cuda.o build/dora-node-api.o \
  -L ../../target/debug -ldora_node_api_cxx \
  -L $CUDA/lib64 -lcudart -lm -lrt -ldl -lz -pthread \
  -o build/pool_receiver_cuda

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

Add `--stop-after 20s` when you expect a failure. A consumer that dies mid-script
leaves the producer waiting for an ack that will never come, and `dora run` has
no timeout of its own.

### Each assertion, and what breaking it looks like

An assertion that cannot fail is decoration. These were checked by breaking what
each one guards, one at a time, and confirming the run failed for the right
reason:

| break this | and this catches it |
|---|---|
| remove `DORA_MEMORY_POOL_TRANSPORT: unified` | `expected transport 'unified', got 'shmem'` |
| take `mapped_.host` for the payload instead of `mapped_.device` | the kernel dies with `an illegal memory access was encountered` — on this Jetson the device alias is a genuinely different address (`0xffff9c133000` vs `0x20310a000`), so the host pointer is not device-addressable |
| use the mapping base without `view_payload_offset` | the kernel reports `the host wrote 165 and the GPU read 68` — 68 is `'D'`, the first byte of the segment's `DORADMA` magic, exactly the failure `cuda_pool.hpp` warns is silent |
| skip `release()` in the free drain | `the mapping of 'frames' is still registered after the free notification` |
| ack only frames, not `kHello`/`kAbandoned`/`kFreed` | the producer never advances: `[sender] stopped at step 0 of 23 without completing the script`, and the run only ends at `--stop-after` |
| query a device that is not there | `device 0 is not an integrated GPU (or could not be queried)` |

## Python → C++ interop

Everything above has C++ on both ends. `dataflow-python-interop.yml` replaces the
producer with `python_sender.py`, which registers its pool through the
**untouched Python binding** — `register_memory_pool(tensor_info, "cpu")` — and
never touches CUDA. `dataflow-python-interop-cuda.yml` runs the same sender
against a receiver that reads the pool from a kernel.

That second run is the whole claim of the transport in one line: a segment
written by Python, read zero-copy by a CUDA kernel on an integrated GPU, with no
CUDA IPC anywhere.

### Why this needs its own consumer

`pool-receiver.cc` and `pool-receiver-cuda.cc` cannot be pointed at a Python
producer, and not for cosmetic reasons:

- **The pool id is generated, not chosen.** Python's is
  `pool_<node-id>_<counter>`, so there is no `"frames"` to ask for. The sender
  publishes the id on a `pool_id` output and the consumer maps whatever it is
  told — which is a stronger check than the hard-coded id, not a weaker one.
- **Python cannot abandon a write cycle.** `write_memory_pool` brackets its own
  seqlock in Rust; there is no API for leaving the generation odd. The `Torn`
  outcome that both existing consumers *require* to have happened exactly once
  can therefore never occur.
- **A Python CPU pool reports transport `shmem`, not `unified`.** Python writes
  no `transport` key in the metadata JSON, so `parse_header` resolves it from
  `ipc_flag = 0`. `pool-receiver-cuda.cc` asserts `unified` and would fail on a
  segment it can otherwise read perfectly.

- **The pool id is not a substring of the segment name.** The C++ producer
  derives both from `naming::segment_name`, so `checks.h` can look for
  `…_<node>_<pool-id>`. Python formats the segment name
  (`dora_pool_{dataflow}_{node}_{counter}`) and the buffer id
  (`pool_{node}_{counter}`) in two independent `format!` calls that share no
  code; only the `<node>_<counter>` tail is common.

### The two segments are not byte-identical

"Byte-identical on the wire" is the claim this example exists to test, and it is
**too strong**. Dumped side by side for the same 16 KiB `uint8` payload, a Python
CPU pool and a C++ `unified` pool disagree in four places:

| | Python (`register_memory_pool(…, "cpu")`) | C++ (`Transport::Unified`) |
|---|---|---|
| `transport` key | absent | `"unified"` |
| `pinned_type` | `"cpu"` | `"cuda"` |
| JSON separators | `json.dumps` defaults: `", "` / `": "` | `serde_json` compact |
| `json_len` | 73 | 91 |
| `write_gen` after registration | `2` — the register copies the source tensor and closes a seqlock cycle around it | `0` — `PoolSegment::create` allocates and registers without an intervening write |

What makes the interop work anyway is a better argument than byte-identity, and
it is the one the consumer actually relies on: **`json_len`, `data_offset`,
`pinned_type` and the generation are read out of the header, never assumed.**
What has to agree is the layout the header *describes* — the `DORADMA` magic,
the `json_len`/`data_offset` pair, the seqlock at offset 96, and a data region
starting at `data_offset` — and that is what makes one `cudaHostRegister` +
device-alias path serve both.

Two consequences worth not misreading:

- **`payload at +512` is a property of this tensor, not of the format.**
  `data_offset_for` rounds `json_len` up to a multiple of 256, and 73 and 91
  both round to 256 — so both producers land on 512 *here*. A longer dtype
  string or a higher-rank shape puts one at 512 and the other at 768, and
  nothing breaks when it does, because the consumer reads the offset.
- **`write_gen = 0` is a state Python cannot represent.** A C++ pool is visible
  to the daemon between `create` and its first write, and a consumer mapping in
  that window reads a full frame of zeros that passes the seqlock — even is
  even. The handshake in every dataflow here closes that window, so it is not
  exercised, but it is a genuine producer asymmetry that "same semantics" would
  hide.

`nodes/pool-interop-receiver.cc` is one source compiled twice — g++ for the CPU
path, `nvcc -x cu` for the device alias. Precisely: the two builds differ in
`read_ring`'s body and in `map()`/`release()` (no-ops on the CPU build), so the
*read* really is two implementations. What the shared source locks together is
everything either build claims about the **segment** — the pool id arriving as
data, the segment-name check, the transport/`ipc_present`/`payload_len`
assertions, the ring model and its script, and the free path through to
`Unavailable`. Those cannot drift because there is one copy of them. The CUDA
build never runs `try_read_pool`'s `Copied` path against a Python pool
(`use_copy_path` is discarded there on purpose — copying the payload would
defeat the point), so that is CPU-only coverage.

### What builds and runs where

`cargo run --example cxx-memory-pool` **compiles `pool_interop_receiver`** along
with the other two nodes, so the interop source cannot rot when the pool surface
changes; that is the whole reason it is in `run.rs`'s `NODES`. It then runs
`dataflow-python-interop.yml` **only if** `get_python_path()`'s interpreter can
`import dora, pyarrow`, and otherwise prints a warning naming what is missing —
the binding is `abi3-py311` and has to come from the workspace, which a bare
checkout has no reason to have. The nightly `examples` job provisions it
(Linux-only) so that skip is not permanent.

The CUDA twin is out of scope for `run.rs`: it needs `nvcc`, and GitHub's
runners have no GPU at all, let alone an integrated one. It is built and run by
hand, below.

### Building and running it

The Python node needs the **workspace** binding, not PyPI `dora-rs`, whose
message format has drifted. The binding is `abi3-py311`, so it needs a Python
≥ 3.11 even where the system Python is older:

```bash
cd /path/to/dora
uv venv --python 3.11 target/interop-venv
VIRTUAL_ENV=$PWD/target/interop-venv uv pip install maturin numpy 'pyarrow>=14.0.1' pyyaml
(cd apis/python/node && VIRTUAL_ENV=$PWD/../../../target/interop-venv \
   PATH=$PWD/../../../target/interop-venv/bin:$PATH maturin develop)
```

The consumers, from `examples/c++-memory-pool`, with `CUDA=/usr/local/cuda` and
`BRIDGE=../../target/cxxbridge/dora-node-api-cxx/install`:

```bash
cargo build -p dora-node-api-cxx -p dora-cli
mkdir -p build
g++ -std=c++17 -c $BRIDGE/dora-node-api.cc -o build/dora-node-api.o -I $BRIDGE

# CPU
g++ -std=c++17 -c nodes/pool-interop-receiver.cc \
  -o build/pool-interop-receiver.o -I $BRIDGE
g++ build/pool-interop-receiver.o build/dora-node-api.o \
  -L ../../target/debug -ldora_node_api_cxx -lm -lrt -ldl -lz -pthread \
  -o build/pool_interop_receiver

# CUDA — same source, nvcc defines __CUDACC__ and the device path compiles in
$CUDA/bin/nvcc -std=c++17 -x cu -c nodes/pool-interop-receiver.cc \
  -o build/pool-interop-receiver-cuda.o -I $BRIDGE
g++ build/pool-interop-receiver-cuda.o build/dora-node-api.o \
  -L ../../target/debug -ldora_node_api_cxx \
  -L $CUDA/lib64 -lcudart -lm -lrt -ldl -lz -pthread \
  -o build/pool_interop_receiver_cuda
```

Then, with the venv exported so the daemon spawns the right interpreter
(`get_python_path` asks `uv python find` first):

```bash
export VIRTUAL_ENV=$PWD/../../target/interop-venv
../../target/debug/dora run dataflow-python-interop.yml      --stop-after 90s
../../target/debug/dora run dataflow-python-interop-cuda.yml --stop-after 90s
ls /dev/shm | grep dora_pool_ || echo "clean after run"
```

`--stop-after` matters on a failure: a consumer that dies mid-script leaves the
sender waiting for an ack that never comes, and `dora run` has no timeout of its
own.

Measured on a Jetson (JetPack 5, CUDA 11.4, Python 3.11.15):

```
[python-sender] registered pool `pool_python-pool-sender_1`
[interop] Python published pool id `pool_python-pool-sender_1`
[interop] mapped Python's pool: segment=dora_pool_019fdb17-…_python-pool-sender_1 transport=shmem ipc_present=false payload=16384 bytes
[interop] daemon released pool `pool_python-pool-sender_1`
[interop] Python's pool is permanently unreadable after its free
[interop] 20 Python-written frames verified (10 copied, 10 zero-copy), 1 unavailable
```

```
[cuda-interop] mapped Python's pool: segment=dora_pool_019fdb17-…_python-pool-sender_1 transport=shmem ipc_present=false payload=16384 bytes
[cuda-interop] page-locked 16896 bytes of Python's segment: host=0xffff95fef000 device=0x20310a000, payload at +512
[cuda-interop] unregistered `pool_python-pool-sender_1` before its segment could go away
[cuda-interop] Python's pool is permanently unreadable after its free
[cuda-interop] 20 Python-written frames verified through the device alias, 1 unavailable
```

`host` and `device` are genuinely different addresses, so the kernel is not
reading through the host pointer; the payload sits at +512 into the segment,
which is where Python's `data_offset` puts it after a 256-byte header and a
padded metadata region.

### What the sender cannot do here

`register_memory_pool(tensor_info, "cuda")` **fails on this machine, by
design**: the Python binding hard-requires a `cudaIpcGetMemHandle` export, which
an integrated GPU does not support, and it bails rather than hand back a pool
every later write would reject. Only the *sender* half of Python is stuck there.
A CPU pool feeds a CUDA C++ consumer perfectly well, which is exactly what the
second run above does. Closing that gap would mean teaching the Python binding
`unified`, and this change does not.

### Each interop assertion, and what breaking it looks like

Same discipline as the table above, but the mutations are applied to
**`python_sender.py`** — to what the producer writes — since that is the side
whose bytes are in question:

| break this in the sender | and this catches it |
|---|---|
| write to slot `n+1` while the notice says slot `n` | CPU: `step 1: slot 0 byte 0 is 0, expected 1`; CUDA: `step 1: the kernel found 2048 mismatched bytes; the first is at offset 0 (slot 0), where Python wrote 1 and the GPU read 0` |
| skip `write_memory_pool` on one step but publish its notice | `step 3: slot 2 byte 0 is 0, expected 3` — the ring model, not the notice, is what is believed |
| overwrite the segment's magic with `XORADMA\0` after registering | `failed to map Python's pool: pool segment '…': segment magic is not DORADMA — not a dora memory pool` |
| register 8 KiB instead of 16 KiB | `payload is 8192 bytes, expected 16384` (`view_payload_len`, before any read) |
| publish a pool id one counter off | `failed to look up memory pool 'pool_python-pool-sender_2': memory pool with ID pool_python-pool-sender_2 not found` — the daemon, not a guessed segment name, is what resolves an id |
| poke the seqlock generation at offset 96 to an odd value after a write | CPU and CUDA both: `step 5 was overwritten mid-read twice` — the seqlock Python writes and the seqlock C++ reads are the same field |
| skip `free_memory_pool` but still send the freed notice | `no free notification for 'pool_python-pool-sender_1' arrived` |

Every one of those also ends with the sender's own
`stopped at step N of 21 without completing the script`, because the consumer
stops acking as soon as it fails.

Two more, on the consumer and the runner rather than the sender, because they
guard the two things this example changed structurally:

| break this | and this catches it |
|---|---|
| pass a tail one character off to the shared `segment_name_from_daemon` | `segment name 'dora_pool_019fdb5b-…_python-pool-sender_1' is not the daemon's name for pool 'pool_python-pool-sender_1'` — the assertion still fires after being hoisted into `checks.h` and parameterised |
| run `cargo run --example cxx-memory-pool` with no importable binding | `skipping the Python interop dataflow: '/usr/bin/python3 -c "import dora, pyarrow"' failed … ModuleNotFoundError: No module named 'dora'`, and the example still exits 0 — the skip is loud and names the fix, rather than passing silently |
