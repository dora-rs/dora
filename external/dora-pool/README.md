# dora-pool — parked memory-pool transport

Pinned-host / CUDA memory-pool transport for zero-copy tensor handoff between
dora nodes. **Extracted from `dora-rs/dora` before the 1.0 release and parked
here.** It does not build against dora today, and dora 1.0 ships no pool API.

This directory is staged for lifting into its own repository. Nothing here is
compiled by a dora build.

---

## ⚠️ Read this before writing any code — the seam contract

If you are reinstating this transport (human or agent), the single most
important constraint is:

> **Add a seam to dora. Do not re-integrate the transport into dora.**

The version parked here was *woven through* dora: 3,043 lines inside
`apis/python/node/src/lib.rs` (70% of that file), 952 lines of handlers and
reclamation logic in `binaries/daemon/src/lib.rs`, five wire-protocol variants,
and plumbing in six more files. That is the shape to avoid, and it is why this
was parked rather than maintained.

### What dora may gain

A seam, in this order of preference:

1. **Nothing.** Check first whether the transport can be built entirely on
   dora's existing public API (`DoraNode`, `send_output`, shared memory).
   #1872 Q1 asks exactly this and was never answered — see below.
2. **One accessor**, if step 1 fails. Something shaped like
   `node._pool_handle() -> PoolHandle`, where `PoolHandle` is an opaque
   capability object the external package drives. Target: **under 100 lines in
   dora, zero `unsafe`, no CUDA symbols, no knowledge of the DORADMA layout.**
3. **A generic extension point**, if a second consumer ever justifies it —
   e.g. a registered side-channel over the existing daemon connection, with
   dora carrying no pool-specific vocabulary at all.

### What must NOT go back into dora

Non-negotiable, because each of these is what made the parked version
unmaintainable:

| Never in dora | Why |
|---|---|
| `unsafe` pointer arithmetic against the DORADMA header | ~40 sites reading hardcoded offsets (8/16/24/32/96) out of memory another process writes |
| The seqlock (`seqlock_begin_write` / `_begin_if_even` / `_end`) | Concurrency primitive owned by the transport, not the framework; still has an open correctness bug (#2890) |
| `unsafe impl Send`/`Sync` on raw-pointer slots | Six of them, asserting thread-safety on process-wide statics |
| Embedded Python that `ctypes`-loads `libcudart.so` | A ~300-line Python program living as a Rust string literal; it belongs in a Python package |
| CUDA transport selection (P2P, IPC handles, transit buffers, host staging) | Entirely the external package's concern |
| Pool-specific `DaemonRequest`/`DaemonReply` variants | Freezes pool vocabulary into dora's wire protocol; see the generic side-channel option above |
| Pool lifecycle logic in daemon node-exit / dataflow-finish paths | The parked version threaded reachability tracking through five separate exit paths |

**Litmus test:** if `grep -ri 'cuda\|doradma\|pinned\|seqlock' ` over the dora
tree returns anything outside a docs file, the seam is wrong.

**Budget:** a correct reinstatement should touch **under 200 lines of dora**,
across no more than two or three files, with no new `unsafe`. For scale: the
integration this replaces touched 4,356 lines across 17 files.

No patch file is shipped here, deliberately — a ready-to-apply re-integration
sitting next to this contract would invite exactly the outcome it argues
against. If you need to see what the old integration touched, it is the reverse
of the commit that removed it:

```bash
git -C /path/to/dora show <extraction-commit>          # dora-rs/dora#3152
```

---

## Why this was parked

Not a judgement on the idea — the gap it addresses is real. It was parked
because the implementation never cleared the bar its own design issue set.

[dora-rs/dora#1872](https://github.com/dora-rs/dora/issues/1872) ("Opt-in
pinned-host memory pool for high-throughput CPU→GPU transfers") was filed on
2026-05-19, the same day PR #1623 (+2,717 lines) was closed for being "7×
larger than the feature needed". #1872 says, verbatim:

> This issue documents the gap and invites proposals. **It does NOT commit to
> an architecture.** dora has historically moved AWAY from custom shared-memory
> infrastructure (see #1745…). Any new transport has to clear a high bar.
>
> Priority: **not currently scheduled.** Filed for visibility.

PR #2168 landed the same architecture 24 days later, merged 5 days after
opening, answering none of the five questions #1872 said a proposal must answer
"before writing code".

### Re-entry criteria

**A. Answer #1872's five questions** — in an issue or RFC, before code:

1. **Why a new transport, not extending an existing one?** Can zenoh-shm take
   an optionally-pinned host-memory provider backend? Can the Arrow IPC path be
   made CUDA-aware? Why are these worse?
2. **What is the cross-platform story?** POSIX shm is Linux/macOS; Windows
   needs `CreateFileMapping`. What is the graceful fallback when CUDA is absent
   entirely — the feature must be invisible to the majority of users who never
   touch a GPU.
3. **Lifecycle: who owns pinned memory and when is it freed?** Pinned memory is
   a finite system resource. Bounded pool size? Freed on producer drop, daemon
   shutdown, or explicit free? What happens when the producer crashes
   mid-write? *This is the question the four bugs below all descend from.*
4. **API shape.** `node.send_output(..., pinned=True)` versus four new methods?
   How does a consumer learn whether it can DMA directly or must fall back?
5. **Measurement.** Benchmark against **zenoh-shm plus caller-side pinning** —
   the workaround that exists today — not against non-pinned zenoh-shm. Real
   workload (1080p RGBA at 30 Hz ≈ 8 MB/frame), not a microbenchmark. #2168
   reported "4.5 GB/s locally" with no baseline at all.

**B. Close the four open correctness bugs.** All were open against dora's 1.0
milestone when this was parked:

| Bug | Symptom | State when parked |
|---|---|---|
| [#3015](https://github.com/dora-rs/dora/issues/3015) | Pool ids collide across node restarts — a restarted node cannot re-register | PR #3056 open |
| [#2881](https://github.com/dora-rs/dora/issues/2881) | Pools not released when a node crashes or is dynamically removed | **fixed** — #3014 landed on dora `main` before this extraction, so the parked copy is the post-fix code |
| [#2935](https://github.com/dora-rs/dora/issues/2935) | Cross-process `FreeMemoryPool` cleanup silently skipped | **partial fix included** — `free_pool_and_notify` / `notify_memory_pool_freed` plus four regression tests; issue still open, verify before relying on it |
| [#2890](https://github.com/dora-rs/dora/issues/2890) | Seqlock overflow fix (#2866) incomplete — two inline end-write paths still use non-wrapping `old_gen + 1` | PR #3149 draft |

**C. Have a GPU CI story.** The CUDA paths — IPC handles, P2P selection,
transit staging — never had automated coverage in dora. The only tested path
was CPU (`smoke_memory_pool_cpu2cpu` and five `smoke_local_memory_pool_*`, all
`#[ignore]`-gated, run in a dedicated nightly job). The unit tests that exist
are pure decision-matrix logic (`should_pin`, `classify_transport`,
`classify_write_path`) — they test which branch is chosen, never what the
branch does.

---

## What is in here

| Path | What it is | Builds? |
|---|---|---|
| `daemon-side/` | The `dora-memory-pool` crate: pool registry, metadata, reachability, orphan cleanup. Only depends on `tracing`. | **Yes**, standalone |
| `python-binding/node_binding.rs` | The 3,043 lines lifted out of `apis/python/node/src/lib.rs`, in original order with section markers. Statics, DORADMA header handling, seqlock, CUDA ctypes helpers, the four `#[pymethods]`, the receive-path free drain, and the `try_doradma_read` fast path. | **No** — needs the seam |
| `examples/` | Nine dataflow YAMLs (`cpu2cpu`, `cpu2cuda`, `cuda2cpu`, `cuda2cuda`, `cuda_inner`, plus four negative-lifecycle scenarios) with `sender.py` / `receiver.py`. | n/a |
| `python-binding/tensor_info_helpers.py` | `get_tensor_info` / `tensor_from_info` and their dtype maps, moved out of `dora/cuda.py` — they only ever fed the pool methods. | n/a |
| `tests/smoke-tests.rs` | The eight smoke tests removed from `tests/example-smoke.rs`. | **No** — needs a harness |

`python-binding/node_binding.rs` does not compile on its own by design. It
needs from dora only: `&mut DoraNode` (for the three pinned-memory calls),
`node_id`, `dataflow_id`, and a free-queue drain — four calls and two fields.
That narrowness is the argument that a small seam is achievable; it is not a
licence to restore the patch.

[#3014](https://github.com/dora-rs/dora/pull/3014) (the #2881 fix) landed on
dora `main` before this extraction, so `daemon-side/src/lib.rs` here is
byte-identical to the post-fix version — the fix travelled with the transport
rather than being stranded.

## Lifting this into its own repository

```bash
git subtree split -P external/dora-pool -b dora-pool-split
# then push dora-pool-split to the new repo's main
```

After lifting, delete `external/dora-pool` from dora and keep only a pointer to
the new repository.
