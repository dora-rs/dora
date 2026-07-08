# Formal verification in dora

This document describes where and how dora uses formal verification, how it
fits alongside the existing QA tiers (tests, proptest, miri, cargo-mutants),
and which components are the next candidates.

## Why and where

Most of dora's correctness budget is well served by tests. Formal methods pay
off in the narrow places where a bug is (a) security- or memory-safety-
critical and (b) lives in pure logic whose input space tests can only sample.
In dora those places are:

- **Authentication primitives** — the coordinator token comparison must be
  both correct and constant-time (`libraries/message/src/auth.rs`).
- **Untrusted-input parsers** — descriptor fields parsed from user YAML
  (`parse_byte_size`, `validate_ros2_name`) must be total (never panic) and
  overflow-free.

## The verification ladder

In increasing strength (and cost):

| Tier | Tool | What a green run means | Where it runs |
|------|------|------------------------|---------------|
| Example-based tests | `cargo test` | the sampled cases behave | every commit (`qa-fast`/CI) |
| Property-based tests | proptest | hundreds of random cases per property behave | every commit (part of `cargo test`) |
| UB detection | miri | no undefined behavior on the executed test paths | nightly (`qa-nightly`) |
| Test-quality audit | cargo-mutants | the tests would catch injected logic mutations | `qa-deep` (diff-scoped) |
| **Model checking** | **Kani (CBMC)** | **the property holds for *every* input in the harness state space** | `make qa-kani` (on demand / pre-release) + nightly CI |

Kani proofs are *bounded model checking*: for loop-free integer logic the
proof is exhaustive over every possible input combination; for loops the
harness states an explicit bound (e.g. slices up to length 8, as in the auth
comparison) and the proof is exhaustive within it.

## Current proof inventory

Proof harnesses live next to the code they verify, in `#[cfg(kani)] mod
verification` modules. They are invisible to normal builds, tests, and
clippy (the `cfg` is registered in `[workspace.lints.rust]` in the root
`Cargo.toml`; proof crates opt in with `[lints] workspace = true`).

### `dora-message` — auth token comparison (`libraries/message/src/auth.rs`)

`constant_time_eq` guards coordinator WebSocket authentication. Proven for
all slice pairs up to length 8 (the loop is length-uniform, so all
control-flow paths — length mismatch, differing byte positions, empty
slices — are covered by the bound):

1. **Functional correctness**: `constant_time_eq(a, b) == (a == b)`,
   including never panicking.

Note: Kani proves functional properties, not timing. The constant-time
*structure* (no data-dependent early exit) remains a code-review invariant —
which is exactly why `AuthToken` deliberately does not derive `PartialEq`.
If stronger timing guarantees are ever needed, the [`subtle`] crate (already
in `Cargo.lock` as a transitive dependency) provides optimization barriers
that the hand-rolled XOR loop lacks; the Kani harness would carry over
unchanged as a proof of its functional behavior.

[`subtle`]: https://docs.rs/subtle

### `dora-core` — descriptor parsing properties (proptest, not Kani)

`libraries/core/src/descriptor/validate.rs` (`mod proptest_properties`)
covers the user-YAML parsers where symbolic strings would be expensive for
a model checker but random sampling is cheap and effective:

- `validate_ros2_name`: total (never panics), complete (all structurally
  valid names accepted), sound (every accepted name satisfies all documented
  ROS2 naming rules).
- `parse_byte_size`: total; integer inputs multiply exactly with overflow
  reported, never wrapped; bare integers are identity.

## Running the proofs

```bash
make qa-kani-install   # one-time: cargo install kani-verifier && cargo kani setup
make qa-kani           # run all proof harnesses (~5-10 min cold, <1 min warm)

# Single harness while iterating:
cargo kani -p dora-message --harness constant_time_eq_matches_slice_equality
```

`qa-kani` is deliberately **not** part of the per-commit `qa-fast` loop or
PR CI (Kani recompiles the crate graph with its own compiler, which is too
slow for every commit, and remote CI is kept lean by policy). Run it when
you touch a file containing proofs, and as part of pre-release gating. The
nightly workflow runs the proofs daily (`kani-proofs` job in
`.github/workflows/nightly.yml`, mirrored by
`scripts/qa/ci-nightly-jobs.sh kani-proofs`), so a broken harness is
caught within a day and auto-filed as a `nightly-regression` issue.

## Adding a new proof

1. Prefer extracting the invariant-bearing logic into a small **pure
   function** (integers/slices in, `Result` out). This is what makes the
   proof exhaustive and fast — and it usually improves the code anyway.
2. Add a `#[cfg(kani)] mod verification` next to it with `#[kani::proof]`
   harnesses. Use `kani::any()` for symbolic values, `kani::assume` for
   preconditions, and `#[kani::unwind(N)]` for loops (bound = max iterations
   + 1).
3. If the crate doesn't opt into workspace lints yet, add to its
   `Cargo.toml` (this registers `cfg(kani)` so clippy's `-D warnings`
   doesn't reject the harness):

   ```toml
   [lints]
   workspace = true
   ```

4. Nothing else: `scripts/qa/kani.sh` discovers proof crates by scanning
   for `#[kani::proof]`, so `make qa-kani` picks the new harness up
   automatically.
5. State each property in prose in the harness doc comment, and aim for the
   soundness/completeness pair — one-directional proofs silently miss
   "rejects everything" regressions.

## Candidate backlog (descending value)

- **Daemon restart-loop atomics** (`binaries/daemon/src/spawn/prepared.rs`:
  `disable_restart`, `force_restart_next`, `restart_count` orderings) —
  needs [loom], which requires swapping `std::sync::atomic` for
  `loom::sync::atomic` behind `cfg(loom)` aliases in production code; an
  invasive but mechanical refactor to do when this code next changes.
- **Coordinator `pending_replies` correlation map**
  (`binaries/coordinator/src/state.rs`) — request/response correlation and
  drop-safety; also a loom candidate.
- **ROS2 bridge FFI sequence/string conversions**
  (`libraries/extensions/ros2-bridge/src/_core/`) — Kani harnesses over the
  length/capacity invariants of `FFISeq<T>`; needs stub functions for the
  rcl allocator calls.
- **`WsMessage` untagged-enum discrimination**
  (`libraries/message/src/ws_protocol.rs`) — already has a strong 2000-case
  proptest suite; a Kani proof would need a symbolic JSON model, low
  marginal value.

[loom]: https://github.com/tokio-rs/loom
