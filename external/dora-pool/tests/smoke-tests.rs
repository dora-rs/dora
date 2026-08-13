// PARKED — not wired to any test harness.
//
// The eight smoke tests removed from `tests/example-smoke.rs` in dora-rs/dora
// when the memory-pool transport was parked. Paths have been rewritten to this
// package's `examples/`; the `run_smoke_test` / `run_smoke_test_local` helpers
// they call live in dora's test harness and would need reproducing here.
//
// The `memory-pool-smoke` nightly job referenced below no longer exists in
// dora's nightly.yml — it was removed with the transport.

// ---------------------------------------------------------------------------
// Memory-pool CPU transport (#2168)
//
// Requires `torch` and `tqdm`, so these are `#[ignore]`-gated and skipped by
// the main `smoke-suite` nightly job. They run in their own nightly job —
// `memory-pool-smoke` in .github/workflows/nightly.yml — which executes them
// with `--ignored`; the per-node `build:` steps pip-install CPU torch, so no
// GPU is needed. Run all six locally with the same filter CI uses:
//   cargo test --test example-smoke -- --ignored memory_pool
// (`memory_pool` matches both `smoke_memory_pool_*` and
// `smoke_local_memory_pool_*`; `smoke_memory_pool` would miss the latter).
// Or via `scripts/smoke-all.sh` which gates on `python3 -c "import torch"`
// and skips gracefully when download.pytorch.org is unreachable.
// ---------------------------------------------------------------------------

#[test]
#[ignore = "requires `torch` and `tqdm` (not in standard CI)"]
fn smoke_memory_pool_cpu2cpu() {
    run_smoke_test(
        "memory-pool-cpu2cpu",
        "examples/cpu2cpu.yml",
        Duration::from_secs(60),
    );
}

#[test]
#[ignore = "requires `torch` and `tqdm` (not in standard CI)"]
fn smoke_local_memory_pool_cpu2cpu() {
    run_smoke_test_local(
        "local-memory-pool-cpu2cpu",
        "examples/cpu2cpu.yml",
        60,
    );
}

// Negative-lifecycle scenarios validate the "warn, don't crash" contract.
#[test]
#[ignore = "requires `torch` and `tqdm` (not in standard CI)"]
fn smoke_local_memory_pool_auto_cleanup() {
    run_smoke_test_local(
        "local-memory-pool-auto-cleanup",
        "examples/auto_cleanup.yml",
        10,
    );
}

#[test]
#[ignore = "requires `torch` and `tqdm` (not in standard CI)"]
fn smoke_local_memory_pool_duplicate_free() {
    run_smoke_test_local(
        "local-memory-pool-duplicate-free",
        "examples/duplicate_free.yml",
        10,
    );
}

#[test]
#[ignore = "requires `torch` and `tqdm` (not in standard CI)"]
fn smoke_local_memory_pool_read_after_free() {
    run_smoke_test_local(
        "local-memory-pool-read-after-free",
        "examples/read_after_free.yml",
        10,
    );
}

#[test]
#[ignore = "requires `torch` and `tqdm` (not in standard CI)"]
fn smoke_local_memory_pool_write_after_free() {
    run_smoke_test_local(
        "local-memory-pool-write-after-free",
        "examples/write_after_free.yml",
        10,
    );
}

// GPU memory-pool tests: require CUDA-capable GPU(s).
// cuda_inner needs at least 1 GPU; cuda2cuda needs ≥2 distinct GPUs.
// Both are `#[ignore]`-gated because standard CI runners lack GPUs.
// Run locally:
//   cargo test --test example-smoke -- --ignored cuda
#[test]
#[ignore = "requires CUDA GPU(s)"]
fn smoke_memory_pool_cuda_inner() {
    run_smoke_test(
        "memory-pool-cuda-inner",
        "examples/cuda_inner.yml",
        Duration::from_secs(60),
    );
}

#[test]
#[ignore = "requires CUDA GPU(s) — ≥2 GPUs"]
fn smoke_memory_pool_cuda2cuda() {
    run_smoke_test(
        "memory-pool-cuda2cuda",
        "examples/cuda2cuda.yml",
        Duration::from_secs(60),
    );
}

