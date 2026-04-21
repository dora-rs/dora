# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Dora (AI-Dora, Agentic Dataflow-Oriented Robotic Architecture) is a 100% Rust framework for building real-time robotics and AI applications. It provides 10-17x faster latency than ROS2 via zero-copy shared memory and Apache Arrow data format. Supports Rust, Python (PyO3), C, and C++ nodes.

## Build Commands

```bash
# Build all (Python packages require maturin, exclude them for normal builds)
cargo build --all --exclude dora-node-api-python --exclude dora-operator-api-python --exclude dora-ros2-bridge-python

# Build specific package
cargo build -p dora-cli
cargo build -p dora-daemon

# Check all
cargo check --all

# Test all (excluding Python)
cargo test --all --exclude dora-node-api-python --exclude dora-operator-api-python --exclude dora-ros2-bridge-python

# Test single package
cargo test -p dora-core

# Lint
cargo clippy --all

# Format
cargo fmt --all

# Format check (CI uses this)
cargo fmt --all -- --check

# Run examples
cargo run --example rust-dataflow
cargo run --example benchmark --release

# Install CLI locally
cargo install --path binaries/cli --locked

# Run a dataflow
dora run examples/rust-dataflow/dataflow.yml
dora run examples/python-dataflow/dataflow.yml --uv --stop-after 10s
```

## Workspace Layout

- **Rust edition 2024; MSRV and default workspace package metadata live in `[workspace.package]` in the root `Cargo.toml`.** Most crates inherit the workspace version via `version.workspace = true`, but a few (e.g. `apis/rust/operator/types`, the `examples/error-propagation/*` samples) pin their own version independently.
- Python packages use PyO3 0.28 and are built with **maturin**, not cargo

### Key crates

| Path | Crate | Role |
|------|-------|------|
| `binaries/cli` | dora-cli | CLI binary (`dora` command) - build, run, stop dataflows |
| `binaries/daemon` | dora-daemon | Spawns nodes, manages local shared-memory/TCP communication |
| `binaries/coordinator` | dora-coordinator | Orchestrates distributed multi-daemon deployments |
| `binaries/runtime` | dora-runtime | In-process operator execution runtime |
| `libraries/message` | dora-message | All inter-component message types and protocol definitions |
| `libraries/core` | dora-core | Dataflow descriptor parsing, build utilities, Zenoh config |
| `libraries/shared-memory-server` | shared-memory-server | Zero-copy IPC for large messages (>4KB) |
| `apis/rust/node` | dora-node-api | Rust API for writing custom nodes |
| `apis/rust/operator` | dora-operator-api | Rust API for writing in-process operators |
| `apis/python/node` | dora-node-api-python | Python node API (PyO3) |

## Architecture

```
CLI  -->  Coordinator  -->  Daemon(s)  -->  Nodes / Operators
              (distributed)    (per machine)    (user code)
```

- **CLI <-> Coordinator**: WebSocket (port 6013) for build/run/stop commands
- **Coordinator <-> Daemon**: WebSocket for node spawning and dataflow lifecycle
- **Daemon <-> Daemon**: Zenoh for distributed cross-machine communication
- **Daemon <-> Node**: Shared memory for messages >4KB (zero-copy), TCP for small messages
- **Data format**: Apache Arrow columnar format throughout (zero serialization overhead)

### Dataflow specification

Dataflows are defined in YAML files. Nodes declare inputs (subscribing to other nodes' outputs) and outputs. Built-in timer nodes: `dora/timer/millis/<N>`, `dora/timer/hz/<N>`.

### Communication patterns

User-facing patterns (see `docs/patterns.md`):
- **Topic**: default pub/sub dataflow
- **Service**: request/reply via `request_id` metadata key; helpers: `send_service_request()`, `send_service_response()`
- **Action**: goal/feedback/result via `goal_id`/`goal_status` metadata keys; supports cancellation

Internal transport:
- **flume** channels: bounded MPSC for internal event routing
- **tokio** async runtime with full features
- **Zenoh**: pub-sub for remote/distributed nodes
- **UHL Clock** (`uhlc`): hybrid logical clock for distributed causality

## Pre-commit Quality Gates (MANDATORY)

**Every commit MUST pass these gates before pushing.** This is a BLOCKING REQUIREMENT — do not skip any step. Remote CI is slower than your laptop and has limited capacity; catching failures locally saves 5-15 minutes per round-trip.

> The bar below is the **Class A** (low-risk) baseline. Behavior
> changes and high-risk subsystem edits have additional requirements
> — see [`docs/agentic-qa-policy.md`](docs/agentic-qa-policy.md)
> (#1634) for the per-class expectations and the PR validation
> summary template.

### Step 1: Run /review

Run the `/review` skill on your changes before committing. This catches structural issues, security problems, and logic errors that tests and clippy miss.

### Step 2: Run /simplify

Run the `/simplify` skill to check for unnecessary complexity, code duplication, or inefficiency in the changed code.

### Step 3: Local CI (fmt + clippy + tests)

```bash
# 1. Format — CI rejects formatting diffs
cargo fmt --all -- --check

# 2. Clippy with warnings-as-errors — CI uses -D warnings
cargo clippy --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  -- -D warnings

# 3. Tests — at minimum the affected crate; full workspace before push
cargo test --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  --exclude dora-cli-api-python \
  --exclude dora-examples

# 4. Example targets — `cargo check --all` does NOT build `[[example]]`
# targets, so signature breaks in `dora_cli::build` / `dora_cli::run`
# can ship silently otherwise. CI gates on this (#1680).
cargo check --examples

# Quick single-crate check while iterating:
# cargo test -p <crate-name>
```

**Shortcut**: `make qa-fast` runs fmt + clippy + supply-chain audit + unwrap budget + typos in ~30 seconds. Use it as a sanity check before every commit. Then run the full `cargo test` above before `git push`.

If you add new example dataflows, also run `./scripts/smoke-all.sh --rust-only` to verify smoke tests pass.

### Enforcement

All three steps are required for every commit. Do NOT push without completing them. If any step fails, fix the issue before proceeding. The order matters: /review and /simplify catch design-level issues that are cheaper to fix before running the full test suite.

### Do NOT run the full QA suite on every commit/push

The deeper QA gates — `make qa-full`, `make qa-deep`, `make qa-nightly`, `make qa-release-gate`, `make qa-mutation-audit`, `make qa-examples`, `make qa-coverage`, `make qa-mutants`, `make qa-semver` — are designed for **focused investigation and pre-release gating**, not the per-commit loop. Do not run them routinely, and do not add them to remote CI:

- `make qa-full` (qa-fast + full tests + coverage) — ~5-10 minutes. Run before a significant push if you want extra confidence; coverage is too slow for every push.
- `make qa-deep` (qa-full + mutation testing + semver) — ~15 minutes. The **target** Tier 1 local gate. Today's CI PR gate only runs the fast subset (fmt/clippy/typos/audit/unwrap-budget + tests); `qa-deep` adds coverage, adversarial review, diff-scoped mutation, and semver — kept laptop-only because they're too slow for every PR (see `docs/plan-agentic-qa-strategy.md` §5). Alias: `make qa-tier1`, kept for back-compat.
- `make qa-nightly` (qa-deep + proptest@1000 + miri + example-smoke + ci-nightly-jobs) — ~3-4 hours. **Full parity with `.github/workflows/nightly.yml`** (#1707, #1710, #1716). After the #1716 rebalance, nightly.yml has **18 test jobs**. example-smoke covers the **4 example-backed** jobs (smoke-suite, log-sinks, service-action, streaming); `scripts/qa/ci-nightly-jobs.sh` drives the **14 remaining** with platform-aware dispatch (record-replay, cluster-smoke, topic-and-top, cpu-affinity [Linux], redb-backend, daemon-reconnect [Linux], state-reconstruction, test-cross-platform [macOS+Windows], examples, cli-tests, bench-example, cross-check, ros2-bridge [Linux+ROS2], msrv). A green local `qa-nightly` on platform X predicts a green CI nightly for platform X's jobs. **Requires both `uv` and Python 3.12** (both preflighted; the script fails fast with a specific install hint for whichever is missing — `curl -LsSf https://astral.sh/uv/install.sh \| sh` for uv, `uv python install 3.12` for the interpreter, matching the GHA `actions/setup-python` step at `.github/workflows/nightly.yml:56`). example-smoke creates a scratch venv at `target/qa-nightly-venv` and installs `-e apis/python/node` into it so Python nodes use the workspace bindings (not PyPI `dora-rs`, whose message format has drifted from the workspace — #1710). The CI-jobs script installs the CLI into a scratch dir (won't clobber `~/.cargo/bin/dora`) and bails if port 6013 is in use; cpu-affinity + daemon-reconnect skip on non-Linux. Skips miri if `cargo +nightly miri` isn't installed. **Does not** include full-repo mutation testing — that's split into `qa-mutation-audit` because it takes 10-18 hours on this workspace.
- `make qa-release-gate` (qa-deep + semver) — the automatable subset of Tier 3. The non-automatable parts (independent security audit, dogfood campaign, migration validation) are documented in `docs/plan-agentic-qa-strategy.md` §7 but not locally gateable.
- `make qa-mutation-audit` — ~10-18 hours. Full-repo `cargo-mutants` across 6 critical crates. Deliberate test-quality audit; run before a release or when investigating a specific crate, not every nightly.
- `make qa-examples` — ~15-20 min. Runs all **smoke-eligible** example dataflows end-to-end via `scripts/smoke-all.sh`. Skips examples that need CUDA, ROS2, webcam, multi-machine deploy, C/C++ toolchains, or interactive CLI (run `scripts/smoke-all.sh -h` to see the SKIP list). Orthogonal to the qa-fast/full/deep ladder: those targets `--exclude dora-examples` to keep per-commit / pre-push budgets tight. Run this when you want actual dataflows exercised (after touching node/operator APIs, CLI subcommands, or the descriptor surface). Pass flags via `ARGS`, e.g. `make qa-examples ARGS="--rust-only"` or `make qa-examples ARGS="-v"`.
- `make qa-coverage` — generates an lcov report locally. Run when you want to see what's covered.
- `make qa-mutants` — mutation testing, slow. Run when investigating test quality on a specific file or on the PR diff.
- `make qa-semver` — breaking-change check. Run before publishing to crates.io.

**Remote CI is deliberately kept lean** — only the fast hard gates (fmt, clippy, test on Linux, typos, supply chain audit, unwrap budget, E2E, contract tests, benchmark regression, license check) — so it stays fast (~30-45 min critical path) and runner capacity is not a bottleneck. Do not expand PR CI with slow jobs. See [`docs/qa-runbook.md`](docs/qa-runbook.md) for when and how to use each deep gate locally.

### Remote CI jobs

**PR CI (`.github/workflows/ci.yml`, ~30-45 min, #1716):** Linux-only. Blocks merge.
- `cargo fmt --all -- --check`
- `cargo clippy --all -- -D warnings` (excluding Python packages)
- `cargo test --all` on **ubuntu-latest only** (excluding Python packages)
- E2E tests: `ws-cli-e2e` + `fault-tolerance-e2e`
- Semantic contract tests (`tests/example-smoke.rs::contract_*`)
- Benchmark regression check (criterion baseline caching)
- Typo checking via `crate-ci/typos` (config: `_typos.toml`)
- Supply-chain audit (`cargo-audit` + `cargo-deny`)
- Unwrap-budget check (production `.unwrap()` / `.expect(` ratchet)
- License check (`cargo-lichking`)
- Rust toolchain pinned to 1.92 (see `.github/workflows/ci.yml`)

**Nightly CI (`.github/workflows/nightly.yml`, ~3-4h, daily 06:40 UTC):** Broader coverage, does NOT block PRs. Auto-files an issue on failure (`nightly-regression` label).
- Test on macOS + Windows
- Examples end-to-end (all 3 platforms, incl. C/C++/CMake/Arrow/multi-daemon)
- CLI Tests (all 3 platforms, incl. template projects + Python Dynamic Node + error-event)
- Bench Example (all 3 platforms)
- Cross-compilation matrix (8 targets)
- ROS2 bridge examples (Humble, ubuntu-22.04)
- MSRV check (`cargo-hack`)
- Plus the integration smokes: record/replay, cluster, topic-and-top, cpu-affinity, redb-backend, daemon-reconnect, state-reconstruction

**Developer guidance:** for non-Linux verification before merge, run `make qa-test` or `make qa-examples` locally; `make qa-nightly` covers the full nightly matrix (except ROS2) locally in ~3-4 hours.

## Test-Driven Development

All new features and bug fixes must follow the RED-GREEN-IMPROVE cycle:

1. **RED**: Write a failing test that defines the expected behavior
2. **GREEN**: Write the minimum code to make the test pass
3. **IMPROVE**: Refactor while keeping tests green

### Which test tier to write first

| Change type | Start with | File/location |
|-------------|-----------|---------------|
| New library function | Unit test | `#[cfg(test)]` in same file |
| New coordinator/daemon behavior | Integration test | `binaries/coordinator/tests/` |
| New CLI command or flag | Smoke test (networked) | `tests/example-smoke.rs` using `run_smoke_test()` |
| New dataflow feature | Smoke test (both modes) | `tests/example-smoke.rs` using both `run_smoke_test()` and `run_smoke_test_local()` |
| Bug fix | Regression test | Whichever tier reproduces the bug |
| New example dataflow | Smoke test entry | Add to `tests/example-smoke.rs` and `scripts/smoke-all.sh` |

### Workflow

```bash
# 1. Write failing test, verify it fails
cargo test -p <crate> <test_name>

# 2. Implement until test passes
cargo test -p <crate> <test_name>

# 3. Refactor, then verify everything still passes
cargo test -p <crate>
cargo clippy -p <crate> -- -D warnings
cargo fmt --all -- --check

# 4. Run smoke tests if the change touches CLI/coordinator/daemon
cargo test --test example-smoke -- --test-threads=1
```

### Smoke test patterns

Two helpers are available in `tests/example-smoke.rs`:

- `run_smoke_test(name, yaml, timeout)` -- networked: `dora up` + `dora start --detach` + poll + `dora stop` + `dora down`
- `run_smoke_test_local(name, yaml, stop_after_secs)` -- local: `dora run --stop-after`

New example dataflows should have tests in both modes. Use `Once` guards to share build steps across tests.

For quick local validation: `./scripts/smoke-all.sh`

### References

- Full testing guide: `docs/testing-guide.md`
- Smoke tests: `tests/example-smoke.rs`
- E2E tests: `tests/ws-cli-e2e.rs`
- Fault tolerance tests: `tests/fault-tolerance-e2e.rs`

## Conventions

- Format with `rustfmt` default settings before submitting PRs
- Discuss non-trivial changes in a GitHub issue or Discord first
- Don't fix unrelated warnings in PRs
- Python node API is distributed via PyPI (`dora-rs`); CLI is distributed via crates.io and GitHub Releases
- Release profile `dist` uses fat LTO
