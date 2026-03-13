# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Adora (AI-Dora, Agentic Dataflow-Oriented Robotic Architecture) is a 100% Rust framework for building real-time robotics and AI applications. It provides 10-17x faster latency than ROS2 via zero-copy shared memory and Apache Arrow data format. Supports Rust, Python (PyO3), C, and C++ nodes.

## Build Commands

```bash
# Build all (Python packages require maturin, exclude them for normal builds)
cargo build --all --exclude adora-node-api-python --exclude adora-operator-api-python --exclude adora-ros2-bridge-python

# Build specific package
cargo build -p adora-cli
cargo build -p adora-daemon

# Check all
cargo check --all

# Test all (excluding Python)
cargo test --all --exclude adora-node-api-python --exclude adora-operator-api-python --exclude adora-ros2-bridge-python

# Test single package
cargo test -p adora-core

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
adora run examples/rust-dataflow/dataflow.yml
adora run examples/python-dataflow/dataflow.yml --uv --stop-after 10s
```

## Workspace Layout

- **Rust edition 2024, MSRV 1.85.0, version 0.1.0** (all crates share workspace version)
- Python packages use PyO3 0.23 and are built with **maturin**, not cargo

### Key crates

| Path | Crate | Role |
|------|-------|------|
| `binaries/cli` | adora-cli | CLI binary (`adora` command) - build, run, stop dataflows |
| `binaries/daemon` | adora-daemon | Spawns nodes, manages local shared-memory/TCP communication |
| `binaries/coordinator` | adora-coordinator | Orchestrates distributed multi-daemon deployments |
| `binaries/runtime` | adora-runtime | In-process operator execution runtime |
| `libraries/message` | adora-message | All inter-component message types and protocol definitions |
| `libraries/core` | adora-core | Dataflow descriptor parsing, build utilities, Zenoh config |
| `libraries/shared-memory-server` | shared-memory-server | Zero-copy IPC for large messages (>4KB) |
| `apis/rust/node` | adora-node-api | Rust API for writing custom nodes |
| `apis/rust/operator` | adora-operator-api | Rust API for writing in-process operators |
| `apis/python/node` | adora-node-api-python | Python node API (PyO3) |

## Architecture

```
CLI  -->  Coordinator  -->  Daemon(s)  -->  Nodes / Operators
              (distributed)    (per machine)    (user code)
```

- **CLI <-> Coordinator**: TCP request-reply for build/run/stop commands
- **Coordinator <-> Daemon**: TCP for node spawning and dataflow lifecycle
- **Daemon <-> Daemon**: Zenoh for distributed cross-machine communication
- **Daemon <-> Node**: Shared memory for messages >4KB (zero-copy), TCP for small messages
- **Data format**: Apache Arrow columnar format throughout (zero serialization overhead)

### Dataflow specification

Dataflows are defined in YAML files. Nodes declare inputs (subscribing to other nodes' outputs) and outputs. Built-in timer nodes: `adora/timer/millis/<N>`, `adora/timer/hz/<N>`.

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

## CI Checks

CI runs on Ubuntu, macOS, and Windows. Key jobs:
- `cargo check --all`
- `cargo build` / `cargo test` (excluding Python packages)
- `cargo clippy --all`
- `cargo fmt --all -- --check`
- Typo checking via `crate-ci/typos` (config: `_typos.toml`)
- License compatibility via `cargo-lichking`
- Cross-compilation checks for arm32, arm64, musl, mingw targets
- MSRV check via `cargo hack check --rust-version`

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

- `run_smoke_test(name, yaml, timeout)` -- networked: `adora up` + `adora start --detach` + poll + `adora stop` + `adora down`
- `run_smoke_test_local(name, yaml, stop_after_secs)` -- local: `adora run --stop-after`

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
- Python packages are distributed via PyPI (`adora-rs-cli`, `adora-node-api`)
- Release profile `dist` uses thin LTO
