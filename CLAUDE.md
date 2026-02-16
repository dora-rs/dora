# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Adora (AI-Dora, Dataflow-Oriented Robotic Architecture) is a 100% Rust framework for building real-time robotics and AI applications. It provides 10-17x faster latency than ROS2 via zero-copy shared memory and Apache Arrow data format. Supports Rust, Python (PyO3), C, and C++ nodes.

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

- **Rust edition 2024, MSRV 1.85.0, version 0.4.1** (adora-message is independently versioned at 0.7.0)
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

## Conventions

- Format with `rustfmt` default settings before submitting PRs
- Discuss non-trivial changes in a GitHub issue or Discord first
- Don't fix unrelated warnings in PRs
- Python packages are distributed via PyPI (`adora-rs-cli`, `adora-node-api`)
- Release profile `dist` uses thin LTO
