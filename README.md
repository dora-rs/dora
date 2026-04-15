[English](README.md) | [简体中文](README.zh-CN.md)

# Dora

**Agentic Dataflow-Oriented Robotic Architecture** -- a 100% Rust framework for building real-time robotics and AI applications.

[**User Guide**](https://dora-rs.ai/dora/) | [**用户指南 (中文)**](https://dora-rs.ai/dora/zh-CN/)

> Built and maintained with **agentic engineering** -- code generation, reviews, refactoring, testing, and commits are driven by autonomous AI agents.

---

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [CLI Commands](#cli-commands)
- [Dataflow Configuration](#dataflow-configuration)
- [Architecture](#architecture)
- [Language Support](#language-support)
- [Examples](#examples)
- [Development](#development)
- [Contributing](#contributing)
- [License](#license)

## Features

### Performance

- **10-17x faster than ROS2 Python** -- 100% Rust internals with zero-copy shared memory IPC for messages >4KB, flat latency from 4KB to 4MB payloads
- **Zenoh SHM data plane** -- nodes publish directly via [Zenoh](https://zenoh.io/) shared memory, bypassing the daemon for 35% lower latency and 3-10x higher throughput on large payloads; automatic network fallback for cross-machine
- **Apache Arrow native** -- columnar memory format end-to-end with zero serialization overhead; optional [Arrow IPC framing](docs/yaml-spec.md) for self-describing wire format; shared across all language bindings
- **Non-blocking event loop** -- Zenoh publishes offloaded to a dedicated drain task; control commands respond in <500ms even under high data throughput

### Developer experience

- **Single CLI, full lifecycle** -- `dora run` for local dev, `dora up/start` for distributed prod, plus build, logs, monitoring, record/replay all from one tool
- **Declarative YAML dataflows** -- define pipelines as directed graphs, connect nodes through typed inputs/outputs, optional [type annotations](docs/types.md) with static validation, override with environment variables
- **Multi-language nodes** -- write nodes in Rust, Python, C, or C++ with native APIs (not wrappers); mix languages freely in one dataflow
- **[Reusable modules](docs/modules.md)** -- compose sub-graphs as standalone YAML files with typed inputs/outputs, parameters, optional ports, and nested composition (compile-time expansion, zero runtime overhead)
- **Hot reload** -- live-reload Python operators without restarting the dataflow
- **Programmatic builder** -- construct dataflows in Python code as an alternative to YAML

### Production readiness

- **Fault tolerance** -- per-node restart policies (never/on-failure/always), exponential backoff, health monitoring, circuit breakers with configurable input timeouts
- **Distributed by default** -- local shared memory between co-located nodes, automatic [Zenoh](https://zenoh.io/) pub-sub for cross-machine communication, SSH-based [cluster management](docs/distributed-deployment.md) with label scheduling, rolling upgrades, and auto-recovery
- **Coordinator HA** -- persistent redb-backed state store (default), daemon auto-reconnect with exponential backoff, dataflow records survive coordinator restart (running dataflow reclaim-across-restart is partial, see the open issue tracker)
- **Dynamic topology** -- add and remove nodes from running dataflows via CLI (`dora node add/remove/connect/disconnect`) without restarting
- **Soft real-time** -- optional `--rt` flag for mlockall + SCHED_FIFO; per-node `cpu_affinity` pinning in YAML; comprehensive [tuning guide](docs/realtime-tuning.md) for memory locking, kernel params, and container deployment
- **OpenTelemetry** -- built-in structured logging with rotation/routing, metrics, distributed tracing, and zero-setup trace viewing via CLI

### Debugging and observability

- **Record/replay** -- capture dataflow messages to `.adorec` files, replay offline at any speed with node substitution for regression testing
- **Topic inspection** -- `topic echo` to print live data, `topic hz` TUI for frequency analysis, `topic info` for schema and bandwidth
- **Resource monitoring** -- `dora top` TUI showing per-node CPU, memory, queue depth, network I/O, restart count, and health status across all machines; `--once` flag for scriptable JSON snapshots
- **Trace inspection** -- `trace list` and `trace view` for viewing coordinator spans without external infrastructure
- **Dataflow visualization** -- generate interactive HTML or Mermaid graphs from YAML descriptors

### Ecosystem

- **Communication patterns** -- built-in [service (request/reply)](docs/patterns.md#2-service-requestreply), [action (goal/feedback/result)](docs/patterns.md#3-action-goalfeedbackresult), and [streaming (session/segment/chunk)](docs/patterns.md#4-streaming-sessionsegmentchunk) patterns via well-known metadata keys; no daemon or YAML changes required
- **ROS2 bridge** -- bidirectional interop with ROS2 topics, services, and actions; QoS mapping; Arrow-native type conversion
- **Pre-packaged nodes** -- [node hub](https://github.com/dora-rs/dora-hub/) with ready-made nodes for cameras, YOLO, LLMs, TTS, and more
- **In-process operators** -- lightweight functions that run inside a shared runtime, avoiding per-node process overhead for simple transformations

## Installation

### From crates.io (recommended)

```bash
cargo install dora-cli           # CLI (dora command)
pip install dora-rs              # Python node/operator API
```

### From source

```bash
git clone https://github.com/dora-rs/adora.git
cd dora
cargo build --release -p dora-cli
PATH=$PATH:$(pwd)/target/release

# Python API (requires maturin >= 1.8: pip install maturin)
# Must run from the package directory for dependency resolution
cd apis/python/node && maturin develop --uv && cd ../../..
```

### Platform installers

**macOS / Linux:**

```bash
curl --proto '=https' --tlsv1.2 -LsSf \
  https://github.com/dora-rs/adora/releases/latest/download/dora-cli-installer.sh | sh
```

**Windows:**

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://github.com/dora-rs/adora/releases/latest/download/dora-cli-installer.ps1 | iex"
```

### Build features

| Feature | Description | Default |
|---------|-------------|---------|
| `tracing` | OpenTelemetry tracing support | Yes |
| `metrics` | OpenTelemetry metrics collection | Yes |
| `python` | Python operator support (PyO3) | Yes |
| `redb-backend` | Persistent coordinator state (redb) | Yes |

```bash
cargo install dora-cli --features redb-backend
```

## Quick Start

### 1. Run a Python dataflow

> **Important:** The PyPI package is **`dora-rs`**, not `dora`. The import name
> is `dora` (`from dora import Node`), but `pip install dora` installs an
> unrelated package.

```bash
cargo install dora-cli            # or use install script below
pip install dora-rs numpy pyarrow
git clone https://github.com/dora-rs/adora.git && cd dora
dora run examples/python-dataflow/dataflow.yml
```

This runs a sender -> transformer -> receiver pipeline. Here's what the Python node code looks like:

```python
# sender.py -- sends 100 messages
from dora import Node
import pyarrow as pa

node = Node()
for i in range(100):
    node.send_output("message", pa.array([i]))
```

```python
# receiver.py -- receives and prints messages
from dora import Node

node = Node()
for event in node:
    if event["type"] == "INPUT":
        print(f"Got {event['id']}: {event['value'].to_pylist()}")
    elif event["type"] == "STOP":
        break
```

See the [Python Getting Started Guide](docs/python-guide.md) for a full tutorial, or the [Python API Reference](docs/api-python.md) for complete API docs.

### 2. Run a Rust dataflow

```bash
cd examples/rust-dataflow
dora run dataflow.yml
```

### 3. Distributed mode (ad-hoc)

```bash
# Terminal 1: start coordinator + daemon
dora up

# Terminal 2: start a dataflow (--debug enables topic inspection)
dora start dataflow.yml --attach --debug

# Terminal 3: monitor
dora list
dora logs <dataflow-id>
dora top

# Stop or restart
dora stop <dataflow-id>
dora restart --name <name>
dora down
```

### 4. Managed cluster

```bash
# Bring up a multi-machine cluster from a config file
dora cluster up cluster.yml

# Start a dataflow across the cluster
dora start dataflow.yml --name my-app --attach

# Check cluster health
dora cluster status

# Tear down
dora cluster down
```

See the [Distributed Deployment Guide](docs/distributed-deployment.md) for cluster.yml configuration, label scheduling, systemd services, rolling upgrades, and operational runbooks.

## CLI Commands

### Lifecycle

| Command | Description |
|---------|-------------|
| `dora run <PATH>` | Run a dataflow locally (no coordinator/daemon needed) |
| `dora up` | Start coordinator and daemon in local mode |
| `dora down` | Tear down coordinator and daemon |
| `dora build <PATH>` | Run build commands from a dataflow descriptor |
| `dora start <PATH>` | Start a dataflow on a running coordinator |
| `dora stop <ID>` | Stop a running dataflow |
| `dora restart <ID>` | Restart a running dataflow (stop + re-start) |

### Monitoring

| Command | Description |
|---------|-------------|
| `dora list` | List running dataflows (alias: `ps`) |
| `dora logs <ID>` | Show logs for a dataflow or node |
| `dora top` | Real-time resource monitor (TUI); also `dora inspect top` |
| `dora topic list` | List topics in a dataflow |
| `dora topic hz <TOPIC>` | Measure topic publish frequency (TUI) |
| `dora topic echo <TOPIC>` | Print topic messages to stdout |
| `dora topic info <TOPIC>` | Show topic type and metadata |
| `dora node list` | List nodes in a dataflow |
| `dora node info <NODE>` | Show detailed node status, inputs, outputs, and metrics |
| `dora node add --from-yaml <FILE>` | Add a node to a running dataflow |
| `dora node remove <NODE>` | Remove a node from a running dataflow |
| `dora node connect <SRC> <DST>` | Add a live mapping between nodes |
| `dora node disconnect <SRC> <DST>` | Remove a live mapping between nodes |
| `dora node restart <NODE>` | Restart a single node within a running dataflow |
| `dora node stop <NODE>` | Stop a single node within a running dataflow |
| `dora topic pub <TOPIC> <DATA>` | Publish JSON data to a topic |
| `dora param list <NODE>` | List runtime parameters for a node |
| `dora param get <NODE> <KEY>` | Get a runtime parameter value |
| `dora param set <NODE> <KEY> <VALUE>` | Set a runtime parameter (JSON value) |
| `dora param delete <NODE> <KEY>` | Delete a runtime parameter |
| `dora trace list` | List recent traces captured by the coordinator |
| `dora trace view <ID>` | View spans for a specific trace (supports prefix matching) |
| `dora record <PATH>` | Record dataflow messages to `.adorec` file |
| `dora replay <FILE>` | Replay recorded messages from `.adorec` file |

### Cluster management

| Command | Description |
|---------|-------------|
| `dora cluster up <PATH>` | Bring up a cluster from a cluster.yml file |
| `dora cluster status` | Show connected daemons and active dataflows |
| `dora cluster down` | Tear down the cluster |
| `dora cluster install <PATH>` | Install daemons as systemd services |
| `dora cluster uninstall <PATH>` | Remove systemd services |
| `dora cluster upgrade <PATH>` | Rolling upgrade: SCP binary + restart per-machine |
| `dora cluster restart <NAME>` | Restart a dataflow by name or UUID |

### Setup and utilities

| Command | Description |
|---------|-------------|
| `dora doctor` | Diagnose environment, connectivity, and dataflow health |
| `dora status` | Check system health (alias: `check`) |
| `dora new` | Generate a new project or node |
| `dora graph <PATH>` | Visualize a dataflow (Mermaid or HTML) |
| `dora expand <PATH>` | Expand module references and print flat YAML |
| `dora validate <PATH>` | Validate dataflow YAML and check [type annotations](docs/types.md) |
| `dora system` | System management (daemon/coordinator control) |
| `dora completion <SHELL>` | Generate shell completions |
| `dora self update` | Update dora CLI |

For full CLI documentation, see [docs/cli.md](docs/cli.md). For distributed deployment, see [docs/distributed-deployment.md](docs/distributed-deployment.md).

## Dataflow Configuration

Dataflows are defined in YAML. Each node declares its binary/script, inputs, and outputs:

```yaml
nodes:
  - id: camera
    build: pip install opencv-video-capture
    path: opencv-video-capture
    inputs:
      tick: dora/timer/millis/20
    outputs:
      - image
    env:
      CAPTURE_PATH: 0
      IMAGE_WIDTH: 640
      IMAGE_HEIGHT: 480

  - id: object-detection
    build: pip install dora-yolo
    path: dora-yolo
    inputs:
      image: camera/image
    outputs:
      - bbox

  - id: plot
    build: pip install dora-rerun
    path: dora-rerun
    inputs:
      image: camera/image
      boxes2d: object-detection/bbox
```

**Built-in timer nodes:** `dora/timer/millis/<N>` and `dora/timer/hz/<N>`.

**Input format:** `<node-id>/<output-name>` to subscribe to another node's output. Long form supports `queue_size`, `queue_policy` (`drop_oldest` or `backpressure`), and `input_timeout`. See the [YAML Specification](docs/yaml-spec.md) for details.

**Type annotations:** Optionally annotate ports with type URNs for static and runtime validation. See the [Type Annotations Guide](docs/types.md) for the full type library.

```yaml
nodes:
  - id: camera
    path: camera.py
    outputs:
      - image
    output_types:
      image: std/media/v1/Image
```

```bash
dora validate dataflow.yml                        # static check (warnings)
dora validate --strict-types dataflow.yml         # fail on warnings (CI)
dora build dataflow.yml --strict-types            # type check during build
DORA_RUNTIME_TYPE_CHECK=warn dora run dataflow.yml  # runtime check
```

**Modules:** Extract reusable sub-graphs into separate files with `module:` instead of `path:`. See the [Modules Guide](docs/modules.md) for details.

```yaml
nodes:
  - id: nav_stack
    module: modules/navigation.module.yml
    inputs:
      goal_pose: localization/goal
```

## Architecture

```
CLI  -->  Coordinator  -->  Daemon(s)  -->  Nodes / Operators
             (orchestration)  (per machine)    (user code)
```

| Layer | Protocol | Purpose |
|-------|----------|---------|
| CLI <-> Coordinator | WebSocket (port 6013) | Build, run, stop commands |
| Coordinator <-> Daemon | WebSocket | Node spawning, dataflow lifecycle |
| Daemon <-> Daemon | Zenoh | Distributed cross-machine communication |
| Node <-> Node | Zenoh SHM | Direct zero-copy data plane for messages >4KB |
| Daemon <-> Node | Shared memory / TCP | Control plane + small message delivery |

### Key components

- **Coordinator** -- orchestrates dataflow lifecycle across daemons. Persistent redb state store by default; daemons auto-reconnect on coordinator restart.
- **Daemon** -- spawns and manages nodes on a single machine. Routes messages and manages Zenoh SHM data plane.
- **Runtime** -- in-process operator execution engine. Operators run inside the runtime process, avoiding per-operator process overhead.
- **Nodes** -- standalone processes that communicate via inputs/outputs. Written in Rust, Python, C, or C++.
- **Operators** -- lightweight functions that run inside the runtime. Faster than nodes for simple transformations.

### Workspace layout

```
binaries/
  cli/                  # dora CLI binary
  coordinator/          # Orchestration service
  daemon/               # Node manager + IPC
  runtime/              # In-process operator runtime
  ros2-bridge-node/     # ROS2 bridge binary
  record-node/          # Dataflow message recorder
  replay-node/          # Recorded message replayer
libraries/
  core/                 # Descriptor parsing, build utilities
  message/              # Inter-component message types
  shared-memory-server/ # Zero-copy IPC
  arrow-convert/        # Arrow data conversion
  recording/            # .adorec recording format
  log-utils/            # Log parsing, merging, formatting
  coordinator-store/    # Persistent coordinator state (redb)
  extensions/
    telemetry/          # OpenTelemetry tracing + metrics
    ros2-bridge/        # ROS2 interop (bridge, msg-gen, arrow, python)
    download/           # Download utilities
apis/
  rust/node/            # Rust node API (dora-node-api)
  rust/operator/        # Rust operator API (dora-operator-api)
  python/node/          # Python node API (PyO3)
  python/operator/      # Python operator API (PyO3)
  python/cli/           # Python CLI interface
  c/node/               # C node API
  c/operator/           # C operator API
  c++/node/             # C++ node API (CXX bridge)
  c++/operator/         # C++ operator API (CXX bridge)
examples/               # Example dataflows
```

## Language Support

| Language | Node API | Operator API | Docs | Status |
|----------|----------|--------------|------|--------|
| Rust | `dora-node-api` | `dora-operator-api` | [API Reference](docs/api-rust.md) | First-class |
| Python >= 3.8 | `pip install dora-rs` | included | [Getting Started](docs/python-guide.md), [API Reference](docs/api-python.md) | First-class |
| C | `dora-node-api-c` | `dora-operator-api-c` | [API Reference](docs/api-c.md) | Supported |
| C++ | `dora-node-api-cxx` | `dora-operator-api-cxx` | [API Reference](docs/api-cxx.md) | Supported |
| ROS2 >= Foxy | `dora-ros2-bridge` | -- | [Bridge Guide](docs/ros2-bridge.md) | Experimental |

### Platform support

| Platform | Rust / Python | C / C++ templates |
|----------|---------------|-------------------|
| Linux (x86_64, ARM64, ARM32) | First-class | First-class (CI-gated) |
| macOS (ARM64) | First-class | Best effort (not CI-gated) |
| Windows (x86_64) | Best effort | Best effort (not CI-gated) |
| WSL (x86_64) | Best effort | Best effort (not CI-gated) |

C/C++ template tests (`dora new --lang c/cxx` + CMake build) run in CI on Linux only.
`dora new --lang rust/python` is smoke-tested on all three platforms, but the C/C++
variants are not. macOS and Windows C/C++ regressions are caught by users rather
than CI; report them via GitHub issues and we will promote the gate. See
[`docs/testing-matrix.md`](docs/testing-matrix.md#platform-parity) for the full rationale.

## Examples

### Core language examples

| Example | Language | Description |
|---------|----------|-------------|
| [rust-dataflow](examples/rust-dataflow) | Rust | Basic Rust node pipeline |
| [python-dataflow](examples/python-dataflow) | Python | Python sender/transformer/receiver |
| [python-operator-dataflow](examples/python-operator-dataflow) | Python | Python operators (in-process) |
| [python-dataflow-builder](examples/python-dataflow-builder) | Python | Pythonic imperative API |
| [c-dataflow](examples/c-dataflow) | C | C node example |
| [c++-dataflow](examples/c++-dataflow) | C++ | C++ node example |
| [c++-arrow-dataflow](examples/c++-arrow-dataflow) | C++ | C++ with Arrow data |
| [cmake-dataflow](examples/cmake-dataflow) | C/C++ | CMake-based build |

### Composition

| Example | Language | Description |
|---------|----------|-------------|
| [module-dataflow](examples/module-dataflow) | Python | Reusable module composition |
| [typed-dataflow](examples/typed-dataflow) | Python | Type annotations with `dora validate` |

### Communication patterns

| Example | Language | Description |
|---------|----------|-------------|
| [service-example](examples/service-example) | Rust | Request/reply with `request_id` correlation |
| [action-example](examples/action-example) | Rust | Goal/feedback/result with cancellation |
| [streaming-example](examples/streaming-example) | Python | Token-by-token generation with session/seq/fin metadata |

See [docs/patterns.md](docs/patterns.md) for the full guide.

### Dynamic topology

| Example | Language | Description |
|---------|----------|-------------|
| [dynamic-add-remove](examples/dynamic-add-remove) | Python | Add/remove nodes from running dataflows |
| [dynamic-agent-tools](examples/dynamic-agent-tools) | Python | AI agent with dynamically-added tools |

### Advanced patterns

| Example | Language | Description |
|---------|----------|-------------|
| [python-async](examples/python-async) | Python | Async Python nodes |
| [python-concurrent-rw](examples/python-concurrent-rw) | Python | Concurrent read-write patterns |
| [python-multiple-arrays](examples/python-multiple-arrays) | Python | Multi-array handling |
| [python-drain](examples/python-drain) | Python | Event draining patterns |
| [multiple-daemons](examples/multiple-daemons) | Rust | Distributed multi-daemon setup |
| [rust-dataflow-git](examples/rust-dataflow-git) | Rust | Git-based dataflow loading |
| [rust-dataflow-url](examples/rust-dataflow-url) | Rust | URL-based dataflow loading |

### Logging

| Example | Language | Description |
|---------|----------|-------------|
| [python-logging](examples/python-logging) | Python | Python logging integration |
| [python-log](examples/python-log) | Python | Basic Python log output |
| [log-sink-tcp](examples/log-sink-tcp) | YAML | TCP-based log sink |
| [log-sink-file](examples/log-sink-file) | YAML | File-based log sink |
| [log-sink-alert](examples/log-sink-alert) | YAML | Alert-based log sink |
| [log-aggregator](examples/log-aggregator) | Python | Centralized log aggregation via `dora/logs` |

### Performance

| Example | Language | Description |
|---------|----------|-------------|
| [benchmark](examples/benchmark) | Rust/Python | Latency and throughput benchmark |
| [ros2-comparison](examples/ros2-comparison) | Python | Dora vs ROS2 comparison |
| [cuda-benchmark](examples/cuda-benchmark) | Rust/CUDA | GPU zero-copy benchmark |

### ROS2 integration

| Example | Description |
|---------|-------------|
| [ros2-bridge/rust](examples/ros2-bridge/rust) | Rust ROS2 topics, services, actions |
| [ros2-bridge/python](examples/ros2-bridge/python) | Python ROS2 integration |
| [ros2-bridge/c++](examples/ros2-bridge/c++) | C++ ROS2 integration |
| [ros2-bridge/yaml-bridge](examples/ros2-bridge/yaml-bridge) | YAML-based ROS2 topic bridge |
| [ros2-bridge/yaml-bridge-service](examples/ros2-bridge/yaml-bridge-service) | YAML ROS2 service bridge |
| [ros2-bridge/yaml-bridge-action](examples/ros2-bridge/yaml-bridge-action) | YAML ROS2 action client |
| [ros2-bridge/yaml-bridge-action-server](examples/ros2-bridge/yaml-bridge-action-server) | YAML ROS2 action server |

## Development

**Rust edition 2024, MSRV 1.85.0, workspace version 0.2.0.**

### Build

```bash
# Build all (excluding Python packages which require maturin)
cargo build --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python

# Build specific package
cargo build -p dora-cli
```

### Test

```bash
# Run all tests
cargo test --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python

# Test single package
cargo test -p dora-core

# Smoke tests (requires coordinator/daemon)
cargo test --test example-smoke -- --test-threads=1
```

### Lint and format

```bash
cargo clippy --all
cargo fmt --all -- --check
```

### Run examples

```bash
cargo run --example rust-dataflow
cargo run --example python-dataflow
cargo run --example benchmark --release
```

## Quality assurance

Dora ships with a three-tier QA system designed for AI-authored code. Everything runs locally first; CI mirrors the same scripts.

```bash
make qa-install     # one-time: install cargo-audit, cargo-deny, cargo-llvm-cov, cargo-mutants, cargo-semver-checks
make qa-fast        # ~15s  -- fmt + clippy + audit + unwrap-budget (pre-commit)
make qa-full        # ~5-10 min -- qa-fast + tests + coverage (pre-push)
make qa-tier1       # ~1-2 hrs  -- qa-full + mutation testing + semver (pre-release)
```

**Gates in place:**

- **Supply chain** -- `cargo-audit` + `cargo-deny` for CVEs, license policy, dependency bans
- **Unwrap ratchet** -- counts `.unwrap()` / `.expect(` in production code; can only go down (`.unwrap-budget`)
- **Coverage** -- `cargo-llvm-cov` with diff-coverage gate (70% on PR-touched lines)
- **Mutation testing** -- `cargo-mutants` against critical crates (library crates at package scope, binary crates with `test_workspace = true`)
- **Property testing** -- `proptest` on wire-protocol types; catches edge cases unit tests miss
- **Miri** -- UB detection on pure-Rust unsafe hotspots (e.g., `dora-core::metadata`)
- **SemVer check** -- `cargo-semver-checks` against the last git tag
- **Adversarial LLM review** -- `scripts/qa/adversarial.sh` runs a *different* model on your diff to catch single-model blind spots (local today; CI pending API secret)

**Reference docs:**

- [QA Runbook](docs/qa-runbook.md) -- day-to-day command reference, failure modes, and fixes
- [Agentic QA Strategy](docs/plan-agentic-qa-strategy.md) -- full three-tier design and rationale
- [POC Report](docs/qa-poc-report-2026-04-09.md) -- case studies, metrics, lessons learned, recommendations for the wider ecosystem

## Contributing

We welcome contributors of all experience levels. See the [contributing guide](CONTRIBUTING.md) to get started.

### Communication

- [Discord](https://discord.gg/6eMGGutkfE)
- [GitHub Discussions](https://github.com/orgs/dora-rs/discussions)

## AI-Assisted Development

This repository is maintained with AI-assisted agentic engineering. Code generation, reviews, refactoring, testing, and commits are driven by autonomous AI agents -- enabling faster iteration and higher code quality at scale.

## License

Apache-2.0. See [NOTICE.md](NOTICE.md) for details.
