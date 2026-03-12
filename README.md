[English](README.md) | [简体中文](README.zh-CN.md)

# Adora

**Agentic Dataflow-Oriented Robotic Architecture** -- a 100% Rust framework for building real-time robotics and AI applications.

[**User Guide**](https://dora-rs.ai/adora/) | [**用户指南 (中文)**](https://dora-rs.ai/adora/zh-CN/)

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
- **Apache Arrow native** -- columnar memory format end-to-end with zero serialization overhead; shared across all language bindings

### Developer experience

- **Single CLI, full lifecycle** -- `adora run` for local dev, `adora up/start` for distributed prod, plus build, logs, monitoring, record/replay all from one tool
- **Declarative YAML dataflows** -- define pipelines as directed graphs, connect nodes through typed inputs/outputs, optional [type annotations](docs/types.md) with static validation, override with environment variables
- **Multi-language nodes** -- write nodes in Rust, Python, C, or C++ with native APIs (not wrappers); mix languages freely in one dataflow
- **[Reusable modules](docs/modules.md)** -- compose sub-graphs as standalone YAML files with typed inputs/outputs, parameters, optional ports, and nested composition (compile-time expansion, zero runtime overhead)
- **Hot reload** -- live-reload Python operators without restarting the dataflow
- **Programmatic builder** -- construct dataflows in Python code as an alternative to YAML

### Production readiness

- **Fault tolerance** -- per-node restart policies (never/on-failure/always), exponential backoff, health monitoring, circuit breakers with configurable input timeouts
- **Distributed by default** -- local shared memory between co-located nodes, automatic [Zenoh](https://zenoh.io/) pub-sub for cross-machine communication, SSH-based [cluster management](docs/distributed-deployment.md) with label scheduling, rolling upgrades, and auto-recovery
- **Coordinator persistence** -- optional redb-backed state store survives coordinator crashes and restarts
- **OpenTelemetry** -- built-in structured logging with rotation/routing, metrics, distributed tracing, and zero-setup trace viewing via CLI

### Debugging and observability

- **Record/replay** -- capture dataflow messages to `.adorec` files, replay offline at any speed with node substitution for regression testing
- **Topic inspection** -- `topic echo` to print live data, `topic hz` TUI for frequency analysis, `topic info` for schema and bandwidth
- **Resource monitoring** -- `adora top` TUI showing per-node CPU, memory, queue depth, network I/O, restart count, and health status across all machines; `--once` flag for scriptable JSON snapshots
- **Trace inspection** -- `trace list` and `trace view` for viewing coordinator spans without external infrastructure
- **Dataflow visualization** -- generate interactive HTML or Mermaid graphs from YAML descriptors

### Ecosystem

- **Communication patterns** -- built-in [service (request/reply)](docs/patterns.md#2-service-requestreply) and [action (goal/feedback/result)](docs/patterns.md#3-action-goalfeedbackresult) patterns via well-known metadata keys; no daemon or YAML changes required
- **ROS2 bridge** -- bidirectional interop with ROS2 topics, services, and actions; QoS mapping; Arrow-native type conversion
- **Pre-packaged nodes** -- [node hub](https://github.com/dora-rs/dora-hub/) with ready-made nodes for cameras, YOLO, LLMs, TTS, and more
- **In-process operators** -- lightweight functions that run inside a shared runtime, avoiding per-node process overhead for simple transformations

## Installation

### From PyPI (recommended)

```bash
pip install adora-rs-cli          # CLI (adora command)
pip install adora-rs              # Python node/operator API
```

### From crates.io

```bash
cargo install adora-cli
```

### From source

```bash
git clone https://github.com/adora-rs/adora.git
cd adora
cargo build --release -p adora-cli
PATH=$PATH:$(pwd)/target/release

# Python API (requires maturin >= 1.8: pip install maturin)
# Must run from the package directory for dependency resolution
cd apis/python/node && maturin develop --uv && cd ../../..
```

### Platform installers

**macOS / Linux:**

```bash
curl --proto '=https' --tlsv1.2 -LsSf \
  https://github.com/adora-rs/adora/releases/latest/download/adora-cli-installer.sh | sh
```

**Windows:**

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://github.com/adora-rs/adora/releases/latest/download/adora-cli-installer.ps1 | iex"
```

### Build features

| Feature | Description | Default |
|---------|-------------|---------|
| `tracing` | OpenTelemetry tracing support | Yes |
| `metrics` | OpenTelemetry metrics collection | No |
| `python` | Python operator support (PyO3) | No |
| `redb-backend` | Persistent coordinator state (redb) | No |
| `prometheus` | Prometheus `/metrics` endpoint on coordinator | No |

```bash
cargo install adora-cli --features redb-backend
```

## Quick Start

### 1. Run a Python dataflow

> **Important:** The PyPI package is **`adora-rs`**, not `adora`. The import name
> is `adora` (`from adora import Node`), but `pip install adora` installs an
> unrelated package.

```bash
pip install adora-rs-cli adora-rs
git clone https://github.com/adora-rs/adora.git && cd adora
adora run examples/python-dataflow/dataflow.yml
```

This runs a sender -> transformer -> receiver pipeline. Here's what the Python node code looks like:

```python
# sender.py -- sends 100 messages
from adora import Node
import pyarrow as pa

node = Node()
for i in range(100):
    node.send_output("message", pa.array([i]))
```

```python
# receiver.py -- receives and prints messages
from adora import Node

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
adora run dataflow.yml
```

### 3. Distributed mode (ad-hoc)

```bash
# Terminal 1: start coordinator + daemon
adora up

# Terminal 2: start a dataflow (--debug enables topic inspection)
adora start dataflow.yml --attach --debug

# Terminal 3: monitor
adora list
adora logs <dataflow-id>
adora top

# Stop or restart
adora stop <dataflow-id>
adora restart --name <name>
adora down
```

### 4. Managed cluster

```bash
# Bring up a multi-machine cluster from a config file
adora cluster up cluster.yml

# Start a dataflow across the cluster
adora start dataflow.yml --name my-app --attach

# Check cluster health
adora cluster status

# Tear down
adora cluster down
```

See the [Distributed Deployment Guide](docs/distributed-deployment.md) for cluster.yml configuration, label scheduling, systemd services, rolling upgrades, and operational runbooks.

## CLI Commands

### Lifecycle

| Command | Description |
|---------|-------------|
| `adora run <PATH>` | Run a dataflow locally (no coordinator/daemon needed) |
| `adora up` | Start coordinator and daemon in local mode |
| `adora down` | Tear down coordinator and daemon |
| `adora build <PATH>` | Run build commands from a dataflow descriptor |
| `adora start <PATH>` | Start a dataflow on a running coordinator |
| `adora stop <ID>` | Stop a running dataflow |
| `adora restart <ID>` | Restart a running dataflow (stop + re-start) |

### Monitoring

| Command | Description |
|---------|-------------|
| `adora list` | List running dataflows (alias: `ps`) |
| `adora logs <ID>` | Show logs for a dataflow or node |
| `adora top` | Real-time resource monitor (TUI); also `adora inspect top` |
| `adora topic list` | List topics in a dataflow |
| `adora topic hz <TOPIC>` | Measure topic publish frequency (TUI) |
| `adora topic echo <TOPIC>` | Print topic messages to stdout |
| `adora topic info <TOPIC>` | Show topic type and metadata |
| `adora node list` | List nodes in a dataflow |
| `adora node info <NODE>` | Show detailed node status, inputs, outputs, and metrics |
| `adora node restart <NODE>` | Restart a single node within a running dataflow |
| `adora node stop <NODE>` | Stop a single node within a running dataflow |
| `adora topic pub <TOPIC> <DATA>` | Publish JSON data to a topic |
| `adora param list <NODE>` | List runtime parameters for a node |
| `adora param get <NODE> <KEY>` | Get a runtime parameter value |
| `adora param set <NODE> <KEY> <VALUE>` | Set a runtime parameter (JSON value) |
| `adora param delete <NODE> <KEY>` | Delete a runtime parameter |
| `adora trace list` | List recent traces captured by the coordinator |
| `adora trace view <ID>` | View spans for a specific trace (supports prefix matching) |
| `adora record <PATH>` | Record dataflow messages to `.adorec` file |
| `adora replay <FILE>` | Replay recorded messages from `.adorec` file |

### Cluster management

| Command | Description |
|---------|-------------|
| `adora cluster up <PATH>` | Bring up a cluster from a cluster.yml file |
| `adora cluster status` | Show connected daemons and active dataflows |
| `adora cluster down` | Tear down the cluster |
| `adora cluster install <PATH>` | Install daemons as systemd services |
| `adora cluster uninstall <PATH>` | Remove systemd services |
| `adora cluster upgrade <PATH>` | Rolling upgrade: SCP binary + restart per-machine |
| `adora cluster restart <NAME>` | Restart a dataflow by name or UUID |

### Setup and utilities

| Command | Description |
|---------|-------------|
| `adora doctor` | Diagnose environment, connectivity, and dataflow health |
| `adora status` | Check system health (alias: `check`) |
| `adora new` | Generate a new project or node |
| `adora graph <PATH>` | Visualize a dataflow (Mermaid or HTML) |
| `adora expand <PATH>` | Expand module references and print flat YAML |
| `adora validate <PATH>` | Validate dataflow YAML and check [type annotations](docs/types.md) |
| `adora system` | System management (daemon/coordinator control) |
| `adora completion <SHELL>` | Generate shell completions |
| `adora self update` | Update adora CLI |

For full CLI documentation, see [docs/cli.md](docs/cli.md). For distributed deployment, see [docs/distributed-deployment.md](docs/distributed-deployment.md).

## Dataflow Configuration

Dataflows are defined in YAML. Each node declares its binary/script, inputs, and outputs:

```yaml
nodes:
  - id: camera
    build: pip install opencv-video-capture
    path: opencv-video-capture
    inputs:
      tick: adora/timer/millis/20
    outputs:
      - image
    env:
      CAPTURE_PATH: 0
      IMAGE_WIDTH: 640
      IMAGE_HEIGHT: 480

  - id: object-detection
    build: pip install adora-yolo
    path: adora-yolo
    inputs:
      image: camera/image
    outputs:
      - bbox

  - id: plot
    build: pip install adora-rerun
    path: adora-rerun
    inputs:
      image: camera/image
      boxes2d: object-detection/bbox
```

**Built-in timer nodes:** `adora/timer/millis/<N>` and `adora/timer/hz/<N>`.

**Input format:** `<node-id>/<output-name>` to subscribe to another node's output.

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
adora validate dataflow.yml                        # static check (warnings)
adora validate --strict-types dataflow.yml         # fail on warnings (CI)
adora build dataflow.yml --strict-types            # type check during build
ADORA_RUNTIME_TYPE_CHECK=warn adora run dataflow.yml  # runtime check
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
| Coordinator <-> Daemon | TCP | Node spawning, dataflow lifecycle |
| Daemon <-> Daemon | Zenoh | Distributed cross-machine communication |
| Daemon <-> Node | Shared memory / TCP | Zero-copy IPC for data >4KB, TCP for small messages |

### Key components

- **Coordinator** -- orchestrates dataflow lifecycle across daemons. Supports in-memory or persistent (redb) state store.
- **Daemon** -- spawns and manages nodes on a single machine. Handles shared memory allocation and message routing.
- **Runtime** -- in-process operator execution engine. Operators run inside the runtime process, avoiding per-operator process overhead.
- **Nodes** -- standalone processes that communicate via inputs/outputs. Written in Rust, Python, C, or C++.
- **Operators** -- lightweight functions that run inside the runtime. Faster than nodes for simple transformations.

### Workspace layout

```
binaries/
  cli/                  # adora CLI binary
  coordinator/          # Orchestration service
  daemon/               # Node manager + IPC
  runtime/              # In-process operator runtime
  ros2-bridge-node/     # ROS2 bridge binary
  record-node/          # Dataflow message recorder
  replay-node/          # Recorded message replayer
libraries/
  core/                 # Descriptor parsing, build utilities
  message/              # Inter-component message types (v0.7.0)
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
  rust/node/            # Rust node API (adora-node-api)
  rust/operator/        # Rust operator API (adora-operator-api)
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
| Rust | `adora-node-api` | `adora-operator-api` | [API Reference](docs/api-rust.md) | First-class |
| Python >= 3.8 | `pip install adora-rs` | included | [Getting Started](docs/python-guide.md), [API Reference](docs/api-python.md) | First-class |
| C | `adora-node-api-c` | `adora-operator-api-c` | [API Reference](docs/api-c.md) | Supported |
| C++ | `adora-node-api-cxx` | `adora-operator-api-cxx` | [API Reference](docs/api-cxx.md) | Supported |
| ROS2 >= Foxy | `adora-ros2-bridge` | -- | [Bridge Guide](docs/ros2-bridge.md) | Experimental |

### Platform support

| Platform | Status |
|----------|--------|
| Linux (x86_64, ARM64, ARM32) | First-class |
| macOS (ARM64) | First-class |
| Windows (x86_64) | Best effort |
| WSL (x86_64) | Best effort |

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
| [typed-dataflow](examples/typed-dataflow) | Python | Type annotations with `adora validate` |

### Communication patterns

| Example | Language | Description |
|---------|----------|-------------|
| [service-example](examples/service-example) | Rust | Request/reply with `request_id` correlation |
| [action-example](examples/action-example) | Rust | Goal/feedback/result with cancellation |

See [docs/patterns.md](docs/patterns.md) for the full guide.

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

### Performance

| Example | Language | Description |
|---------|----------|-------------|
| [benchmark](examples/benchmark) | Rust/Python | Latency and throughput benchmark |
| [ros2-comparison](examples/ros2-comparison) | Python | Adora vs ROS2 comparison |
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

**Rust edition 2024, MSRV 1.85.0, workspace version 0.4.1.**

### Build

```bash
# Build all (excluding Python packages which require maturin)
cargo build --all \
  --exclude adora-node-api-python \
  --exclude adora-operator-api-python \
  --exclude adora-ros2-bridge-python

# Build specific package
cargo build -p adora-cli
```

### Test

```bash
# Run all tests
cargo test --all \
  --exclude adora-node-api-python \
  --exclude adora-operator-api-python \
  --exclude adora-ros2-bridge-python

# Test single package
cargo test -p adora-core

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

## Contributing

We welcome contributors of all experience levels. See the [contributing guide](CONTRIBUTING.md) to get started.

### Communication

- [Discord](https://discord.gg/6eMGGutkfE)
- [GitHub Discussions](https://github.com/orgs/adora-rs/discussions)

## AI-Assisted Development

This repository is maintained with AI-assisted agentic engineering. Code generation, reviews, refactoring, testing, and commits are driven by autonomous AI agents -- enabling faster iteration and higher code quality at scale.

## License

Apache-2.0. See [NOTICE.md](NOTICE.md) for details.
