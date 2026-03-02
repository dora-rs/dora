# Adora

**Agentic Dataflow-Oriented Robotic Architecture** -- a 100% Rust framework for building real-time robotics and AI applications.

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
- **Declarative YAML dataflows** -- define pipelines as directed graphs, connect nodes through typed inputs/outputs, override with environment variables
- **Multi-language nodes** -- write nodes in Rust, Python, C, or C++ with native APIs (not wrappers); mix languages freely in one dataflow
- **Hot reload** -- live-reload Python operators without restarting the dataflow
- **Programmatic builder** -- construct dataflows in Python code as an alternative to YAML

### Production readiness

- **Fault tolerance** -- per-node restart policies (never/on-failure/always), exponential backoff, health monitoring, circuit breakers with configurable input timeouts
- **Distributed by default** -- local shared memory between co-located nodes, automatic [Zenoh](https://zenoh.io/) pub-sub for cross-machine communication, machine-targeted deployment
- **Coordinator persistence** -- optional redb-backed state store survives coordinator crashes and restarts
- **OpenTelemetry** -- built-in structured logging with rotation/routing, metrics, distributed tracing, and zero-setup trace viewing via CLI

### Debugging and observability

- **Record/replay** -- capture dataflow messages to `.adorec` files, replay offline at any speed with node substitution for regression testing
- **Topic inspection** -- `topic echo` to print live data, `topic hz` TUI for frequency analysis, `topic info` for schema and bandwidth
- **Resource monitoring** -- `inspect top` TUI showing per-node CPU, memory, and I/O across all machines
- **Trace inspection** -- `trace list` and `trace view` for viewing coordinator spans without external infrastructure
- **Dataflow visualization** -- generate interactive HTML or Mermaid graphs from YAML descriptors

### Ecosystem

- **ROS2 bridge** -- bidirectional interop with ROS2 topics, services, and actions; QoS mapping; Arrow-native type conversion
- **Pre-packaged nodes** -- [node hub](https://github.com/dora-rs/dora-hub/) with ready-made nodes for cameras, YOLO, LLMs, TTS, and more
- **In-process operators** -- lightweight functions that run inside a shared runtime, avoiding per-node process overhead for simple transformations

## Installation

### From PyPI (recommended)

```bash
pip install adora-rs-cli
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

```bash
cargo install adora-cli --features redb-backend
```

## Quick Start

### 1. Run a Python dataflow

```bash
uv venv --seed -p 3.11
adora build https://raw.githubusercontent.com/adora-rs/adora/refs/heads/main/examples/python-dataflow/dataflow.yml --uv
adora run dataflow.yml --uv
```

Press `Ctrl+C` to stop.

### 2. Run a Rust dataflow

```bash
cd examples/rust-dataflow
adora run dataflow.yml
```

### 3. Distributed mode

```bash
# Terminal 1: start coordinator + daemon
adora up

# Terminal 2: start a dataflow
adora start dataflow.yml --attach

# Terminal 3: monitor
adora list
adora logs <dataflow-id>
adora inspect top

# Stop
adora stop <dataflow-id>
adora down
```

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

### Monitoring

| Command | Description |
|---------|-------------|
| `adora list` | List running dataflows (alias: `ps`) |
| `adora logs <ID>` | Show logs for a dataflow or node |
| `adora inspect top` | Real-time resource monitor (TUI) |
| `adora topic list` | List topics in a dataflow |
| `adora topic hz <TOPIC>` | Measure topic publish frequency (TUI) |
| `adora topic echo <TOPIC>` | Print topic messages to stdout |
| `adora topic info <TOPIC>` | Show topic type and metadata |
| `adora node list` | List nodes in a dataflow |
| `adora trace list` | List recent traces captured by the coordinator |
| `adora trace view <ID>` | View spans for a specific trace (supports prefix matching) |
| `adora record <PATH>` | Record dataflow messages to `.adorec` file |
| `adora replay <FILE>` | Replay recorded messages from `.adorec` file |

### Setup and utilities

| Command | Description |
|---------|-------------|
| `adora status` | Check system health (alias: `check`) |
| `adora new` | Generate a new project or node |
| `adora graph <PATH>` | Visualize a dataflow (Mermaid or HTML) |
| `adora system` | System management (daemon/coordinator control) |
| `adora completion <SHELL>` | Generate shell completions |
| `adora self update` | Update adora CLI |

For full CLI documentation, see [docs/cli.md](docs/cli.md).

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
| Python >= 3.8 | `adora-node-api-python` | `adora-operator-api-python` | [API Reference](docs/api-python.md) | First-class |
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
| [benchmark](examples/benchmark) | Rust | CPU latency benchmark |
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
