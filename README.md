# Adora (AI-Dora)

**Agentic Dataflow-Oriented Robotic Architecture** -- a 100% Rust framework for building real-time robotics and AI applications.

<p align="center">
  <img src="https://raw.githubusercontent.com/adora-rs/adora/main/docs/src/logo.svg" width="350"/>
</p>

<div align="center">

[Website](https://www.adora-rs.ai) |
[Python API](https://adora-rs.ai/docs/guides/getting-started/conversation_py/) |
[Rust API](https://docs.rs/adora-node-api/latest/adora_node_api/) |
[Guide](https://www.adora-rs.ai/docs/guides/) |
[Discord](https://discord.gg/6eMGGutkfE)

[![Build and test](https://github.com/adora-rs/adora/workflows/CI/badge.svg)](https://github.com/adora-rs/adora/actions)
[![crates.io](https://img.shields.io/crates/v/adora_node_api.svg)](https://crates.io/crates/adora-rs)
[![docs.rs](https://docs.rs/adora-node-api/badge.svg)](https://docs.rs/adora-node-api/latest/adora_node_api/)
[![PyPI](https://img.shields.io/pypi/v/adora-rs.svg)](https://pypi.org/project/adora-rs/)
[![License](https://img.shields.io/github/license/adora-rs/adora)](https://github.com/adora-rs/adora/blob/main/LICENSE)

</div>

> **Built with AI-assisted agentic engineering** -- code generation, reviews, refactoring, testing, and commits are driven by autonomous AI agents.

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

- **10-17x faster than ROS2** -- 100% Rust internals with zero-copy shared memory IPC
- **Apache Arrow data format** -- columnar memory format with zero serialization overhead
- **Multi-language** -- write nodes in Rust, Python, C, or C++
- **Declarative dataflows** -- define pipelines in YAML, connect nodes through inputs/outputs
- **Distributed** -- local shared memory + [Zenoh](https://zenoh.io/) for cross-machine communication
- **Hot reload** -- live-reload Python operators without restarting the dataflow
- **Pre-packaged nodes** -- [node hub](https://github.com/dora-rs/dora-hub/) for cameras, YOLO, LLMs, TTS, and more
- **OpenTelemetry** -- built-in logging, metrics, and tracing

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
| `adora node list` | List nodes in a dataflow |

### Setup and utilities

| Command | Description |
|---------|-------------|
| `adora status` | Check system health (alias: `check`) |
| `adora new` | Generate a new project or node |
| `adora graph <PATH>` | Visualize a dataflow (Mermaid or HTML) |
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
  cli/              # adora CLI binary
  coordinator/      # Orchestration service
  daemon/           # Node manager + IPC
  runtime/          # In-process operator runtime
libraries/
  core/             # Descriptor parsing, build utilities
  message/          # Inter-component message types
  shared-memory-server/  # Zero-copy IPC
  arrow-convert/    # Arrow data conversion
  extensions/
    telemetry/      # OpenTelemetry tracing + metrics
    ros2-bridge/    # ROS2 interop
apis/
  rust/             # Rust node and operator APIs
  python/           # Python APIs (PyO3)
  c/                # C node and operator APIs
  c++/              # C++ node and operator APIs
examples/           # Example dataflows
```

## Language Support

| Language | Node API | Operator API | Status |
|----------|----------|--------------|--------|
| Rust | `adora-node-api` | `adora-operator-api` | First-class |
| Python >= 3.8 | `adora-node-api-python` | `adora-operator-api-python` | First-class |
| C | `adora-node-api-c` | `adora-operator-api-c` | Supported |
| C++ | via C API | via C API | Supported |
| ROS2 >= Foxy | `adora-ros2-bridge` | -- | Experimental |

### Platform support

| Platform | Status |
|----------|--------|
| Linux (x86_64, ARM64, ARM32) | First-class |
| macOS (ARM64) | First-class |
| Windows (x86_64) | Best effort |
| WSL (x86_64) | Best effort |

## Examples

| Example | Language | Description |
|---------|----------|-------------|
| [rust-dataflow](examples/rust-dataflow) | Rust | Basic Rust node pipeline |
| [python-dataflow](examples/python-dataflow) | Python | Python sender/transformer/receiver |
| [python-operator-dataflow](examples/python-operator-dataflow) | Python | Python operators (in-process) |
| [python-dataflow-builder](examples/python-dataflow-builder) | Python | Pythonic imperative API |
| [c-dataflow](examples/c-dataflow) | C | C node example |
| [c++-dataflow](examples/c++-dataflow) | C++ | C++ node example |
| [cmake-dataflow](examples/cmake-dataflow) | C/C++ | CMake-based build |
| [benchmark](examples/benchmark) | Rust | CPU latency benchmark |
| [cuda-benchmark](examples/cuda-benchmark) | Rust/CUDA | GPU zero-copy benchmark |
| [multiple-daemons](examples/multiple-daemons) | Rust | Distributed multi-daemon setup |
| [ros2-bridge](examples/ros2-bridge) | Rust/Python | ROS2 interoperability |

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
