# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

### Build Commands
- `cargo build --all` - Build all crates in the workspace
- `cargo build --workspace` - Alternative workspace build command
- `cargo build --examples` - Build all examples
- `cargo build --all --exclude dora-node-api-python --exclude dora-operator-api-python --exclude dora-ros2-bridge-python` - Build excluding Python bindings

### Test Commands
- `cargo test --all` - Run all tests
- `cargo test --all --exclude dora-node-api-python --exclude dora-operator-api-python --exclude dora-ros2-bridge-python` - Run tests excluding Python bindings
- `cargo test -p <package-name>` - Run tests for specific package

### Lint and Format Commands
- `cargo fmt --all` - Format all code using rustfmt
- `cargo fmt --all -- --check` - Check formatting without making changes
- `cargo clippy --all` - Run Clippy linter on all crates
- `cargo clippy --all --features tracing` - Run Clippy with tracing feature
- `cargo clippy --all --features metrics` - Run Clippy with metrics feature

### CLI Installation and Testing
- `cargo install --path binaries/cli --locked` - Install dora CLI locally
- `dora new <project-name>` - Create new project template
- `dora up` - Start dora daemon
- `dora build <dataflow.yml>` - Build dataflow dependencies
- `dora run <dataflow.yml>` - Run dataflow
- `dora start <dataflow.yml> --name <instance-name> --detach` - Start dataflow in background
- `dora stop --name <instance-name>` - Stop running dataflow
- `dora list` - List running dataflows
- `dora destroy` - Clean up daemon

### Python Development
- `uv venv --seed -p 3.12` - Create Python virtual environment
- `uv pip install -e apis/python/node` - Install Python node API in development mode
- `uv run ruff check .` - Run Python linter
- `uv run pytest` - Run Python tests
- `dora build <dataflow.yml> --uv` - Build with uv package manager
- `dora run <dataflow.yml> --uv` - Run with uv

### Example Commands
- `cargo run --example rust-dataflow` - Run Rust dataflow example
- `cargo run --example python-dataflow` - Run Python dataflow example
- `cargo run --example c-dataflow` - Run C dataflow example
- `cargo run --example cxx-dataflow` - Run C++ dataflow example
- `cargo run --example benchmark --release` - Run benchmark in release mode

## High-Level Architecture

### Core Components

**Dora** is a dataflow-oriented robotic architecture that implements a declarative dataflow paradigm where tasks are split between nodes isolated as individual processes.

### Key Architecture Elements

1. **Workspace Structure**: Cargo workspace with multiple crates organized by function:
   - `apis/` - Language bindings (Rust, Python, C, C++)
   - `binaries/` - Executables (CLI, coordinator, daemon, runtime)
   - `libraries/` - Core libraries and extensions
   - `examples/` - Sample implementations

2. **Core Runtime Components**:
   - **Daemon** (`binaries/daemon/`) - Manages node lifecycle and communication
   - **Coordinator** (`binaries/coordinator/`) - Orchestrates distributed dataflows
   - **CLI** (`binaries/cli/`) - Command-line interface for users
   - **Runtime** (`binaries/runtime/`) - Node execution environment

3. **Communication Layer**:
   - **Message Format**: Apache Arrow for zero-copy data transfer
   - **Local Communication**: Shared memory with message tracking
   - **Remote Communication**: TCP for distributed setups
   - **Extensions**: Optional Zenoh support for distributed systems

4. **Node APIs**:
   - **Rust API** (`apis/rust/node/`) - Primary API with `DoraNode` and `EventStream`
   - **Python API** (`apis/python/node/`) - Python bindings via PyO3
   - **C/C++ APIs** (`apis/c*/`) - Foreign function interfaces

5. **Key Libraries**:
   - **dora-core** - Core functionality and utilities
   - **dora-message** - Message serialization/deserialization
   - **arrow-convert** - Arrow data format conversions
   - **shared-memory-server** - Shared memory management

### Dataflow Model

Nodes communicate through:
- **Inputs/Outputs**: Defined in YAML dataflow configuration
- **Events**: Input data, control messages, stop signals
- **Message Passing**: Arrow-formatted data via shared memory or TCP

### Extension System

- **ROS2 Bridge** - Compilation-free message passing to ROS2
- **Telemetry** - OpenTelemetry for metrics, tracing, and logging
- **Download** - Git-based node distribution

### Development Patterns

1. **Node Development**: Use `DoraNode::init_from_env()` for standard nodes
2. **Event Handling**: Iterate over `EventStream` to process `Event::Input`
3. **Output Sending**: Use Arrow format for best performance
4. **Dynamic Nodes**: Set `path: dynamic` in YAML for manual node control

### Testing Strategy

The project uses comprehensive CI including:
- Cross-platform testing (Linux, macOS, Windows)
- Example execution verification
- ROS2 bridge integration tests
- License compatibility checks
- Code formatting and linting

### Language Support

- **Rust**: First-class support with full API
- **Python**: Mature bindings with async support
- **C/C++**: Basic API support
- **Cross-compilation**: Support for multiple architectures