# Changelog

## Unreleased

### Added

- **MAVLink 2 bridge** (`dora-mavlink2-bridge` extension + `dora-mavlink2-bridge-node` binary): Apache Arrow ↔ MAVLink 2 conversion for the common dialect (HEARTBEAT, SYS_STATUS, SYSTEM_TIME, ATTITUDE, ATTITUDE_QUATERNION, LOCAL_POSITION_NED, GLOBAL_POSITION_INT, GPS_RAW_INT, RC_CHANNELS, SERVO_OUTPUT_RAW, COMMAND_LONG, COMMAND_ACK, MISSION_CURRENT). TCP/UDP/serial transports, daemon-spawnable bridge node, and a self-contained `examples/mavlink2-bridge` dataflow with an in-process UDP simulator (no SITL/MAVProxy required). The example ships three consumer variants — Rust (`dataflow-rust.yml`), Python (`dataflow-python.yml`, `--uv`), and C++ (`dataflow-cxx.yml` via `cargo run --example mavlink2-bridge-cxx`) — all reading the same `bridge/heartbeat` Arrow output. See [#1786](https://github.com/dora-rs/dora/issues/1786).
- **`examples/mavlink2-bridge-sitl-mission`**: closed-loop ArduCopter SITL demo. A Python dora node arms, takes off, hovers, lands, and disarms a simulated multirotor by driving the bridge's `command_long_cmd` input and watching `command_ack` + `global_position_int`. Local-only on Ubuntu / macOS; not part of CI (SITL needs a one-time ArduPilot install per developer machine). See `examples/mavlink2-bridge-sitl-mission/README.md`.

## 0.1.0 (2026-03-13)

First official release of Dora (AI-Dora) -- a 100% Rust framework for building real-time robotics and AI applications.

### Highlights

- **10-17x faster than ROS2 Python** with zero-copy shared memory IPC and Apache Arrow native data format
- **Multi-language support** -- write nodes in Rust, Python, C, or C++ with native APIs
- **Declarative YAML dataflows** -- define pipelines as directed graphs with typed inputs/outputs
- **Single CLI for full lifecycle** -- build, run, monitor, record/replay, and distributed deployment

### Core Framework

- Coordinator/daemon architecture for distributed multi-machine deployments
- WebSocket control plane (single port for CLI and topic data)
- Shared memory transport for messages >= 4KB (zero-copy)
- Zenoh pub-sub for cross-machine communication
- Apache Arrow columnar format throughout (zero serialization overhead)

### Fault Tolerance

- Per-node restart policies (never/on-failure/always) with exponential backoff
- Passive health monitoring for hung node detection
- Per-input circuit breakers with configurable timeouts and auto re-subscription
- Coordinator state persistence via redb

### Communication Patterns

- Topic (default pub/sub)
- Service (request/reply with correlation IDs)
- Action (goal/feedback/result with cancellation)
- Streaming (session/segment/chunk for voice and real-time apps)

### Logging and Observability

- Unified logging with `dora/logs` virtual input and OTel propagation
- Structured output with filtering, rotation, and log routing
- `dora top` TUI for per-node CPU, memory, queue depth, network I/O
- `topic echo/hz/info` for live data inspection
- `trace list/view` for coordinator span inspection
- Record/replay to `.drec` files with node substitution

### CLI

- `dora run` for local dev, `dora up/start` for distributed prod
- `dora record/replay` for dataflow debugging
- `dora cluster up/status/down` for SSH-based fleet management
- `dora node info/restart/stop`, `dora topic pub`
- `dora doctor` for environment diagnostics
- `dora status --json` for programmatic access
- Configurable queue policies (`drop_oldest` / `backpressure`) per input

### Module System

- Reusable sub-graphs as standalone YAML files
- Typed inputs/outputs with parameters and nested composition
- Optional type annotations with build-time static validation

### ROS2 Bridge

- Bidirectional topic, service, and action bridging
- Declarative YAML configuration
- Per-topic QoS override

### Developer Experience

- Hot reload for Python operators
- In-process operators for lightweight functions
- Comprehensive examples (15+ dataflows)
- mdBook user guide (English and Chinese)
- GitHub Actions CI with cross-platform testing

### Security

- Bearer token authentication (opt-in)
- Per-IP WebSocket rate limiting
- Path traversal guards, input validation, error sanitization
- Resource limits and bounded data structures
