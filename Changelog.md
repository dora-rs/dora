# Changelog

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
- Record/replay to `.adorec` files with node substitution

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
