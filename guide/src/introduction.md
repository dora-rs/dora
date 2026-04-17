# Dora

**Agentic Dataflow-Oriented Robotic Architecture** -- a 100% Rust framework for building real-time robotics and AI applications.

## Why Dora?

### Performance

- **10-17x faster than ROS2 Python** -- 100% Rust internals with zero-copy shared memory IPC for messages >4KB, flat latency from 4KB to 4MB payloads
- **Apache Arrow native** -- columnar memory format end-to-end with zero serialization overhead; shared across all language bindings
- **Zenoh SHM data plane** -- nodes publish directly via Zenoh shared memory for zero-copy data transfer; automatic network fallback for cross-machine
- **Non-blocking event loop** -- Zenoh publishes offloaded to drain task; metrics collection runs in background

### Developer Experience

- **Single CLI, full lifecycle** -- `dora run` for local dev, `dora up/start` for distributed prod, plus build, logs, monitoring, record/replay all from one tool
- **Declarative YAML dataflows** -- define pipelines as directed graphs, connect nodes through typed inputs/outputs, optional [type annotations](concepts/types.md) with static validation
- **Multi-language nodes** -- write nodes in Rust, Python, C, or C++ with native APIs (not wrappers); mix languages freely in one dataflow
- **[Reusable modules](concepts/modules.md)** -- compose sub-graphs as standalone YAML files with typed inputs/outputs, parameters, and nested composition
- **Hot reload** -- live-reload Python operators without restarting the dataflow

### Production Readiness

- **Fault tolerance** -- per-node restart policies (never/on-failure/always), exponential backoff, health monitoring, circuit breakers with configurable input timeouts
- **Distributed by default** -- local shared memory between co-located nodes, automatic [Zenoh](https://zenoh.io/) pub-sub for cross-machine communication, SSH-based cluster management with label scheduling
- **Coordinator HA** -- persistent redb state store, daemon auto-reconnect, dataflow records survive coordinator restart (full running-dataflow reclaim across restart is partial; see open tracker)
- **Dynamic topology** -- add and remove nodes from running dataflows via CLI without restarting
- **Configurable queue policies** -- `drop_oldest` (default) or `backpressure` per input, with metrics on dropped messages
- **Soft real-time** -- optional `--rt` flag for mlockall + SCHED_FIFO; per-node `cpu_affinity` pinning
- **OpenTelemetry** -- built-in structured logging with rotation/routing, metrics, distributed tracing

### Debugging and Observability

- **Record/replay** -- capture dataflow messages to `.drec` files, replay offline at any speed with node substitution
- **Topic inspection** -- `topic echo` to print live data, `topic hz` TUI for frequency analysis, `topic info` for schema and bandwidth
- **Resource monitoring** -- `dora top` TUI showing per-node CPU, memory, queue depth, network I/O across all machines
- **Log aggregation** -- subscribe to `dora/logs` to receive structured log messages from all nodes without extra wiring
- **Trace inspection** -- `trace list` and `trace view` for viewing coordinator spans without external infrastructure

### Ecosystem

- **Communication patterns** -- built-in [service (request/reply)](concepts/patterns.md), [action (goal/feedback/result)](concepts/patterns.md), and [streaming (session/segment/chunk)](concepts/patterns.md) patterns via well-known metadata keys
- **ROS2 bridge** -- bidirectional interop with ROS2 topics, services, and actions
- **In-process operators** -- lightweight functions that run inside a shared runtime, avoiding per-node process overhead

## Next Steps

- [Install Dora](getting-started/installation.md)
- [Quick Start tutorial](getting-started/quickstart.md)
- [Architecture overview](concepts/architecture.md)
- [Dataflow YAML reference](concepts/dataflow-yaml.md)
- [Type annotations](concepts/types.md)
- [Communication patterns](concepts/patterns.md)
