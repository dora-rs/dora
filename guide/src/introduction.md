# Adora

**Agentic Dataflow-Oriented Robotic Architecture** -- a 100% Rust framework for building real-time robotics and AI applications.

## Why Adora?

- **10-17x faster than ROS2 Python** -- zero-copy shared memory IPC, flat latency from 4KB to 4MB payloads
- **Apache Arrow native** -- columnar memory format end-to-end with zero serialization overhead
- **Single CLI, full lifecycle** -- `adora run` for local dev, `adora up/start` for distributed prod, plus build, logs, monitoring, record/replay
- **Declarative YAML dataflows** -- define pipelines as directed graphs, connect nodes through typed inputs/outputs
- **Multi-language nodes** -- write nodes in Rust, Python, C, or C++ with native APIs; mix languages freely
- **Fault tolerance** -- per-node restart policies, exponential backoff, health monitoring, circuit breakers
- **Distributed by default** -- local shared memory between co-located nodes, automatic Zenoh pub-sub across machines
- **Record/replay** -- capture dataflow messages to `.adorec` files, replay offline at any speed
- **Built-in observability** -- `adora top` TUI, topic inspection, trace viewing, structured logging with rotation

## Next Steps

- [Install Adora](getting-started/installation.md)
- [Quick Start tutorial](getting-started/quickstart.md)
- [Architecture overview](concepts/architecture.md)
