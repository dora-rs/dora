---
description: "Adora project architecture, conventions, and patterns. Auto-activate when working in this repository. Keywords: adora, dataflow, node, operator, daemon, coordinator, shared memory, zero-copy, Arrow, YAML, DaemonRequest, DaemonReply, AdoraNode, AdoraOperator, DataflowId, NodeId, RuntimeConfig"
triggers:
  - dataflow
  - node
  - operator
  - daemon
  - coordinator
  - shared memory
  - DaemonRequest
  - AdoraNode
  - RuntimeConfig
  - DataflowId
---

# Adora Project Patterns

## Architecture Overview

```
CLI (WS:6013) --> Coordinator --> Daemon(s) --> Nodes / Operators
                  (orchestrate)   (per machine)  (user code)
```

- **Coordinator**: Single orchestrator, persistent state (in-memory or redb), label-based node scheduling
- **Daemon**: Per-machine process manager, owns shared memory regions, TCP/Unix sockets to nodes
- **Node**: Separate OS process, communicates via shared memory (>4KB) or TCP (small msgs)
- **Operator**: In-process (runtime), no IPC overhead

## Message Protocol

All inter-component messages are in `libraries/message/src/`. Key types:

| Message | Direction | Purpose |
|---------|-----------|---------|
| `DaemonRequest` / `DaemonReply` | Node <-> Daemon | Data send/recv, subscribe, register |
| `DaemonCoordinatorEvent` | Daemon -> Coordinator | Status, logs, node results |
| `CoordinatorRequest` | Coordinator -> Daemon | Spawn/stop nodes, reload |
| `InterDaemonEvent` | Daemon <-> Daemon | Cross-machine message routing |

Messages are versioned. Version check happens at connection init.

## Node Lifecycle

1. Coordinator sends `SpawnNode` to target Daemon (selected by labels)
2. Daemon creates `RunningNode` with `RuntimeConfig` passed as env var (JSON)
3. Node process starts, connects back to Daemon via TCP/Unix socket
4. Node registers inputs/outputs, subscribes to data channels
5. Data flows via bounded `flume` channels internally, shared memory for large payloads
6. On stop: soft kill (SIGTERM) -> grace period -> hard kill (SIGKILL)

## Key Abstractions

```rust
// Node API (apis/rust/node/)
pub struct AdoraNode { /* ... */ }
impl AdoraNode {
    pub fn init() -> Result<(Self, Events)>;       // Standard init
    pub fn init_from_env() -> Result<(Self, Events)>; // From RuntimeConfig env var
    pub fn send_output(id, data, metadata) -> Result<()>;
    pub fn send_service_request(...) -> Result<()>;
    pub fn send_service_response(...) -> Result<()>;
}

// Operator API (apis/rust/operator/)
pub trait DoraOperator {
    fn on_event(&mut self, event: &Event, send: &mut SendOutput) -> DoraStatus;
}
```

## Data Format

- **Apache Arrow** columnar format throughout (zero serialization)
- Messages >4KB use shared memory (4 named regions per node)
- Messages <4KB use TCP directly
- Python bindings use `arrow::pyarrow` for zero-copy across FFI

## Dataflow YAML Schema

```yaml
nodes:
  my-node:
    path: path/to/executable        # or "shell:" or Python script
    inputs:
      input_name: other-node/output  # Subscribe to another node's output
    outputs:
      - output_name
    env:
      KEY: value
    args: [--flag, value]
    restart_policy: on_failure       # never | on_failure | always
    queue_size: 10                   # Bounded channel size
    queue_policy: latest             # latest (drop old) | blocking
    labels:
      machine: gpu-server            # For distributed placement
    health_check:
      interval_secs: 5
      timeout_secs: 2
```

**Virtual inputs** (daemon-generated):
- `adora/timer/secs/<N>` or `adora/timer/millis/<N>` or `adora/timer/hz/<N>`
- `adora/logs`, `adora/logs/<level>`, `adora/logs/<level>/<node>`

## Communication Patterns

### Topic (Default)
Standard pub/sub. Node sends output, all subscribers receive.

### Service (Request/Reply)
```rust
// Client
node.send_service_request("service_input", data, request_id)?;
// Server
if let Some(req_id) = metadata.get("request_id") {
    node.send_service_response("response_output", result, req_id)?;
}
```

### Action (Goal/Feedback/Result)
Uses `goal_id` and `goal_status` metadata keys. Supports cancellation.

## State & Parameters

- **Coordinator Store**: Persistent state via `CoordinatorStore` trait (in-memory or redb backend)
- **Parameters**: Key-value per dataflow, propagated via `SetParam`/`GetParam`/`DeleteParam`
- **Distributed state**: UHLC (hybrid logical clock) for distributed causality
- **Node state**: `restart_count`, `last_activity` (atomic), `pid`
- **Fault tolerance**: `FaultToleranceStats` with atomic counters

## ID Conventions

- **DataflowId**: UUID v7 (sortable, timestamp-based)
- **NodeId**: String identifier from YAML
- **Timestamps**: UHLC for cross-machine ordering
- Metadata keys defined in `libraries/message/`: `REQUEST_ID`, `GOAL_ID`, `GOAL_STATUS`, `SESSION_ID`

## Error Handling Patterns

- `eyre::Result` for application errors (with `.context()` chains)
- `NodeErrorCause` enum: `GraceTimeout`, `Cascading`, `SpawnFailure`, `Other`
- Restart policies with exponential backoff (configurable window)
- Cascading error tracking: which node caused which failure

## Testing Conventions

| Change Type | Test Tier | Location |
|------------|-----------|----------|
| Library function | Unit `#[cfg(test)]` | Same file |
| Coordinator/Daemon behavior | Integration | `binaries/coordinator/tests/` |
| CLI command | Smoke (networked) | `tests/example-smoke.rs` |
| Dataflow feature | Smoke (both modes) | `tests/example-smoke.rs` |
| Bug fix | Regression | Whichever tier reproduces it |

**Smoke test helpers**:
- `run_smoke_test(name, yaml, timeout)` -- networked mode (up/start/poll/stop/down)
- `run_smoke_test_local(name, yaml, stop_after)` -- local mode (run --stop-after)

**Integration test setup**: `setup_integration_testing()` with JSON inputs via `ADORA_TEST_WITH_INPUTS` env var.

## Build Commands (Quick Reference)

```bash
# Build (exclude Python)
cargo build --all --exclude adora-node-api-python --exclude adora-operator-api-python --exclude adora-ros2-bridge-python

# Test (exclude Python + examples)
cargo test --all --exclude adora-node-api-python --exclude adora-operator-api-python --exclude adora-ros2-bridge-python --exclude adora-cli-api-python --exclude adora-examples

# Single crate
cargo test -p adora-core

# Pre-commit (mandatory)
cargo fmt --all -- --check
cargo clippy --all --exclude adora-node-api-python --exclude adora-operator-api-python --exclude adora-ros2-bridge-python -- -D warnings
```

## PyO3 Bindings

- Python Node API: `apis/python/node/` -- `#[pyclass] Node` with iterator protocol
- Python Operator API: `apis/python/operator/` -- type conversion utilities
- Python CLI API: `apis/python/cli/` -- `build()`, `run()`, `start_runtime()`
- Uses PyO3 0.28 with `eyre`, `abi3-py37`, `multiple-pymethods` features
- Arrow arrays passed zero-copy via `arrow::pyarrow` FFI
- GIL released during blocking Rust ops (`py.detach()`)

## Key Conventions

- Rust edition 2024, MSRV 1.85.0
- Workspace version 0.2.0 (all crates share it)
- `flume` bounded MPSC for internal event routing
- `tokio` async runtime with full features
- `Zenoh` for distributed cross-machine pub/sub
- Zero-copy threshold: 4KB
- Fire-and-forget for logging/metrics (never block data path)
