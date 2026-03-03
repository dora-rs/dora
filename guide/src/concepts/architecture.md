# Architecture

## Overview

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

## Key Components

- **Coordinator** -- orchestrates dataflow lifecycle across daemons. Supports in-memory or persistent (redb) state store.
- **Daemon** -- spawns and manages nodes on a single machine. Handles shared memory allocation and message routing.
- **Runtime** -- in-process operator execution engine. Operators run inside the runtime process, avoiding per-operator process overhead.
- **Nodes** -- standalone processes that communicate via inputs/outputs. Written in Rust, Python, C, or C++.
- **Operators** -- lightweight functions that run inside the runtime. Faster than nodes for simple transformations.

## Data Format

All data flows through the system as **Apache Arrow** columnar arrays. This enables zero-copy shared memory transfer between co-located nodes and zero-serialization overhead.

## Workspace Layout

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
