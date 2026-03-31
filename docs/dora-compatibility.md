# Dora Compatibility Guide

Adora is a fork of [dora](https://github.com/dora-rs/dora) with enhanced
features. This document describes the compatibility layer that allows
[dora-hub](https://github.com/dora-rs/dora-hub) nodes and operators to run
on adora without modification.

## Quick Reference

| dora import | adora equivalent | compat mechanism |
|-------------|-----------------|------------------|
| `dora-node-api` (Rust crate) | `adora-node-api` | shim crate at `apis/rust/compat/dora-node-api` |
| `dora-operator-api` (Rust crate) | `adora-operator-api` | shim crate at `apis/rust/compat/dora-operator-api` |
| `from dora import Node` (Python) | `from adora import Node` | shim package at `apis/python/node/dora/` |
| `DoraNode` (Rust type) | `AdoraNode` | `pub type DoraNode = AdoraNode` |
| `DoraOperator` (Rust trait) | `AdoraOperator` | re-export in shim crate |
| `DoraStatus` (Python enum) | `AdoraStatus` | `DoraStatus = AdoraStatus` |
| `dora_send_output` (C function) | `adora_send_output` | `#define` in `node_api.h` |

## Adora as a Superset

Adora's API is a **strict superset** of dora's API. Every dora capability
exists in adora, plus additional features:

| Feature | dora | adora |
|---------|------|-------|
| Node init / event stream | Yes | Yes |
| `send_output` / Arrow data | Yes | Yes |
| Event types (Input, Stop, InputClosed) | Yes | Yes + InputRecovered, NodeRestarted, ParamUpdate, ParamDeleted |
| Service pattern (request/reply) | Manual via metadata | `send_service_request()` / `send_service_response()` helpers |
| Action pattern (goal/feedback) | Manual via metadata | Built-in metadata constants + helpers |
| Streaming chunks | No | `send_stream_chunk()` + `StreamSegment` |
| Structured logging | No | `node.log()` / `node.log_info()` etc. |
| Restart awareness | No | `is_restart()` / `restart_count()` |
| Input health tracking | No | `InputTracker` / `InputState` |
| Dataflow builder (Python) | No | `DataflowBuilder` class |
| Testing utilities | No | `MockNode` (Python), `init_testing()` (Rust) |
| Incremental state catch-up | No | Coordinator replays state to reconnecting daemons |

## Rust Compatibility

### Shim Crates

Two shim crates under `apis/rust/compat/` re-export adora APIs with dora names:

```
apis/rust/compat/
├── dora-node-api/       # re-exports adora-node-api::*
└── dora-operator-api/   # re-exports adora-operator-api::*
```

**For dora-hub Rust nodes**, add the shim as a dependency:

```toml
# Cargo.toml
[dependencies]
dora-node-api = { path = "path/to/adora/apis/rust/compat/dora-node-api" }
```

Then existing code works unchanged:

```rust
use dora_node_api::{self, dora_core::config::DataId, DoraNode, Event};

let (mut node, mut events) = DoraNode::init_from_env()?;
```

**For dora-hub Rust operators**:

```rust
use dora_operator_api::{register_operator, DoraOperator, DoraOutputSender, DoraStatus, Event};

#[derive(Default)]
struct MyOperator;

impl DoraOperator for MyOperator {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut DoraOutputSender,
    ) -> Result<DoraStatus, String> {
        // ...
        Ok(DoraStatus::Continue)
    }
}

register_operator!(MyOperator);
```

### Type Aliases in adora-node-api

Even without the shim crates, `adora-node-api` directly exports these aliases:

| Alias | Target |
|-------|--------|
| `DoraNode` | `AdoraNode` |
| `DoraEvent` | `Event` |

And `adora-operator-api` exports:

| Alias | Target |
|-------|--------|
| `DoraStatus` | `AdoraStatus` |
| `DoraOutputSender<'a>` | `AdoraOutputSender<'a>` |

### Module Path Mapping

| dora path | adora path |
|-----------|------------|
| `dora_node_api::dora_core` | `adora_node_api::adora_core` (also available as `dora_core` via shim) |
| `dora_node_api::dora_core::config::DataId` | `adora_node_api::adora_core::config::DataId` |
| `dora_node_api::arrow` | `adora_node_api::arrow` |
| `dora_node_api::flume` | `adora_node_api::flume` |
| `dora_node_api::futures` | `adora_node_api::futures` |

## Python Compatibility

### The `dora` Shim Package

A `dora/` Python package is bundled inside the `adora-rs` wheel. When you
`pip install adora-rs`, both `import adora` and `import dora` work.

```python
# dora-hub pattern — works unchanged
from dora import Node

node = Node()
for event in node:
    if event["type"] == "INPUT":
        node.send_output("output_id", event["value"])
```

```python
# dora-hub operator pattern — works unchanged
from dora import DoraStatus

def on_input(event, send_output):
    send_output("result", event["value"])
    return DoraStatus.CONTINUE
```

### Alias Details

| dora name | adora name | How |
|-----------|------------|-----|
| `dora.Node` | `adora.Node` | Same class (re-export) |
| `dora.DoraStatus` | `adora.AdoraStatus` | `DoraStatus = AdoraStatus` |
| `dora.build()` | `adora.build()` | Same function (re-export) |
| `dora.run()` | `adora.run()` | Same function (re-export) |
| `dora.start_runtime()` | `adora.start_runtime()` | Same function (re-export) |
| `dora.Ros2Context` | `adora.Ros2Context` | Same class (re-export) |

### Adora-Only Python APIs

These are available via `from adora import ...` but not from dora:

- `Node.send_service_request()` / `Node.send_service_response()`
- `Node.log()` / `Node.log_info()` / `Node.log_error()` etc.
- `Node.drain()` / `Node.try_recv()` / `Node.is_empty()`
- `Node.is_restart()` / `Node.restart_count()`
- `adora.testing.MockNode`
- `adora.builder.DataflowBuilder`

## C/C++ Compatibility

### Header Aliases

`node_api.h` provides `#define` macros mapping dora names to adora names:

```c
#include "node_api.h"

// dora-hub C pattern — works unchanged
void *ctx = init_dora_context_from_env();
void *event = dora_next_event(ctx);
enum DoraEventType ty = read_dora_event_type(event);
```

`operator_api.h` provides the same for operator functions:

```c
// dora-hub C operator pattern — works unchanged
DoraInitResult_t dora_init_operator(void);
DoraResult_t dora_drop_operator(void *operator_context);
```

### Full C Function Mapping

| dora name | adora name |
|-----------|------------|
| `init_dora_context_from_env` | `init_adora_context_from_env` |
| `free_dora_context` | `free_adora_context` |
| `dora_next_event` | `adora_next_event` |
| `free_dora_event` | `free_adora_event` |
| `read_dora_event_type` | `read_adora_event_type` |
| `read_dora_input_id` | `read_adora_input_id` |
| `read_dora_input_data` | `read_adora_input_data` |
| `read_dora_input_timestamp` | `read_adora_input_timestamp` |
| `dora_send_output` | `adora_send_output` |
| `dora_log` | `adora_log` |

### C Type Mapping

| dora type | adora type |
|-----------|------------|
| `DoraEventType` | `AdoraEventType` |
| `DoraEventType_Input` | `AdoraEventType_Input` |
| `DoraResult_t` | `AdoraResult_t` |
| `DoraStatus_t` | `AdoraStatus_t` |
| `DORA_STATUS_CONTINUE` | `ADORA_STATUS_CONTINUE` |
| `DORA_STATUS_STOP` | `ADORA_STATUS_STOP` |
| `DORA_STATUS_STOP_ALL` | `ADORA_STATUS_STOP_ALL` |

## Known Limitations and Potential Issues

### 1. Rust Crate Resolution

The shim crates (`dora-node-api`, `dora-operator-api`) are **local workspace
crates** — they are not published to crates.io. Dora-hub nodes that declare
`dora-node-api = "0.3"` in their `Cargo.toml` will pull from crates.io
(the upstream dora version), not the adora shim.

**Workaround**: When using dora-hub nodes with adora, override the dependency
via a workspace `[patch]` section or use path dependencies:

```toml
# In your workspace Cargo.toml
[patch.crates-io]
dora-node-api = { path = "path/to/adora/apis/rust/compat/dora-node-api" }
dora-operator-api = { path = "path/to/adora/apis/rust/compat/dora-operator-api" }
```

### 2. `register_operator!` Macro Generates `adora_*` Symbols

The `register_operator!` macro (re-exported in the shim crate) generates
`adora_init_operator`, `adora_drop_operator`, and `adora_on_event` FFI symbols.
This is correct for the adora runtime. However, if you need `dora_*` FFI
symbols (e.g., loading an operator shared library into the upstream dora
runtime), you would need the upstream macro.

**In practice**: this is not an issue because adora's runtime loads operators
and expects `adora_*` symbols.

### 3. Error Types Differ

| dora | adora |
|------|-------|
| `eyre::Result<T>` | `NodeResult<T>` (wraps `NodeError`) |

The adora node API uses a dedicated `NodeError` enum instead of bare
`eyre::Report`. Code that pattern-matches on `eyre::Report` internals will
need adjustment. The `?` operator works the same way since `NodeError`
implements `std::error::Error`.

### 4. Event Enum is `#[non_exhaustive]`

Adora's `Event` enum has additional variants not present in dora
(`InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted`, `Reload`).
These variants are guarded by `#[non_exhaustive]`, so existing dora-hub code
with a wildcard `_ => {}` arm will compile and handle them correctly.

Code that does *not* have a wildcard arm will get a compile error — this is
intentional and desirable.

### 5. Python Package Namespace

The `dora` shim package is bundled inside the `adora-rs` wheel. If both
`dora-rs` (upstream) and `adora-rs` are installed in the same Python
environment, the `dora` package from whichever was installed last wins.

**Recommendation**: Use a virtual environment with only `adora-rs` installed
when running dora-hub nodes on adora.

### 6. `operator_types.h` is Auto-Generated

The C operator types header (`operator_types.h`) is generated by `safer_ffi`
and must not be edited manually. All dora-compat aliases for C operator types
live in `operator_api.h` instead. If `safer_ffi` regenerates `operator_types.h`
and renames types, the aliases in `operator_api.h` must be updated to match.

### 7. `dora_core` vs `adora_core` in Rust Path

The shim crate provides `dora_node_api::dora_core` as an alias for
`adora_node_api::adora_core`. However, if a dora-hub node uses
`dora_core` as a *standalone* crate dependency (not via `dora_node_api`),
it will need its own shim or `[patch]` entry.

## Testing

Compatibility is verified by tests that mirror actual dora-hub import patterns:

```
cargo test -p dora-node-api -p dora-operator-api
```

Python compatibility tests (requires built wheel):

```
pytest apis/python/node/tests/test_dora_compat.py
```

### What the Tests Cover

- **Rust node shim**: `DoraNode` type equivalence, `dora_core::config::DataId`
  path, `Event` variant matching, metadata constants, `ZERO_COPY_THRESHOLD`
- **Rust operator shim**: `impl DoraOperator` compiles and satisfies
  `AdoraOperator` trait bound, `DoraStatus` variants, `DoraOutputSender`
  type equivalence
- **Python shim**: `from dora import Node`, `DoraStatus` is `AdoraStatus`,
  enum values (0/1/2), ROS2 types importable, `build()`/`run()` importable

## Migration Path (dora -> adora)

For developers migrating from dora to adora:

1. **No code changes required** for existing dora-hub nodes — the compat layer
   handles everything.
2. **To use adora-only features**, change imports from `dora` to `adora`
   (Python) or from `dora-node-api` to `adora-node-api` (Rust) and use the
   new APIs directly.
3. **New nodes** should use adora APIs directly (`AdoraNode`, `AdoraStatus`,
   `from adora import Node`).
4. **Mixed usage** is fine — `DoraNode` and `AdoraNode` are the same type,
   so adora-native and dora-compat code can interoperate freely.
