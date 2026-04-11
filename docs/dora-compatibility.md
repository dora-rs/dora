# Dora Compatibility Guide

Dora is a fork of [dora](https://github.com/dora-rs/dora) with enhanced
features. This document describes the compatibility layer that allows
[dora-hub](https://github.com/dora-rs/dora-hub) nodes and operators to run
on dora without modification.

## Quick Reference

| dora import | dora equivalent | compat mechanism |
|-------------|-----------------|------------------|
| `dora-node-api` (Rust crate) | `dora-node-api` | shim crate at `apis/rust/compat/dora-node-api` |
| `dora-operator-api` (Rust crate) | `dora-operator-api` | shim crate at `apis/rust/compat/dora-operator-api` |
| `from dora import Node` (Python) | `from dora import Node` | shim package at `apis/python/node/dora/` |
| `DoraNode` (Rust type) | `DoraNode` | `pub type DoraNode = DoraNode` |
| `DoraOperator` (Rust trait) | `DoraOperator` | re-export in shim crate |
| `DoraStatus` (Python enum) | `DoraStatus` | `DoraStatus = DoraStatus` |
| `dora_send_output` (C function) | `dora_send_output` | `#define` in `node_api.h` |

## Dora as a Superset

Dora's API is a **strict superset** of dora's API. Every dora capability
exists in dora, plus additional features:

| Feature | dora | dora |
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

Two shim crates under `apis/rust/compat/` re-export dora APIs with dora names:

```
apis/rust/compat/
├── dora-node-api/       # re-exports dora-node-api::*
└── dora-operator-api/   # re-exports dora-operator-api::*
```

**For dora-hub Rust nodes**, add the shim as a dependency:

```toml
# Cargo.toml
[dependencies]
dora-node-api = { path = "path/to/dora/apis/rust/compat/dora-node-api" }
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

### Type Aliases in dora-node-api

Even without the shim crates, `dora-node-api` directly exports these aliases:

| Alias | Target |
|-------|--------|
| `DoraNode` | `DoraNode` |
| `DoraEvent` | `Event` |

And `dora-operator-api` exports:

| Alias | Target |
|-------|--------|
| `DoraStatus` | `DoraStatus` |
| `DoraOutputSender<'a>` | `DoraOutputSender<'a>` |

### Module Path Mapping

| dora path | dora path |
|-----------|------------|
| `dora_node_api::dora_core` | `dora_node_api::dora_core` (also available as `dora_core` via shim) |
| `dora_node_api::dora_core::config::DataId` | `dora_node_api::dora_core::config::DataId` |
| `dora_node_api::arrow` | `dora_node_api::arrow` |
| `dora_node_api::flume` | `dora_node_api::flume` |
| `dora_node_api::futures` | `dora_node_api::futures` |

## Python Compatibility

### The `dora` Shim Package

A `dora/` Python package is bundled inside the `dora-rs` wheel. When you
`pip install dora-rs`, both `import dora` and `import dora` work.

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

| dora name | dora name | How |
|-----------|------------|-----|
| `dora.Node` | `dora.Node` | Same class (re-export) |
| `dora.DoraStatus` | `dora.DoraStatus` | `DoraStatus = DoraStatus` |
| `dora.build()` | `dora.build()` | Same function (re-export) |
| `dora.run()` | `dora.run()` | Same function (re-export) |
| `dora.start_runtime()` | `dora.start_runtime()` | Same function (re-export) |
| `dora.Ros2Context` | `dora.Ros2Context` | Same class (re-export) |

### Dora-Only Python APIs

These are available via `from dora import ...` but not from dora:

- `Node.send_service_request()` / `Node.send_service_response()`
- `Node.log()` / `Node.log_info()` / `Node.log_error()` etc.
- `Node.drain()` / `Node.try_recv()` / `Node.is_empty()`
- `Node.is_restart()` / `Node.restart_count()`
- `dora.testing.MockNode`
- `dora.builder.DataflowBuilder`

## C/C++ Compatibility

### Header Aliases

`node_api.h` provides `#define` macros mapping dora names to dora names:

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

| dora name | dora name |
|-----------|------------|
| `init_dora_context_from_env` | `init_dora_context_from_env` |
| `free_dora_context` | `free_dora_context` |
| `dora_next_event` | `dora_next_event` |
| `free_dora_event` | `free_dora_event` |
| `read_dora_event_type` | `read_dora_event_type` |
| `read_dora_input_id` | `read_dora_input_id` |
| `read_dora_input_data` | `read_dora_input_data` |
| `read_dora_input_timestamp` | `read_dora_input_timestamp` |
| `dora_send_output` | `dora_send_output` |
| `dora_log` | `dora_log` |

### C Type Mapping

| dora type | dora type |
|-----------|------------|
| `DoraEventType` | `DoraEventType` |
| `DoraEventType_Input` | `DoraEventType_Input` |
| `DoraResult_t` | `DoraResult_t` |
| `DoraStatus_t` | `DoraStatus_t` |
| `DORA_STATUS_CONTINUE` | `DORA_STATUS_CONTINUE` |
| `DORA_STATUS_STOP` | `DORA_STATUS_STOP` |
| `DORA_STATUS_STOP_ALL` | `DORA_STATUS_STOP_ALL` |

## Known Limitations and Potential Issues

### 1. Rust Crate Resolution

The shim crates (`dora-node-api`, `dora-operator-api`) are **local workspace
crates** — they are not published to crates.io. Dora-hub nodes that declare
`dora-node-api = "0.3"` in their `Cargo.toml` will pull from crates.io
(the upstream dora version), not the dora shim.

**Workaround**: When using dora-hub nodes with dora, override the dependency
via a workspace `[patch]` section or use path dependencies:

```toml
# In your workspace Cargo.toml
[patch.crates-io]
dora-node-api = { path = "path/to/dora/apis/rust/compat/dora-node-api" }
dora-operator-api = { path = "path/to/dora/apis/rust/compat/dora-operator-api" }
```

### 2. `register_operator!` Macro Generates `dora_*` Symbols

The `register_operator!` macro (re-exported in the shim crate) generates
`dora_init_operator`, `dora_drop_operator`, and `dora_on_event` FFI symbols.
This is correct for the dora runtime. However, if you need `dora_*` FFI
symbols (e.g., loading an operator shared library into the upstream dora
runtime), you would need the upstream macro.

**In practice**: this is not an issue because dora's runtime loads operators
and expects `dora_*` symbols.

### 3. Error Types Differ

| dora | dora |
|------|-------|
| `eyre::Result<T>` | `NodeResult<T>` (wraps `NodeError`) |

The dora node API uses a dedicated `NodeError` enum instead of bare
`eyre::Report`. Code that pattern-matches on `eyre::Report` internals will
need adjustment. The `?` operator works the same way since `NodeError`
implements `std::error::Error`.

### 4. Event Enum is `#[non_exhaustive]`

Dora's `Event` enum has additional variants not present in dora
(`InputRecovered`, `NodeRestarted`, `ParamUpdate`, `ParamDeleted`, `Reload`).
These variants are guarded by `#[non_exhaustive]`, so existing dora-hub code
with a wildcard `_ => {}` arm will compile and handle them correctly.

Code that does *not* have a wildcard arm will get a compile error — this is
intentional and desirable.

### 5. Python Package Namespace

The `dora` shim package is bundled inside the `dora-rs` wheel. If both
`dora-rs` (upstream) and `dora-rs` are installed in the same Python
environment, the `dora` package from whichever was installed last wins.

**Recommendation**: Use a virtual environment with only `dora-rs` installed
when running dora-hub nodes on dora.

### 6. `operator_types.h` is Auto-Generated

The C operator types header (`operator_types.h`) is generated by `safer_ffi`
and must not be edited manually. All dora-compat aliases for C operator types
live in `operator_api.h` instead. If `safer_ffi` regenerates `operator_types.h`
and renames types, the aliases in `operator_api.h` must be updated to match.

### 7. `dora_core` vs `dora_core` in Rust Path

The shim crate provides `dora_node_api::dora_core` as an alias for
`dora_node_api::dora_core`. However, if a dora-hub node uses
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
  `DoraOperator` trait bound, `DoraStatus` variants, `DoraOutputSender`
  type equivalence
- **Python shim**: `from dora import Node`, `DoraStatus` is `DoraStatus`,
  enum values (0/1/2), ROS2 types importable, `build()`/`run()` importable

## Migration Path (dora -> dora)

For developers migrating from dora to dora:

1. **No code changes required** for existing dora-hub nodes — the compat layer
   handles everything.
2. **To use dora-only features**, change imports from `dora` to `dora`
   (Python) or from `dora-node-api` to `dora-node-api` (Rust) and use the
   new APIs directly.
3. **New nodes** should use dora APIs directly (`DoraNode`, `DoraStatus`,
   `from dora import Node`).
4. **Mixed usage** is fine — `DoraNode` and `DoraNode` are the same type,
   so dora-native and dora-compat code can interoperate freely.
