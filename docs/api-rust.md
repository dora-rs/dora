# Rust API Reference

This document covers the two main Rust crates for building Adora dataflow components:

- **`adora-node-api`** -- for standalone node executables
- **`adora-operator-api`** -- for in-process operators managed by the Adora runtime

---

## Node API (`adora-node-api`)

Add to your `Cargo.toml`:

```toml
[dependencies]
adora-node-api = { workspace = true }
```

### AdoraNode

The primary struct for sending outputs and retrieving node information. Obtained through one of the initialization functions below.

#### Initialization

```rust
// Recommended: auto-detect environment (daemon, testing, or interactive).
pub fn init_from_env() -> NodeResult<(Self, EventStream)>

// Same as init_from_env but errors instead of falling back to interactive mode.
pub fn init_from_env_force() -> NodeResult<(Self, EventStream)>

// For dynamic nodes: connect to the daemon by node ID.
pub fn init_from_node_id(node_id: NodeId) -> NodeResult<(Self, EventStream)>

// Try init_from_env first; fall back to init_from_node_id.
pub fn init_flexible(node_id: NodeId) -> NodeResult<(Self, EventStream)>

// Standalone interactive mode (prompts for inputs on the terminal).
pub fn init_interactive() -> NodeResult<(Self, EventStream)>

// Integration test mode with synthetic inputs/outputs.
pub fn init_testing(
    input: TestingInput,
    output: TestingOutput,
    options: TestingOptions,
) -> NodeResult<(Self, EventStream)>
```

`init_from_env` is the recommended entry point. It checks, in order:

1. Thread-local testing state set by `setup_integration_testing`
2. `ADORA_NODE_CONFIG` environment variable (set by the daemon)
3. `ADORA_TEST_WITH_INPUTS` environment variable (file-based integration testing)
4. Interactive terminal fallback (only if stdin is a TTY)

#### Sending Outputs

All send methods silently ignore output IDs not declared in the dataflow YAML.

```rust
// Send an Arrow array. Copies data into shared memory when beneficial.
pub fn send_output(
    &mut self,
    output_id: DataId,
    parameters: MetadataParameters,
    data: impl Array,
) -> NodeResult<()>

// Send raw bytes. Copies into shared memory when beneficial.
pub fn send_output_bytes(
    &mut self,
    output_id: DataId,
    parameters: MetadataParameters,
    data_len: usize,
    data: &[u8],
) -> NodeResult<()>

// Send raw bytes via a closure for zero-copy writing.
pub fn send_output_raw<F>(
    &mut self,
    output_id: DataId,
    parameters: MetadataParameters,
    data_len: usize,
    data: F,
) -> NodeResult<()>
where
    F: FnOnce(&mut [u8])

// Send raw bytes with explicit Arrow type information.
pub fn send_typed_output<F>(
    &mut self,
    output_id: DataId,
    type_info: ArrowTypeInfo,
    parameters: MetadataParameters,
    data_len: usize,
    data: F,
) -> NodeResult<()>
where
    F: FnOnce(&mut [u8])

// Send a pre-allocated DataSample with type information.
pub fn send_output_sample(
    &mut self,
    output_id: DataId,
    type_info: ArrowTypeInfo,
    parameters: MetadataParameters,
    sample: Option<DataSample>,
) -> NodeResult<()>

// Report output IDs as closed. No further sends allowed for those IDs.
pub fn close_outputs(&mut self, outputs_ids: Vec<DataId>) -> NodeResult<()>
```

#### Data Allocation

```rust
// Allocate a DataSample of the given size.
// Uses shared memory for data >= ZERO_COPY_THRESHOLD (4096 bytes).
pub fn allocate_data_sample(&mut self, data_len: usize) -> NodeResult<DataSample>
```

#### Node Information

```rust
// Node ID from the dataflow YAML.
pub fn id(&self) -> &NodeId

// Unique identifier for this dataflow run.
pub fn dataflow_id(&self) -> &DataflowId

// Input/output configuration for this node.
pub fn node_config(&self) -> &NodeRunConfig

// True if this node was restarted after a previous exit or failure.
pub fn is_restart(&self) -> bool

// Number of times this node has been restarted (0 on first run).
pub fn restart_count(&self) -> u32

// Parsed dataflow YAML descriptor.
pub fn dataflow_descriptor(&self) -> NodeResult<&Descriptor>
```

#### Logging

All log methods emit structured JSONL to stdout, which the daemon parses automatically. Works with `min_log_level` filtering and `send_logs_as` routing.

```rust
// General structured log. Level: "error", "warn", "info", "debug", "trace".
pub fn log(&self, level: &str, message: &str, target: Option<&str>)

// Structured log with additional key-value fields.
pub fn log_with_fields(
    &self,
    level: &str,
    message: &str,
    target: Option<&str>,
    fields: Option<&BTreeMap<String, String>>,
)

// Convenience methods (no target parameter).
pub fn log_error(&self, message: &str)
pub fn log_warn(&self, message: &str)
pub fn log_info(&self, message: &str)
pub fn log_debug(&self, message: &str)
pub fn log_trace(&self, message: &str)
```

---

### EventStream

Asynchronous iterator over incoming events destined for this node. Implements the `futures::Stream` trait.

The event stream closes itself after a `Stop` event is received. Nodes should exit once the stream ends.

```rust
// Block until the next event arrives. Returns None when the stream closes.
// Uses an internal EventScheduler that may reorder events for fairness.
pub fn recv(&mut self) -> Option<Event>

// Block with a timeout. Returns an Event::Error on timeout.
pub fn recv_timeout(&mut self, dur: Duration) -> Option<Event>

// Async receive with EventScheduler reordering.
pub async fn recv_async(&mut self) -> Option<Event>

// Async receive with a timeout. Returns Event::Error on timeout.
pub async fn recv_async_timeout(&mut self, dur: Duration) -> Option<Event>

// Non-blocking receive. Returns TryRecvError::Empty if nothing is ready.
pub fn try_recv(&mut self) -> Result<Event, TryRecvError>

// Drain all buffered events without blocking.
// Returns Some(Vec::new()) if nothing is ready; None if the stream is closed.
pub fn drain(&mut self) -> Option<Vec<Event>>

// True if no events are buffered in the scheduler or receiver.
pub fn is_empty(&self) -> bool
```

`EventStream` also implements `futures::Stream<Item = Event>`, so it can be used with `StreamExt::next()` and other combinators. Unlike `recv`/`recv_async`, the `Stream` implementation does **not** use the EventScheduler, preserving chronological event order.

---

### Event

Represents an incoming event. This enum is `#[non_exhaustive]` -- ignore unknown variants to stay forward-compatible.

```rust
#[non_exhaustive]
pub enum Event {
    // An input was received from another node.
    Input {
        id: DataId,           // input ID from the YAML (not the sender's output ID)
        metadata: Metadata,   // timestamp and type information
        data: ArrowData,      // Apache Arrow data
    },

    // The sender mapped to this input exited; no more data will arrive.
    InputClosed { id: DataId },

    // A previously closed input recovered (e.g., upstream node came back after timeout).
    InputRecovered { id: DataId },

    // An upstream node has restarted. Useful for resetting caches or state.
    NodeRestarted { id: NodeId },

    // The event stream is about to close. See StopCause for the reason.
    Stop(StopCause),

    // Instructs the node to reload an operator (used internally by the runtime).
    Reload { operator_id: Option<OperatorId> },

    // An unexpected internal error. Log it for debugging.
    Error(String),
}
```

### StopCause

```rust
#[non_exhaustive]
pub enum StopCause {
    // Explicit stop via `adora stop` or Ctrl-C. Exit promptly or be killed.
    Manual,

    // All inputs were closed (upstream nodes exited). Only sent if the node has inputs.
    AllInputsClosed,
}
```

---

### Supporting Types

#### DataSample

A data region suitable for sending as an output message. Uses shared memory for data >= `ZERO_COPY_THRESHOLD` to enable zero-copy transfer.

Implements `Deref<Target = [u8]>` and `DerefMut` for reading and writing the underlying bytes.

#### Metadata and MetadataParameters

```rust
// Full metadata attached to every input event.
pub struct Metadata {
    // Contains timestamp, Arrow type info, and user-defined parameters.
}

// User-controlled metadata fields attached when sending outputs.
pub struct MetadataParameters {
    // Default is empty. Pass metadata.parameters from an input to forward metadata.
}

// A single metadata parameter (key-value pair).
pub struct Parameter { /* ... */ }
```

#### Identity Types

```rust
// Unique identifier for a running dataflow instance (UUID v4).
pub struct DataflowId(/* ... */);

// Node identifier, as defined in the dataflow YAML.
pub struct NodeId(/* ... */);

// Input/output identifier, as defined in the dataflow YAML.
pub struct DataId(/* ... */);
```

#### Error Types

```rust
#[derive(Debug, Error)]
pub enum NodeError {
    Init(String),        // config parsing, env vars, daemon handshake
    Connection(String),  // daemon connection lost
    Output(String),      // send or close failure
    Data(String),        // allocation or descriptor parsing
    Internal(eyre::Report),  // catch-all for unexpected errors
}

pub type NodeResult<T> = Result<T, NodeError>;
```

#### TryRecvError

```rust
pub enum TryRecvError {
    Empty,   // no event available right now
    Closed,  // event stream has been closed
}
```

#### ZERO_COPY_THRESHOLD

```rust
pub const ZERO_COPY_THRESHOLD: usize = 4096;
```

Messages smaller than this threshold are sent via TCP. Messages at or above this size use shared memory for zero-copy transfer.

#### ArrowData

```rust
// Wrapper around arrow::array::ArrayRef. Implements Deref to the inner ArrayRef.
pub struct ArrowData(pub arrow::array::ArrayRef);
```

Data from `Event::Input` arrives as `ArrowData`. Use `TryFrom` conversions or Arrow APIs to extract typed values.

---

### InputTracker

Helper for tracking input health and caching the last received value per input. Useful for graceful degradation when upstream nodes time out.

```rust
pub struct InputTracker { /* ... */ }

impl InputTracker {
    pub fn new() -> Self

    // Update state from an event. Returns true if the event was relevant.
    pub fn process_event(&mut self, event: &Event) -> bool

    // Current state of an input (Healthy or Closed), if tracked.
    pub fn state(&self, id: &DataId) -> Option<InputState>

    // True if the input is currently closed.
    pub fn is_closed(&self, id: &DataId) -> bool

    // Last received value for an input. Available even when closed.
    pub fn last_value(&self, id: &DataId) -> Option<&ArrowData>

    // All inputs currently in Closed state.
    pub fn closed_inputs(&self) -> Vec<&DataId>

    // True if any tracked input is closed.
    pub fn any_closed(&self) -> bool
}

pub enum InputState {
    Healthy,  // receiving data normally
    Closed,   // upstream exited or timed out
}
```

---

### Integration Testing

The `integration_testing` module provides tools for testing nodes without a running daemon.

#### setup_integration_testing

Sets up thread-local state so that the next call to `AdoraNode::init_from_env` on the same thread initializes in test mode.

```rust
pub fn setup_integration_testing(
    input: TestingInput,
    output: TestingOutput,
    options: TestingOptions,
)
```

#### TestingInput

```rust
pub enum TestingInput {
    // Load events from a JSON file (must deserialize to IntegrationTestInput).
    FromJsonFile(PathBuf),

    // Provide events directly.
    Input(IntegrationTestInput),
}
```

#### TestingOutput

```rust
pub enum TestingOutput {
    // Write outputs to a JSONL file (created or overwritten).
    ToFile(PathBuf),

    // Write outputs as JSONL to any writer.
    ToWriter(Box<dyn std::io::Write + Send>),

    // Send each output as a JSON object to a flume channel.
    ToChannel(flume::Sender<serde_json::Map<String, serde_json::Value>>),
}
```

#### TestingOptions

```rust
#[derive(Debug, Clone, Default)]
pub struct TestingOptions {
    // Skip time offsets in outputs for deterministic comparison.
    pub skip_output_time_offsets: bool,
}
```

#### Environment Variable Testing

Nodes using `init_from_env` also support file-based testing via environment variables:

| Variable | Description |
|----------|-------------|
| `ADORA_TEST_WITH_INPUTS` | Path to a JSON input file (`IntegrationTestInput` format) |
| `ADORA_TEST_WRITE_OUTPUTS_TO` | Path for the output JSONL file (default: `outputs.jsonl` next to inputs) |
| `ADORA_TEST_NO_OUTPUT_TIME_OFFSET` | If set, omit time offsets for deterministic outputs |

---

## Operator API (`adora-operator-api`)

Operators are in-process components managed by the Adora runtime. They are compiled as shared libraries (`.so`/`.dylib`/`.dll`) and loaded by the runtime.

Add to your `Cargo.toml`:

```toml
[dependencies]
adora-operator-api = { workspace = true }

[lib]
crate-type = ["cdylib"]
```

### AdoraOperator Trait

```rust
pub trait AdoraOperator: Default {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut AdoraOutputSender,
    ) -> Result<AdoraStatus, String>;
}
```

Implement this trait to define your operator's behavior. The runtime calls `on_event` for each incoming event. Return `AdoraStatus` to control execution flow.

### Event (Operator)

The operator `Event` enum is simpler than the node `Event` and uses `&str` for IDs.

```rust
#[non_exhaustive]
pub enum Event<'a> {
    // An input was received.
    Input { id: &'a str, data: ArrowData },

    // Failed to parse the input data as an Arrow array.
    InputParseError { id: &'a str, error: String },

    // An input was closed by the sender.
    InputClosed { id: &'a str },

    // The operator should stop.
    Stop,
}
```

### AdoraOutputSender

```rust
pub struct AdoraOutputSender<'a>(/* ... */);

impl AdoraOutputSender<'_> {
    // Send an output. `id` is the output ID from your dataflow YAML.
    pub fn send(&mut self, id: String, data: impl Array) -> Result<(), String>
}
```

### AdoraStatus

Returned from `on_event` to control the operator lifecycle.

```rust
pub enum AdoraStatus {
    Continue,  // keep running, wait for the next event
    Stop,      // stop this operator
    StopAll,   // stop the entire dataflow
}
```

### register_operator! Macro

Generates the FFI entry points required by the Adora runtime to load and call your operator.

```rust
use adora_operator_api::register_operator;

register_operator!(MyOperator);
```

This must be called exactly once per crate, at the top level, with the type that implements `AdoraOperator`.

---

## Quick Start Example: Node

A minimal node that receives `tick` inputs and sends a random number as output.

```rust
use adora_node_api::{AdoraNode, Event, IntoArrow, adora_core::config::DataId};

fn main() -> eyre::Result<()> {
    let (mut node, mut events) = AdoraNode::init_from_env()?;

    let output = DataId::from("random".to_owned());

    while let Some(event) = events.recv() {
        match event {
            Event::Input { id, metadata, data } => {
                if id.as_str() == "tick" {
                    let value: u64 = fastrand::u64(..);
                    node.send_output(
                        output.clone(),
                        metadata.parameters,
                        value.into_arrow(),
                    )?;
                }
            }
            Event::Stop(_) => {}
            _ => {}
        }
    }

    Ok(())
}
```

Corresponding dataflow YAML:

```yaml
nodes:
  - id: timer
    path: adora/timer/millis/100
    outputs:
      - tick

  - id: my-node
    path: ./target/debug/my-node
    inputs:
      tick: timer/tick
    outputs:
      - random

  - id: sink
    path: ./target/debug/sink
    inputs:
      data: my-node/random
```

---

## Quick Start Example: Operator

A minimal operator that counts ticks and forwards formatted messages.

```rust
#![warn(unsafe_op_in_unsafe_fn)]

use adora_operator_api::{
    AdoraOperator, AdoraOutputSender, AdoraStatus, Event, IntoArrow, register_operator,
};

register_operator!(MyOperator);

#[derive(Debug, Default)]
struct MyOperator {
    ticks: usize,
}

impl AdoraOperator for MyOperator {
    fn on_event(
        &mut self,
        event: &Event,
        output_sender: &mut AdoraOutputSender,
    ) -> Result<AdoraStatus, String> {
        match event {
            Event::Input { id, data } => match *id {
                "tick" => {
                    self.ticks += 1;
                    let msg = format!("tick count: {}", self.ticks);
                    output_sender.send("status".into(), msg.into_arrow())?;
                }
                other => eprintln!("ignoring unexpected input {other}"),
            },
            Event::InputClosed { id } => {
                if *id == "tick" {
                    return Ok(AdoraStatus::Stop);
                }
            }
            Event::Stop => {}
            other => {
                eprintln!("received unknown event {other:?}");
            }
        }

        Ok(AdoraStatus::Continue)
    }
}
```

Corresponding dataflow YAML:

```yaml
nodes:
  - id: timer
    path: adora/timer/millis/500
    outputs:
      - tick

  - id: runtime-node
    operator:
      shared_library: ./target/debug/libmy_operator
      inputs:
        tick: timer/tick
      outputs:
        - status
```
