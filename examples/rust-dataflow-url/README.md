# Rust Dataflow (Programmatic Build + Run)

The [Rust runner](run.rs) builds and runs the same three local nodes as [rust-dataflow](../rust-dataflow). The Cargo example retains its historical name, `rust-dataflow-url`.

## Architecture

```
timer (10ms) --> rust-node --> random --> rust-status-node --> status --> rust-sink
                               timer (100ms) -->
```

## Nodes

**rust-node** -- Generates random values on each 10ms timer tick.

**rust-status-node** -- Combines random input with its own 100ms timer to produce status messages.

**rust-sink** -- Consumes and prints status messages.

## How It Works

The runner changes into the example directory and calls `build` with
`force_local: true` to build the binaries listed in [dataflow.yml](dataflow.yml).
It then starts the pipeline with `RunCommand`:

```rust
use dora_cli::{Executable, RunCommand};
use std::time::Duration;

let mut run = RunCommand::new("dataflow.yml".to_string());
run.stop_after = Some(Duration::from_secs(120));
run.execute()?;
```

The full runner also checks elapsed time after `execute()`. A node stopped
at the deadline can report a clean exit, so this check makes a hung pipeline
fail the example in CI.

## Run

```bash
cargo run --example rust-dataflow-url
```

To run the same descriptor directly, start in `examples/rust-dataflow-url`:

```bash
dora build dataflow.yml
dora run dataflow.yml
```
