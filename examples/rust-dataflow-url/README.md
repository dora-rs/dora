# Rust Dataflow (Programmatic Build + Run)

Same three-node pipeline as [rust-dataflow](../rust-dataflow), but launched programmatically via `adora_cli::build()` and `adora_cli::run()` in `run.rs` instead of the CLI.

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

The `run.rs` harness calls adora CLI functions as a library:

```rust
use adora_cli::{build, run};

fn main() -> eyre::Result<()> {
    build("dataflow.yml".to_string(), None, None, false, true)?;
    run("dataflow.yml".to_string(), false)?;
    Ok(())
}
```

This demonstrates embedding adora's build and run steps in a Rust program -- useful for testing, CI, or custom orchestration.

## Run

```bash
cargo run --example rust-dataflow-url
```

Or using the CLI directly:

```bash
adora build dataflow.yml
adora run dataflow.yml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `adora_cli::build()` programmatic API | `run.rs` |
| `adora_cli::run()` programmatic API | `run.rs` |
| `build:` commands in YAML | YAML (all nodes) |
| Same topology, programmatic launch | vs rust-dataflow |
