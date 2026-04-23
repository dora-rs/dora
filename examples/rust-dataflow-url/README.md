# Rust Dataflow (Programmatic Build + Run)

Same three-node pipeline as [rust-dataflow](../rust-dataflow), but launched programmatically via `dora_cli::build()` and `dora_cli::run()` in `run.rs` instead of the CLI.

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

The `run.rs` harness calls dora CLI functions as a library:

```rust
use dora_cli::{BuildConfig, build, run};

fn main() -> eyre::Result<()> {
    build(BuildConfig {
        dataflow: "dataflow.yml".to_string(),
        force_local: true,
        ..Default::default()
    })?;
    run("dataflow.yml".to_string(), false)?;
    Ok(())
}
```

This demonstrates embedding dora's build and run steps in a Rust program -- useful for testing, CI, or custom orchestration.

## Run

```bash
cargo run --example rust-dataflow-url
```

Or using the CLI directly:

```bash
dora build dataflow.yml
dora run dataflow.yml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `dora_cli::build()` programmatic API | `run.rs` |
| `dora_cli::run()` programmatic API | `run.rs` |
| `build:` commands in YAML | YAML (all nodes) |
| Same topology, programmatic launch | vs rust-dataflow |
