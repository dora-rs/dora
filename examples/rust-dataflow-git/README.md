# Rust Dataflow (Git-based)

Same three-node pipeline as [rust-dataflow](../rust-dataflow), but node source code is fetched from a Git repository with version pinning via tags.

## Architecture

```
timer (10ms) --> rust-node --> random --> rust-status-node --> status --> rust-sink
                               timer (100ms) -->
```

## How Git Loading Works

Instead of referencing local paths, each node specifies a Git URL and tag:

```yaml
- id: rust-node
  git: https://github.com/adora-rs/adora.git
  tag: v0.4.0
  build: cargo build -p rust-dataflow-example-node
  path: target/debug/rust-dataflow-example-node
```

Adora clones the repository at the pinned tag, runs the `build:` command inside the clone, then executes the built binary. This enables:
- **Version pinning**: `tag:` ensures reproducible builds
- **Remote nodes**: pull node code from any Git repository
- **Independent versioning**: each node can use a different tag or repo

## Run

```bash
cargo run --example rust-dataflow-git
```

Or directly:

```bash
adora build dataflow.yml
adora run dataflow.yml
```

## What This Demonstrates

| Feature | Where |
|---------|-------|
| `git:` for remote source fetching | YAML (all nodes) |
| `tag:` for version pinning | YAML (all nodes) |
| `build:` for post-clone compilation | YAML (all nodes) |
| Same topology, different deployment model | vs rust-dataflow |
