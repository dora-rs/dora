# AGENTS.md

This file provides guidance to Codex and other agentic coding tools working in this repository. It is intended to complement, not replace, [`CLAUDE.md`](CLAUDE.md).

## Scope

- Use this file for agent-specific operating guidance.
- Use [`CLAUDE.md`](CLAUDE.md) for the main repository workflow, architecture, and QA expectations.
- If the two documents overlap, follow the stricter rule.

## Project Overview

Dora is a Rust-first framework for real-time robotics and AI applications. It uses zero-copy shared memory, Apache Arrow data, and supports Rust, Python, C, and C++ nodes.

Primary architecture:

```text
CLI -> Coordinator -> Daemon(s) -> Nodes / Operators
```

Key communication paths:

- CLI <-> Coordinator: WebSocket
- Coordinator <-> Daemon: WebSocket
- Daemon <-> Daemon: Zenoh
- Daemon <-> Node: shared memory for large payloads, TCP for small payloads

## Workspace Facts

- Rust edition: 2024
- MSRV: 1.88.0
- Shared workspace versioning
- Python packages are built with `maturin`, not normal `cargo` flows
- The repository is in an `adora` -> `dora` consolidation transition. Prefer `dora` names for new code, but preserve documented compatibility paths unless the change is explicitly a breaking cleanup.

Important packages:

- `binaries/cli`: `dora` CLI
- `binaries/daemon`: local process manager and transport bridge
- `binaries/coordinator`: distributed orchestration
- `binaries/runtime`: in-process operator runtime
- `libraries/core`: descriptor parsing and shared build/runtime utilities
- `libraries/message`: protocol and message definitions
- `apis/rust/node`: Rust node API
- `apis/rust/operator`: Rust operator API
- `apis/python/node`: Python node API

## Rename Transition Rules

- Default to `dora` naming in code, docs, examples, and user-facing text.
- Do not remove `adora` compatibility aliases lightly. If a path currently supports old `adora` prefixes, env vars, imports, or crate/package shims, assume that compatibility is intentional unless the task explicitly removes it.
- When touching renamed surfaces, verify both:
  - the new `dora` path works
  - any documented transition compatibility still works
- Treat these as compatibility-sensitive areas:
  - `DORA_*` / `ADORA_*` environment variables
  - `dora/...` / `adora/...` virtual YAML inputs
  - public API aliases and shim crates/packages
  - CLI binary/package/import names across Rust, Python, C, and C++

## Agent Working Rules

- Do not make up architecture details. Read the affected crate(s) first.
- Prefer minimal, targeted changes over broad refactors.
- Preserve existing style and naming in the touched area.
- Do not fix unrelated warnings or unrelated formatting drift.
- Never revert user changes you did not author.
- For non-trivial changes, keep behavior aligned with docs, examples, and tests.
- When touching CLI, coordinator, daemon, dataflow parsing, or transport behavior, assume smoke or integration validation is needed.
- For large mechanical rename or migration changes, review compatibility promises in issues/docs before changing behavior.

## Build And Test Commands

Normal workspace commands:

```bash
cargo build --all --exclude dora-node-api-python --exclude dora-operator-api-python --exclude dora-ros2-bridge-python

cargo check --all

cargo test --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  --exclude dora-cli-api-python \
  --exclude dora-examples

cargo clippy --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  -- -D warnings

cargo fmt --all -- --check
```

Useful targeted commands while iterating:

```bash
cargo test -p <crate>
cargo test -p <crate> <test_name>
cargo clippy -p <crate> -- -D warnings
```

Examples and local runs:

```bash
cargo run --example rust-dataflow
dora run examples/rust-dataflow/dataflow.yml
dora run examples/python-dataflow/dataflow.yml --uv --stop-after 10s
```

## Required Development Workflow

Follow RED-GREEN-IMPROVE:

1. Add or update a failing test that captures the intended behavior.
2. Implement the smallest change that makes the test pass.
3. Refactor only while keeping tests green.

Default test selection:

- Library logic: unit test near the code
- Coordinator/daemon behavior: integration test
- CLI flags or commands: smoke or integration test
- Dataflow behavior: smoke tests in both local and networked modes
- Bug fixes: regression test at the narrowest useful layer

Relevant test entry points:

- `tests/example-smoke.rs`
- `tests/ws-cli-e2e.rs`
- `tests/fault-tolerance-e2e.rs`
- `scripts/smoke-all.sh`

## Validation Expectations

Before finishing substantial code changes, run the smallest set that proves correctness:

- Always: targeted tests for touched crates or behavior
- Usually: `cargo fmt --all -- --check`
- Usually: `cargo clippy -p <crate> -- -D warnings` or workspace clippy if cross-cutting
- When changing CLI/coordinator/daemon/dataflow flows: relevant smoke or integration tests

Before a branch is ready to push, the expected local baseline is:

```bash
cargo fmt --all -- --check
cargo clippy --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  -- -D warnings
cargo test --all \
  --exclude dora-node-api-python \
  --exclude dora-operator-api-python \
  --exclude dora-ros2-bridge-python \
  --exclude dora-cli-api-python \
  --exclude dora-examples
```

If available and appropriate, `make qa-fast` is the preferred quick pre-commit sanity check.

## Change Guidance By Area

### CLI

- Keep user-facing help, docs, and examples in sync with flag or command changes.
- Prefer extending existing command patterns rather than introducing inconsistent UX.

### Coordinator / Daemon

- Be careful with lifecycle transitions, reconnect behavior, and state reconstruction.
- Treat distributed behavior, retries, and timeouts as regression-prone areas.

### Dataflow / YAML

- Preserve backward compatibility unless the change explicitly introduces a documented break.
- Update schema docs or examples if user-visible config changes.

### Python APIs

- Remember Python packages use `maturin`.
- Do not assume Rust-only validation is enough when changing Python bindings.

## Documentation Sync

When behavior changes, check whether any of these need updates:

- `README.md`
- `docs/`
- example dataflows under `examples/`
- crate-level README files
- smoke tests covering examples

## Commit And Review Hygiene

- Keep commits scoped and descriptive.
- Conventional commit style is preferred: `feat`, `fix`, `refactor`, `docs`, `test`, `chore`, `perf`, `ci`.
- Summaries for review should focus on behavior change, risk, and validation performed.

## Practical Notes For Agents

- Prefer `rg` for search.
- Prefer reading the local crate and tests before proposing design changes.
- If a change spans multiple crates, trace the message and control flow end-to-end before editing.
- If you cannot run a required validation step, say so explicitly and state what remains unverified.
