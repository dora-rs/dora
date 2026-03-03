# Adora Testing Guide

This guide covers how to run, write, and troubleshoot tests across the Adora workspace.

## Quick Start (5-minute validation)

Run these three commands to validate that the workspace is healthy:

```bash
# 1. Format check (~5s)
cargo fmt --all -- --check

# 2. Lint (~60s first run, cached after)
cargo clippy --all \
  --exclude adora-node-api-python \
  --exclude adora-operator-api-python \
  --exclude adora-ros2-bridge-python \
  -- -D warnings

# 3. Unit + integration tests (~90s first run)
cargo test --all \
  --exclude adora-node-api-python \
  --exclude adora-operator-api-python \
  --exclude adora-ros2-bridge-python
```

All three must pass before opening a PR. Python packages are excluded because they require maturin.

## Test Tiers

| Tier | What it covers | Command | Speed |
|------|---------------|---------|-------|
| **Format** | Code style | `cargo fmt --all -- --check` | ~5s |
| **Lint** | Warnings, correctness | `cargo clippy --all ...` | ~60s |
| **Unit** | Individual functions | `cargo test --all ...` | ~90s |
| **CLI** | Command parsing, validation | `cargo test -p adora-cli` | ~5s |
| **Integration** | Node I/O via env vars | `cargo test --test example-tests` | ~30s |
| **Smoke** | Full CLI lifecycle | `cargo test --test example-smoke -- --test-threads=1` | ~3min |
| **E2E** | Multi-dataflow scenarios | `cargo test --test ws-cli-e2e -- --ignored --test-threads=1` | ~2min |
| **Fault tolerance** | Restart policies, timeouts | `cargo test --test fault-tolerance-e2e` | ~45s |
| **Typos** | Spelling | Install [typos-cli](https://github.com/crate-ci/typos), then `typos` | ~2s |

## Tier Details

### Unit Tests

Unit tests live alongside the code they test using `#[cfg(test)]` modules. Key crates with tests:

| Crate | Test count | What's tested |
|-------|-----------|---------------|
| adora-arrow-convert | ~26 | Round-trip Arrow type conversions |
| adora-cli | ~96 | Command parsing, value parsers, log grep/filtering, JSON parsing, WebSocket client, cluster config |
| adora-coordinator | ~24 | WS control/daemon plane, health check, concurrent requests, artifact store, rate limiter, error sanitization |
| adora-coordinator-store | ~10 | In-memory and redb CRUD, schema versioning, persistence |
| adora-core | ~8 | Dataflow descriptor validation |
| adora-daemon | ~2 | Shlex argument parsing |
| adora-node-api | ~10 | Input tracking, service/action helpers (ID generation, send_service_request/response) |
| adora-log-utils | ~11 | Log parsing utilities |
| adora-message | ~36 | Common types, WS protocol, node/data IDs, metadata, auth tokens |
| ros2-bridge | ~30 | ROS2 message/service/action parsing |

Run a single crate's tests:

```bash
cargo test -p adora-cli
cargo test -p adora-core
cargo test -p adora-arrow-convert
```

### CLI Tests

CLI tests verify command parsing, argument validation, and value parsers without running any commands. They live in `#[cfg(test)]` modules inside the CLI crate.

**What's tested:**
- Clap schema validation (`Args::command().debug_assert()`)
- Parsing of every subcommand (`run`, `up`, `down`, `start`, `stop`, `list`, `logs`, `build`, `graph`, `new`, `status`, `inspect top`, `topic list/hz/echo`, `node list`)
- Rejection of unknown subcommands
- `--help` and `--version` exit codes
- Value parsers: `parse_store_spec` (coordinator store backend), `parse_window` (topic hz window)
- Utility functions: `parse_version_from_pip_show`

**How to run:**

```bash
cargo test -p adora-cli
```

**How to add new tests:**

When adding a new CLI subcommand or value parser, add a corresponding test in the `#[cfg(test)]` module of the same file. For subcommand parsing, add a `parse_ok` call in `binaries/cli/src/command/mod.rs`. For value parsers, add tests in the file that defines the parser function.

### Integration Tests (Node I/O)

File: `tests/example-tests.rs`

These tests run compiled node executables with pre-recorded inputs and compare outputs against expected baselines. No coordinator or daemon is needed.

```bash
cargo test --test example-tests
```

How it works:
1. Builds and runs a node crate (e.g., `rust-dataflow-example-node`)
2. Sets `ADORA_TEST_WITH_INPUTS` to a JSON file with timed events
3. Sets `ADORA_TEST_NO_OUTPUT_TIME_OFFSET=1` for deterministic output
4. Compares JSONL output against `tests/sample-inputs/expected-outputs-*.jsonl`

Sample input/output files live in `tests/sample-inputs/`.

### Smoke Tests

File: `tests/example-smoke.rs`

Two execution modes are tested for each applicable example:

- **Networked** (`adora up` + `adora start --detach` + poll + `adora stop` + `adora down`): exercises the full coordinator/daemon WS control plane.
- **Local** (`adora run --stop-after`): runs everything in-process, testing the single-process dataflow path.

```bash
# Must run single-threaded (shared coordinator port)
cargo test --test example-smoke -- --test-threads=1

# Run only networked or local tests
cargo test --test example-smoke smoke_rust -- --test-threads=1
cargo test --test example-smoke smoke_local -- --test-threads=1
```

A bash script is also available for quick local validation:

```bash
./scripts/smoke-all.sh              # all examples
./scripts/smoke-all.sh --rust-only  # Rust examples only
./scripts/smoke-all.sh --python-only # Python examples only
```

**Networked tests (17):**

| Test | Example | Timeout |
|------|---------|---------|
| `smoke_rust_dataflow` | rust-dataflow/dataflow.yml | 30s |
| `smoke_rust_dataflow_dynamic` | rust-dataflow/dataflow_dynamic.yml | 30s |
| `smoke_rust_dataflow_socket` | rust-dataflow/dataflow_socket.yml | 30s |
| `smoke_rust_dataflow_url` | rust-dataflow-url/dataflow.yml | 30s |
| `smoke_benchmark` | benchmark/dataflow.yml | 30s |
| `smoke_log_sink_file` | log-sink-file/dataflow.yml | 30s |
| `smoke_log_sink_alert` | log-sink-alert/dataflow.yml | 30s |
| `smoke_log_sink_tcp` | log-sink-tcp/dataflow.yml | 30s |
| `smoke_python_dataflow` | python-dataflow/dataflow.yml | 30s |
| `smoke_python_async` | python-async/dataflow.yaml | 15s |
| `smoke_python_drain` | python-drain/dataflow.yaml | 15s |
| `smoke_python_log` | python-log/dataflow.yaml | 15s |
| `smoke_python_logging` | python-logging/dataflow.yml | 15s |
| `smoke_python_multiple_arrays` | python-multiple-arrays/dataflow.yml | 15s |
| `smoke_python_concurrent_rw` | python-concurrent-rw/dataflow.yml | 15s |
| `smoke_service_example` | service-example/dataflow.yml | 30s |
| `smoke_action_example` | action-example/dataflow.yml | 30s |

**Local tests (9):**

| Test | Example | stop-after |
|------|---------|------------|
| `smoke_local_python_dataflow` | python-dataflow/dataflow.yml | 30s |
| `smoke_local_python_async` | python-async/dataflow.yaml | 10s |
| `smoke_local_python_drain` | python-drain/dataflow.yaml | 10s |
| `smoke_local_python_log` | python-log/dataflow.yaml | 10s |
| `smoke_local_python_logging` | python-logging/dataflow.yml | 10s |
| `smoke_local_python_multiple_arrays` | python-multiple-arrays/dataflow.yml | 10s |
| `smoke_local_python_concurrent_rw` | python-concurrent-rw/dataflow.yml | 10s |
| `smoke_local_service_example` | service-example/dataflow.yml | 10s |
| `smoke_local_action_example` | action-example/dataflow.yml | 10s |

Examples requiring special dependencies (webcam, CUDA, ROS2, C/C++ toolchain, multi-machine deploy) are not included in smoke tests.

### E2E Tests (WebSocket CLI)

File: `tests/ws-cli-e2e.rs`

Two groups:

**Non-ignored (fast):** Start an in-process coordinator and test `WsSession` directly:
```bash
cargo test --test ws-cli-e2e
```
- `cli_list_empty` -- empty dataflow listing
- `cli_status_no_daemon` -- daemon connectivity check
- `cli_stop_nonexistent` -- error for missing dataflows
- `cli_multiple_requests_same_session` -- session reuse

**Ignored (full stack):** Use `adora up` with real nodes:
```bash
cargo test --test ws-cli-e2e -- --ignored --test-threads=1
```
- `e2e_start_list_stop` -- start, list, stop lifecycle
- `e2e_sequential_dataflows` -- two dataflows in sequence

### Fault Tolerance Tests

File: `tests/fault-tolerance-e2e.rs`

These test restart policies and input timeouts using `Daemon::run_dataflow` directly (no CLI needed).

```bash
cargo test --test fault-tolerance-e2e
```

Tests:
- `restart_recovers_from_failure` -- node with `restart_policy: on-failure` survives panics (15s)
- `max_restarts_limit_reached` -- node exhausts `max_restarts: 2` budget (15s)
- `input_timeout_closes_stale_input` -- `input_timeout: 2.0s` fires when upstream stops (10s)

Dataflow YAMLs for these tests live in `tests/dataflows/`.

### Coordinator Integration Tests

Files: `binaries/coordinator/tests/ws_control_tests.rs`, `binaries/coordinator/tests/ws_daemon_tests.rs`

These start an in-process coordinator and test the WebSocket control/daemon planes.

```bash
cargo test -p adora-coordinator
```

Topics covered: health check, list/stop/destroy requests, invalid JSON/params, concurrent requests, ping/pong, daemon registration, disconnect cleanup, error sanitization (no internal chain leaks), artifact store cleanup on drop.

## CI Pipeline

CI runs on push/PR to `main`. See `.github/workflows/ci.yml`.

```
fmt  ──────────────┐
clippy ────────────┤ (all run in parallel)
test ──────────────┤
typos ─────────────┘
                   │
              e2e (depends on test)
```

| Job | Runner | What runs |
|-----|--------|-----------|
| **fmt** | ubuntu-latest | `cargo fmt --all -- --check` |
| **clippy** | ubuntu-latest | `cargo clippy --all ... -- -D warnings` |
| **test** | ubuntu-latest | `cargo test --all ...` (excl. Python + adora-examples) |
| **e2e** | ubuntu-latest | example-tests, fault-tolerance, smoke tests, WS E2E |
| **typos** | ubuntu-latest | `crate-ci/typos@master` |

The `e2e` job only runs after `test` passes. All other jobs run in parallel.

## Writing New Tests

### Unit tests

Add a `#[cfg(test)]` module in the same file as the code under test:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_valid_input() {
        let result = parse("valid");
        assert_eq!(result, expected);
    }
}
```

### Integration tests for nodes

Use the integration testing framework in `adora-node-api`. Three approaches:

**1. `setup_integration_testing` (recommended)**

Call before the node's `main` function to inject inputs and capture outputs:

```rust
#[test]
fn test_main_function() -> eyre::Result<()> {
    let events = vec![
        TimedIncomingEvent {
            time_offset_secs: 0.01,
            event: IncomingEvent::Input {
                id: "tick".into(),
                metadata: None,
                data: None,
            },
        },
        TimedIncomingEvent {
            time_offset_secs: 0.055,
            event: IncomingEvent::Stop,
        },
    ];
    let inputs = TestingInput::Input(
        IntegrationTestInput::new("node_id".parse().unwrap(), events),
    );
    let (tx, rx) = flume::unbounded();
    let outputs = TestingOutput::ToChannel(tx);
    let options = TestingOptions { skip_output_time_offsets: true };

    integration_testing::setup_integration_testing(inputs, outputs, options);
    crate::main()?;

    let outputs = rx.try_iter().collect::<Vec<_>>();
    assert_eq!(outputs, expected_outputs);
    Ok(())
}
```

**2. Environment variable mode**

Test the compiled executable directly, closest to production behavior:

```bash
ADORA_TEST_WITH_INPUTS=path/to/inputs.json \
ADORA_TEST_NO_OUTPUT_TIME_OFFSET=1 \
ADORA_TEST_WRITE_OUTPUTS_TO=/tmp/out.jsonl \
cargo run -p my-node
```

**3. `AdoraNode::init_testing`**

For testing node logic without going through `main`:

```rust
let (node, events) = AdoraNode::init_testing(inputs, outputs, Default::default())?;
```

### Generating test input files

Record real dataflow events by setting `ADORA_WRITE_EVENTS_TO`:

```bash
ADORA_WRITE_EVENTS_TO=/tmp/recorded-events adora run examples/rust-dataflow/dataflow.yml
```

This writes `inputs-{node_id}.json` files that can be used directly with `ADORA_TEST_WITH_INPUTS`.

### Workspace-level integration tests

Add new test files in the `tests/` directory. For tests that need the full CLI stack, follow the patterns in `tests/example-smoke.rs`:

**Networked pattern** (exercises coordinator + daemon):
1. Build nodes with `Once` guards (avoid rebuilding per test)
2. Clean up stale processes with `adora down`
3. Start cluster with `adora up`
4. Run dataflow with `adora start --detach`
5. Poll `adora list --json` for completion
6. Clean up with `adora stop --all` and `adora down`

**Local pattern** (single-process, in-process coordinator):
1. Build CLI with `Once` guard
2. Run `adora run <yaml> --stop-after <duration>`
3. Assert exit code is success

### Conventions

- Use `assert2::assert!` for better error messages (available as dev-dependency)
- Use `tempfile::NamedTempFile` for temporary output files
- E2E tests that need exclusive port access should be `#[ignore]` and run with `--test-threads=1`
- Async tests use `#[tokio::test(flavor = "multi_thread")]`
- Fault tolerance test dataflows go in `tests/dataflows/`
- Sample input/output baselines go in `tests/sample-inputs/`

## Troubleshooting

### `cargo test` fails to compile Python packages

Always exclude Python packages:
```bash
cargo test --all \
  --exclude adora-node-api-python \
  --exclude adora-operator-api-python \
  --exclude adora-ros2-bridge-python
```

### Smoke/E2E tests fail with "address already in use"

A stale coordinator or daemon is still running. Clean up:
```bash
adora down
# or kill processes manually:
pkill -f adora-coordinator
pkill -f adora-daemon
```

### Smoke tests hang or timeout

- Increase the timeout in the test if your machine is slow (look for `Duration::from_secs(...)`)
- Check that example nodes build successfully:
  ```bash
  cargo build -p rust-dataflow-example-node -p rust-dataflow-example-status-node \
    -p rust-dataflow-example-sink -p rust-dataflow-example-sink-dynamic
  cargo build -p log-sink-file -p log-sink-alert -p log-sink-tcp
  cargo build --release -p benchmark-example-node -p benchmark-example-sink
  ```
- For Python smoke tests, ensure `pyarrow` and `numpy` are installed

### E2E tests fail when run in parallel

Smoke and ignored E2E tests must run single-threaded:
```bash
cargo test --test example-smoke -- --test-threads=1
cargo test --test ws-cli-e2e -- --ignored --test-threads=1
```

### Integration test output doesn't match expected

1. Check that `ADORA_TEST_NO_OUTPUT_TIME_OFFSET=1` is set (time offsets vary per machine)
2. Regenerate baselines if the node's behavior intentionally changed:
   ```bash
   ADORA_TEST_WITH_INPUTS=tests/sample-inputs/inputs-rust-node.json \
   ADORA_TEST_NO_OUTPUT_TIME_OFFSET=1 \
   ADORA_TEST_WRITE_OUTPUTS_TO=tests/sample-inputs/expected-outputs-rust-node.jsonl \
   cargo run -p rust-dataflow-example-node
   ```

### Typos check fails

The typos config is in `_typos.toml`. To add a false-positive exclusion:
```toml
[default.extend-identifiers]
MyCustomIdent = "MyCustomIdent"
```

### Tests pass locally but fail in CI

- CI runs on Ubuntu; check for platform-specific assumptions (paths, process signals)
- CI uses `rust-cache` so dependency versions may differ from your local lockfile
- Ensure `cargo fmt --all -- --check` passes (CI enforces this)
