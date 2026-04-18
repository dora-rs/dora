//! Smoke tests for example dataflows.
//!
//! Two modes are exercised:
//!
//! - **Networked** (`dora up` + `dora start --detach` + poll + `dora stop` +
//!   `dora down`): exercises the full coordinator/daemon WS control plane.
//! - **Local** (`dora run --stop-after`): runs everything in-process, testing
//!   the single-process dataflow path.
//!
//! Run with: `cargo test --test example-smoke -- --test-threads=1`

use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::Once;
use std::time::Duration;

static BUILD_CLI: Once = Once::new();
static BUILD_RUST_NODES: Once = Once::new();
static BUILD_BENCHMARK_NODES: Once = Once::new();
static BUILD_LOG_SINK_NODES: Once = Once::new();
static BUILD_SERVICE_NODES: Once = Once::new();
static BUILD_ACTION_NODES: Once = Once::new();
static BUILD_CROSS_LANGUAGE_NODES: Once = Once::new();
static BUILD_VALIDATED_PIPELINE_NODES: Once = Once::new();
static BUILD_QUEUE_LATEST_RUST: Once = Once::new();

fn dora_bin() -> String {
    let manifest = env!("CARGO_MANIFEST_DIR");
    let target_dir = Path::new(manifest).join("target/debug/dora");
    if target_dir.exists() {
        return target_dir.to_string_lossy().to_string();
    }
    "dora".to_string()
}

fn ensure_cli_built() {
    BUILD_CLI.call_once(|| {
        let status = Command::new("cargo")
            .args(["build", "-p", "dora-cli"])
            .status()
            .expect("failed to run cargo build for CLI");
        assert!(status.success(), "failed to build dora CLI");
    });
}

fn ensure_rust_nodes_built() {
    BUILD_RUST_NODES.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "rust-dataflow-example-node",
                "-p",
                "rust-dataflow-example-status-node",
                "-p",
                "rust-dataflow-example-sink",
                "-p",
                "rust-dataflow-example-sink-dynamic",
            ])
            .status()
            .expect("failed to run cargo build");
        assert!(status.success(), "failed to build example nodes");
    });
}

fn ensure_benchmark_nodes_built() {
    BUILD_BENCHMARK_NODES.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "--release",
                "-p",
                "benchmark-example-node",
                "-p",
                "benchmark-example-sink",
            ])
            .status()
            .expect("failed to run cargo build for benchmark");
        assert!(status.success(), "failed to build benchmark nodes");
    });
}

fn ensure_log_sink_nodes_built() {
    BUILD_LOG_SINK_NODES.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "log-sink-file",
                "-p",
                "log-sink-alert",
                "-p",
                "log-sink-tcp",
            ])
            .status()
            .expect("failed to run cargo build for log sinks");
        assert!(status.success(), "failed to build log sink nodes");
    });
}

fn ensure_service_nodes_built() {
    BUILD_SERVICE_NODES.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "service-example-client",
                "-p",
                "service-example-server",
            ])
            .status()
            .expect("failed to run cargo build for service example");
        assert!(status.success(), "failed to build service example nodes");
    });
}

fn ensure_action_nodes_built() {
    BUILD_ACTION_NODES.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "action-example-client",
                "-p",
                "action-example-server",
            ])
            .status()
            .expect("failed to run cargo build for action example");
        assert!(status.success(), "failed to build action example nodes");
    });
}

/// Ensure no leftover coordinator/daemon from a previous test or manual run.
fn cleanup_stale(dora: &str) {
    let _ = Command::new(dora)
        .args(["stop", "--all"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    let _ = Command::new(dora)
        .arg("destroy")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    let _ = Command::new(dora)
        .arg("down")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    // Wait for port 6013 to be fully released (TCP TIME_WAIT)
    std::thread::sleep(Duration::from_secs(1));
}

/// Detect whether a dataflow YAML uses Python nodes (needs --uv).
fn needs_uv(yaml_path: &Path) -> bool {
    std::fs::read_to_string(yaml_path)
        .map(|content| content.contains(".py") || content.contains("pip install"))
        .unwrap_or(false)
}

/// Run an example dataflow through the full WS control plane lifecycle.
///
/// 1. `dora up` -- start coordinator + daemon
/// 2. `dora start <yaml> --detach` -- launch the dataflow
/// 3. Poll `dora list --json` until "Running" disappears or timeout
/// 4. `dora stop --all` + `dora down` -- clean up
fn run_smoke_test(name: &str, yaml_path: &str, timeout: Duration) {
    ensure_cli_built();

    let dora = dora_bin();
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let full_yaml = Path::new(manifest_dir).join(yaml_path);
    assert!(
        full_yaml.exists(),
        "{name}: dataflow YAML not found at {full_yaml:?}"
    );

    cleanup_stale(&dora);

    let uv = needs_uv(&full_yaml);

    // `dora up` starts coordinator + daemon and returns when both are ready.
    // Use Stdio::null() for all streams to prevent child processes from
    // keeping inherited pipe fds open.
    let up_status = Command::new(&dora)
        .arg("up")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .unwrap_or_else(|e| panic!("{name}: failed to run dora up: {e}"));

    assert!(up_status.success(), "{name}: dora up failed");

    // Build first so start has resolved artifacts (required for Python/git/etc.)
    let mut build_cmd = Command::new(&dora);
    build_cmd.args(["build", full_yaml.to_str().unwrap()]);
    if uv {
        build_cmd.arg("--uv");
    }
    let build_status = build_cmd
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .unwrap_or_else(|e| panic!("{name}: failed to run dora build: {e}"));
    assert!(build_status.success(), "{name}: dora build failed");

    // Start dataflow (detach so we get control back immediately)
    let mut start_cmd = Command::new(&dora);
    start_cmd.args(["start", full_yaml.to_str().unwrap(), "--detach"]);
    if uv {
        start_cmd.arg("--uv");
    }
    let start_status = start_cmd
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .unwrap_or_else(|e| panic!("{name}: failed to start dataflow: {e}"));

    assert!(start_status.success(), "{name}: dora start failed");

    // Wait for completion or timeout
    let start_time = std::time::Instant::now();
    while start_time.elapsed() < timeout {
        std::thread::sleep(Duration::from_secs(2));

        // Check if any dataflows are still running
        let list_result = Command::new(&dora)
            .args(["list", "--json"])
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .output()
            .ok();
        if let Some(output) = list_result {
            let stdout = String::from_utf8_lossy(&output.stdout);
            if stdout.contains("Failed") {
                // Clean up before panicking
                let _ = Command::new(&dora)
                    .args(["stop", "--all"])
                    .stdout(Stdio::null())
                    .stderr(Stdio::null())
                    .status();
                let _ = Command::new(&dora)
                    .arg("down")
                    .stdout(Stdio::null())
                    .stderr(Stdio::null())
                    .status();
                panic!("{name}: dataflow entered Failed state");
            }
            if stdout.trim().is_empty() || !stdout.contains("Running") {
                break;
            }
        }
    }

    // Clean up: stop all dataflows and destroy coordinator+daemon
    cleanup_stale(&dora);
}

/// Run an example dataflow locally with `dora run --stop-after`.
///
/// Exercises the single-process path where CLI, coordinator, and daemon
/// all run in the same process.
fn run_smoke_test_local(name: &str, yaml_path: &str, stop_after_secs: u64) {
    ensure_cli_built();

    let dora = dora_bin();
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let full_yaml = Path::new(manifest_dir).join(yaml_path);
    assert!(
        full_yaml.exists(),
        "{name}: dataflow YAML not found at {full_yaml:?}"
    );

    let uv = needs_uv(&full_yaml);

    // Build first so local runs also have resolved artifacts (required for
    // Python/git/etc. and examples that don't prebuild their nodes in the test).
    let mut build_cmd = Command::new(&dora);
    build_cmd.args(["build", full_yaml.to_str().unwrap()]);
    if uv {
        build_cmd.arg("--uv");
    }
    let build_status = build_cmd
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .unwrap_or_else(|e| panic!("{name}: failed to run dora build: {e}"));
    assert!(build_status.success(), "{name}: dora build failed");

    let stop_after = format!("{stop_after_secs}s");
    let mut cmd = Command::new(&dora);
    cmd.args([
        "run",
        full_yaml.to_str().unwrap(),
        "--stop-after",
        &stop_after,
    ]);
    if uv {
        cmd.arg("--uv");
    }
    let output = cmd
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .output()
        .unwrap_or_else(|e| panic!("{name}: failed to run dora run: {e}"));

    assert!(
        output.status.success(),
        "{name}: dora run failed\nstderr:\n{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

// ---------------------------------------------------------------------------
// Rust dataflow examples (pre-built nodes)
// ---------------------------------------------------------------------------

#[test]
fn smoke_rust_dataflow() {
    ensure_rust_nodes_built();
    run_smoke_test(
        "rust-dataflow",
        "examples/rust-dataflow/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_rust_dataflow_dynamic() {
    ensure_rust_nodes_built();
    run_smoke_test(
        "rust-dataflow-dynamic",
        "examples/rust-dataflow/dataflow_dynamic.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_rust_dataflow_url() {
    ensure_rust_nodes_built();
    run_smoke_test(
        "rust-dataflow-url",
        "examples/rust-dataflow-url/dataflow.yml",
        Duration::from_secs(30),
    );
}

// ---------------------------------------------------------------------------
// Benchmark example (release build)
// ---------------------------------------------------------------------------

#[test]
fn smoke_benchmark() {
    ensure_benchmark_nodes_built();
    run_smoke_test(
        "benchmark",
        "examples/benchmark/dataflow.yml",
        Duration::from_secs(30),
    );
}

// ---------------------------------------------------------------------------
// Log-sink examples (Rust sinks + Python sources)
// ---------------------------------------------------------------------------

#[test]
fn smoke_log_sink_file() {
    ensure_log_sink_nodes_built();
    run_smoke_test(
        "log-sink-file",
        "examples/log-sink-file/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_log_sink_alert() {
    ensure_log_sink_nodes_built();
    run_smoke_test(
        "log-sink-alert",
        "examples/log-sink-alert/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_log_sink_tcp() {
    ensure_log_sink_nodes_built();
    run_smoke_test(
        "log-sink-tcp",
        "examples/log-sink-tcp/dataflow.yml",
        Duration::from_secs(30),
    );
}

// ---------------------------------------------------------------------------
// Python examples (all use networked lifecycle: up/start/stop/destroy)
//
// Timer-driven examples run until the timeout, then get stopped.
// Self-terminating examples (e.g. python-dataflow) exit on their own.
// ---------------------------------------------------------------------------

#[test]
fn smoke_python_dataflow() {
    run_smoke_test(
        "python-dataflow",
        "examples/python-dataflow/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_python_async() {
    run_smoke_test(
        "python-async",
        "examples/python-async/dataflow.yaml",
        Duration::from_secs(15),
    );
}

#[test]
fn smoke_python_echo() {
    run_smoke_test(
        "python-echo",
        "examples/python-echo/dataflow.yml",
        Duration::from_secs(15),
    );
}

#[test]
fn smoke_python_drain() {
    run_smoke_test(
        "python-drain",
        "examples/python-drain/dataflow.yaml",
        Duration::from_secs(15),
    );
}

#[test]
fn smoke_python_log() {
    run_smoke_test(
        "python-log",
        "examples/python-log/dataflow.yaml",
        Duration::from_secs(15),
    );
}

#[test]
fn smoke_python_logging() {
    run_smoke_test(
        "python-logging",
        "examples/python-logging/dataflow.yml",
        Duration::from_secs(15),
    );
}

#[test]
fn smoke_python_multiple_arrays() {
    run_smoke_test(
        "python-multiple-arrays",
        "examples/python-multiple-arrays/dataflow.yml",
        Duration::from_secs(15),
    );
}

#[test]
fn smoke_python_concurrent_rw() {
    run_smoke_test(
        "python-concurrent-rw",
        "examples/python-concurrent-rw/dataflow.yml",
        Duration::from_secs(15),
    );
}

#[test]
fn smoke_typed_dataflow() {
    run_smoke_test(
        "typed-dataflow",
        "examples/typed-dataflow/dataflow.yml",
        Duration::from_secs(15),
    );
}

#[test]
fn smoke_streaming_example() {
    run_smoke_test(
        "streaming-example",
        "examples/streaming-example/dataflow.yml",
        Duration::from_secs(15),
    );
}

// ---------------------------------------------------------------------------
// Local-mode tests (dora run --stop-after)
//
// Same examples as above but exercising the single-process path.
// ---------------------------------------------------------------------------

#[test]
fn smoke_local_python_dataflow() {
    run_smoke_test_local(
        "local-python-dataflow",
        "examples/python-dataflow/dataflow.yml",
        30,
    );
}

#[test]
fn smoke_local_python_async() {
    run_smoke_test_local(
        "local-python-async",
        "examples/python-async/dataflow.yaml",
        20,
    );
}

#[test]
fn smoke_local_python_echo() {
    run_smoke_test_local("local-python-echo", "examples/python-echo/dataflow.yml", 10);
}

#[test]
fn smoke_local_python_drain() {
    run_smoke_test_local(
        "local-python-drain",
        "examples/python-drain/dataflow.yaml",
        10,
    );
}

#[test]
fn smoke_local_python_log() {
    run_smoke_test_local("local-python-log", "examples/python-log/dataflow.yaml", 10);
}

#[test]
fn smoke_local_python_logging() {
    run_smoke_test_local(
        "local-python-logging",
        "examples/python-logging/dataflow.yml",
        10,
    );
}

#[test]
fn smoke_local_python_multiple_arrays() {
    run_smoke_test_local(
        "local-python-multiple-arrays",
        "examples/python-multiple-arrays/dataflow.yml",
        10,
    );
}

#[test]
fn smoke_local_python_concurrent_rw() {
    // 15s gives more headroom on slow CI runners — this example has two
    // Python nodes in circular data dependency with threaded publish/read,
    // and each publishes every 1s; 10s was too tight under load.
    run_smoke_test_local(
        "local-python-concurrent-rw",
        "examples/python-concurrent-rw/dataflow.yml",
        15,
    );
}

// ---------------------------------------------------------------------------
// Module dataflow example
// ---------------------------------------------------------------------------

#[test]
fn smoke_module_dataflow() {
    run_smoke_test(
        "module-dataflow",
        "examples/module-dataflow/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_local_module_dataflow() {
    run_smoke_test_local(
        "local-module-dataflow",
        "examples/module-dataflow/dataflow.yml",
        10,
    );
}

// ---------------------------------------------------------------------------
// Service example
// ---------------------------------------------------------------------------

#[test]
fn smoke_service_example() {
    ensure_service_nodes_built();
    run_smoke_test(
        "service-example",
        "examples/service-example/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_local_service_example() {
    ensure_service_nodes_built();
    run_smoke_test_local(
        "local-service-example",
        "examples/service-example/dataflow.yml",
        10,
    );
}

// ---------------------------------------------------------------------------
// Action example
// ---------------------------------------------------------------------------

#[test]
fn smoke_action_example() {
    ensure_action_nodes_built();
    run_smoke_test(
        "action-example",
        "examples/action-example/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_local_action_example() {
    ensure_action_nodes_built();
    run_smoke_test_local(
        "local-action-example",
        "examples/action-example/dataflow.yml",
        15,
    );
}

#[test]
fn smoke_local_log_aggregator() {
    run_smoke_test_local(
        "local-log-aggregator",
        "examples/log-aggregator/dataflow.yml",
        10,
    );
}

#[test]
fn smoke_local_typed_dataflow() {
    run_smoke_test_local(
        "local-typed-dataflow",
        "examples/typed-dataflow/dataflow.yml",
        10,
    );
}

#[test]
fn smoke_local_streaming_example() {
    run_smoke_test_local(
        "local-streaming-example",
        "examples/streaming-example/dataflow.yml",
        10,
    );
}

// ---------------------------------------------------------------------------
// Cross-language interoperability tests (Rust <-> Python)
// ---------------------------------------------------------------------------

fn ensure_cross_language_nodes_built() {
    BUILD_CROSS_LANGUAGE_NODES.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "cross-language-rust-sender",
                "-p",
                "cross-language-rust-receiver",
            ])
            .status()
            .expect("failed to run cargo build for cross-language nodes");
        assert!(status.success(), "failed to build cross-language nodes");
    });
}

#[test]
fn smoke_cross_language_rust_to_python() {
    ensure_cross_language_nodes_built();
    run_smoke_test(
        "cross-language-rust-to-python",
        "examples/cross-language/rust-to-python.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_cross_language_python_to_rust() {
    ensure_cross_language_nodes_built();
    run_smoke_test(
        "cross-language-python-to-rust",
        "examples/cross-language/python-to-rust.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_local_cross_language_rust_to_python() {
    ensure_cross_language_nodes_built();
    run_smoke_test_local(
        "local-cross-language-rust-to-python",
        "examples/cross-language/rust-to-python.yml",
        15,
    );
}

#[test]
fn smoke_local_cross_language_python_to_rust() {
    ensure_cross_language_nodes_built();
    run_smoke_test_local(
        "local-cross-language-python-to-rust",
        "examples/cross-language/python-to-rust.yml",
        15,
    );
}

// ---------------------------------------------------------------------------
// Python recv_async() smoke test (GIL/async deadlock regression)
// ---------------------------------------------------------------------------

#[test]
fn smoke_python_recv_async() {
    run_smoke_test(
        "python-recv-async",
        "examples/python-recv-async/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_local_python_recv_async() {
    run_smoke_test_local(
        "local-python-recv-async",
        "examples/python-recv-async/dataflow.yml",
        15,
    );
}

// ---------------------------------------------------------------------------
// Validated pipeline: deterministic source -> transform -> sink with assertions
// ---------------------------------------------------------------------------

fn ensure_validated_pipeline_nodes_built() {
    BUILD_VALIDATED_PIPELINE_NODES.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "validated-pipeline-source",
                "-p",
                "validated-pipeline-transform",
                "-p",
                "validated-pipeline-sink",
            ])
            .status()
            .expect("failed to run cargo build for validated-pipeline nodes");
        assert!(status.success(), "failed to build validated-pipeline nodes");
    });
}

#[test]
fn smoke_validated_pipeline() {
    ensure_validated_pipeline_nodes_built();
    run_smoke_test(
        "validated-pipeline",
        "examples/validated-pipeline/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_local_validated_pipeline() {
    ensure_validated_pipeline_nodes_built();
    run_smoke_test_local(
        "local-validated-pipeline",
        "examples/validated-pipeline/dataflow.yml",
        15,
    );
}

/// Semantic contract test for `validated-pipeline` (#1630).
///
/// The liveness-only `smoke_local_validated_pipeline` above catches crashes
/// and `Failed` state, but it would still pass if the sink silently
/// received **zero** messages (as long as the node exited cleanly). That is
/// exactly the regression class the issue is asking us to close: an
/// example-backed test should prove the advertised feature behavior, not
/// just process exit.
///
/// The contract we assert here:
///
/// - Source emits exactly 10 `Int64` values (`0..=9`) over a 50ms timer,
///   then exits (see `examples/validated-pipeline/source/src/main.rs`).
/// - Transform doubles each value (`0, 2, …, 18`).
/// - Sink verifies `value == received * 2` and, on clean completion,
///   prints `"sink: SUCCESS - validated 10 doubled values"` to stderr.
///
/// The test captures combined stdout+stderr from `dora run --stop-after`
/// and asserts the exact success marker appears. If any layer of the
/// pipeline silently drops messages, truncates the run, or produces
/// incorrect doubled values, the marker won't match and the test fails
/// with the full dataflow log attached for triage.
#[test]
fn contract_validated_pipeline_produces_exactly_ten_doubled_outputs() {
    ensure_cli_built();
    ensure_validated_pipeline_nodes_built();

    let dora = dora_bin();
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let yaml = Path::new(manifest_dir).join("examples/validated-pipeline/dataflow.yml");
    assert!(yaml.exists(), "validated-pipeline YAML missing at {yaml:?}");

    // 15s is far above the ~500ms natural runtime (10 outputs × 50ms tick +
    // teardown). `--stop-after` just bounds worst-case; the dataflow should
    // finish and dora run should exit before the timer fires.
    let output = Command::new(&dora)
        .args(["run", yaml.to_str().unwrap(), "--stop-after", "15s"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("failed to spawn dora run for validated-pipeline contract test");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);

    let success_marker = "sink: SUCCESS - validated 10 doubled values";
    let combined_contains_marker =
        stdout.contains(success_marker) || stderr.contains(success_marker);

    assert!(
        combined_contains_marker,
        "validated-pipeline did not reach the 10-message SUCCESS milestone.\n\
         expected marker: {success_marker:?}\n\
         ---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );

    assert!(
        output.status.success(),
        "dora run exited non-zero: {:?}\n\
         ---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}",
        output.status
    );
}

/// Emit a test fixture YAML for validated-pipeline that uses absolute
/// paths and no `build:` directives, so `dora record` / `dora replay`
/// (which write their modified YAML into a `/tmp` tempfile) don't trip
/// over `cargo build -p …` failing from a directory with no Cargo.toml
/// in its parent chain. Returns the fixture path; caller deletes after
/// the test. Pre-built binaries are a prerequisite — callers should run
/// `ensure_validated_pipeline_nodes_built()` first.
fn write_absolute_path_validated_pipeline_fixture() -> std::path::PathBuf {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let target_debug = manifest_dir.join("target/debug");
    let source_bin = target_debug.join("validated-pipeline-source");
    let transform_bin = target_debug.join("validated-pipeline-transform");
    let sink_bin = target_debug.join("validated-pipeline-sink");
    for b in [&source_bin, &transform_bin, &sink_bin] {
        assert!(b.exists(), "pre-built binary missing at {b:?}");
    }

    let yaml = format!(
        r#"nodes:
  - id: source
    path: {source}
    inputs:
      tick: dora/timer/millis/50
    outputs:
      - value
    output_types:
      value: std/core/v1/Int64

  - id: transform
    path: {transform}
    inputs:
      value: source/value
    input_types:
      value: std/core/v1/Int64
    outputs:
      - doubled
    output_types:
      doubled: std/core/v1/Int64

  - id: sink
    path: {sink}
    inputs:
      doubled: transform/doubled
    input_types:
      doubled: std/core/v1/Int64
"#,
        source = source_bin.display(),
        transform = transform_bin.display(),
        sink = sink_bin.display(),
    );
    let path = manifest_dir.join("tests/dataflows/record-replay-validated-fixture.yml");
    std::fs::write(&path, yaml).expect("failed to write record/replay test fixture");
    path
}

/// Semantic record/replay equivalence (#1660).
///
/// The nightly `record-replay` subjob proves `dora record` produces a
/// `.drec` file and `dora replay` doesn't crash. That's weaker than
/// the contract users care about: **replay reproduces the downstream
/// behavior the recording captured**.
///
/// Fixture: validated-pipeline (source emits `0..=9`, transform
/// doubles, sink `bail!`s unless it sees each `received * 2` and
/// prints `sink: SUCCESS - validated 10 doubled values` at the end —
/// see `examples/validated-pipeline/sink/src/main.rs`).
///
/// Protocol:
///
/// 1. `dora record` runs the dataflow live; capture stdout+stderr and
///    assert the SUCCESS marker (same assertion as
///    `contract_validated_pipeline_produces_exactly_ten_doubled_outputs`).
///    This is the "recorded run" baseline.
/// 2. `dora replay` replaces the recorded source with
///    `dora-replay-node` (it plays back the captured `value` outputs);
///    transform + sink run live and must produce the *same* SUCCESS
///    marker. Anything that silently desyncs replayed values from
///    original would fail sink's in-process `value == received * 2`
///    check and never print the marker.
///
/// A regression in `dora-recording` / `dora-record-node` /
/// `dora-replay-node` that reordered, duplicated, or dropped
/// messages, or a regression that broke the replay-node substitution
/// path in `binaries/cli/src/command/replay.rs`, would fail this
/// test — the existing nightly smoke wouldn't.
#[test]
fn contract_record_replay_reproduces_validated_pipeline() {
    ensure_cli_built();
    ensure_validated_pipeline_nodes_built();

    // record spawns `dora-record-node`; replay spawns `dora-replay-node`.
    // Pre-build so neither CLI step falls into a cold cargo compile
    // (which, per PR #1641, can take minutes on a fresh rust-cache).
    let build_status = Command::new("cargo")
        .args(["build", "-p", "dora-record-node", "-p", "dora-replay-node"])
        .status()
        .expect("failed to build record/replay nodes");
    assert!(
        build_status.success(),
        "failed to build dora-record-node / dora-replay-node"
    );

    let dora = dora_bin();
    // Use the absolute-path fixture instead of examples/validated-pipeline/dataflow.yml:
    // `dora record` writes its modified descriptor into a tempfile in
    // /tmp, so any `build: cargo build -p …` directive carried over
    // from the original YAML runs cargo from /tmp — where there's no
    // Cargo.toml. Stripping the build directives in the fixture (and
    // using absolute `path:`) sidesteps that.
    let yaml = write_absolute_path_validated_pipeline_fixture();

    let success_marker = "sink: SUCCESS - validated 10 doubled values";

    // Stable per-test filename; --test-threads=1 means no cross-test
    // contention. Use std::env::temp_dir so macOS `/var/folders/...`
    // works alongside `/tmp` on Linux.
    let drec = std::env::temp_dir().join("dora-record-replay-validated-pipeline.drec");
    let _ = std::fs::remove_file(&drec);

    // ---- Step 1: record ----
    let rec = Command::new(&dora)
        .args([
            "record",
            yaml.to_str().unwrap(),
            "-o",
            drec.to_str().unwrap(),
        ])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("failed to spawn dora record");

    let rec_stdout = String::from_utf8_lossy(&rec.stdout);
    let rec_stderr = String::from_utf8_lossy(&rec.stderr);
    // Clean the fixture now; everything record needed is captured in
    // stdout/stderr and the .drec on disk.
    let _ = std::fs::remove_file(&yaml);
    assert!(
        rec.status.success(),
        "dora record exited non-zero: {:?}\n---- stdout ----\n{rec_stdout}\n---- stderr ----\n{rec_stderr}",
        rec.status
    );
    assert!(
        rec_stdout.contains(success_marker) || rec_stderr.contains(success_marker),
        "recorded run did not reach the SUCCESS marker — the baseline run is broken.\n\
         ---- stdout ----\n{rec_stdout}\n---- stderr ----\n{rec_stderr}"
    );
    assert!(
        drec.exists() && std::fs::metadata(&drec).unwrap().len() > 0,
        "dora record did not produce a non-empty .drec at {drec:?}"
    );

    // ---- Step 2: replay ----
    let rep = Command::new(&dora)
        .args(["replay", drec.to_str().unwrap()])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("failed to spawn dora replay");

    let rep_stdout = String::from_utf8_lossy(&rep.stdout);
    let rep_stderr = String::from_utf8_lossy(&rep.stderr);
    let _ = std::fs::remove_file(&drec);

    assert!(
        rep.status.success(),
        "dora replay exited non-zero: {:?}\n---- stdout ----\n{rep_stdout}\n---- stderr ----\n{rep_stderr}",
        rep.status
    );
    // The contract: replay produces the exact same SUCCESS marker as
    // the recorded run. If the replay dropped/reordered/mutated any
    // value in the recording, sink's `expected = received * 2` check
    // would bail and this marker would never appear.
    assert!(
        rep_stdout.contains(success_marker) || rep_stderr.contains(success_marker),
        "replay did not reproduce the SUCCESS marker — semantic equivalence broken.\n\
         ---- stdout ----\n{rep_stdout}\n---- stderr ----\n{rep_stderr}"
    );
}

/// Run `dora run --stop-after <secs>s` against `yaml_path`, capture combined
/// stdout+stderr, and return `(exit_status, stdout, stderr)`. Pass
/// `use_uv = true` for dataflows with Python nodes so `dora run` provisions
/// an isolated venv for each node.
///
/// Shared helper for the contract tests below (#1630) — they all follow the
/// same run-and-grep pattern but differ in markers and timing.
fn run_dora_capture(
    label: &str,
    yaml_path: &str,
    stop_after_secs: u64,
    use_uv: bool,
) -> (std::process::ExitStatus, String, String) {
    ensure_cli_built();
    let dora = dora_bin();
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let yaml = Path::new(manifest_dir).join(yaml_path);
    assert!(yaml.exists(), "{label}: dataflow YAML missing at {yaml:?}");

    let stop_after = format!("{stop_after_secs}s");
    let mut cmd = Command::new(&dora);
    cmd.args(["run", yaml.to_str().unwrap(), "--stop-after", &stop_after]);
    if use_uv {
        cmd.arg("--uv");
    }
    let output = cmd
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .unwrap_or_else(|e| panic!("{label}: failed to spawn dora run: {e}"));

    let stdout = String::from_utf8_lossy(&output.stdout).into_owned();
    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    (output.status, stdout, stderr)
}

/// Semantic contract test for `service-example` (#1630).
///
/// Contract:
///
/// - Client emits a request on each 500ms (or 700ms) tick, tagged with a
///   unique `request_id` in metadata.
/// - Server receives `{a, b}`, replies with `{sum}` preserving the
///   `request_id`.
/// - Client correlates the response with the in-flight request by
///   `request_id` and asserts `sum == a + b` before printing
///   `"[client-*] response <rid>: a + b = sum"`.
///
/// The existing `smoke_service_example` would pass even if every response
/// was dropped: the dataflow would still exit cleanly on `--stop-after`.
/// This test instead asserts that the CLI log shows at least 3 correlated
/// "response" lines from clients, proving the request_id pipeline carried
/// a reply back to the right client for at least three ticks.
#[test]
fn contract_service_example_correlates_requests_and_responses() {
    let (status, stdout, stderr) = run_dora_capture(
        "contract-service-example",
        "examples/service-example/dataflow.yml",
        10,
        false,
    );
    let combined = format!("{stdout}\n{stderr}");

    // "[service-client-1] response r: a + b = sum" (client_id is the node
    // id from the YAML; two client instances run in parallel in the
    // fixture, so we match the common prefix).
    let matches: Vec<&str> = combined
        .lines()
        .filter(|l| l.contains("] response ") && l.contains(" = "))
        .collect();

    assert!(
        matches.len() >= 3,
        "service-example produced fewer than 3 correlated responses in {}s.\n\
         matched {} line(s): {:?}\n---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}",
        10,
        matches.len(),
        matches
    );
    assert!(
        status.success(),
        "dora run exited non-zero: {status:?}\n---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
}

/// Semantic contract test for `action-example` (#1630).
///
/// Contract:
///
/// - Client sends a goal on each tick with `start_value = 3 + tick_count`.
/// - Server decrements the countdown on every subsequent input event and
///   emits a `feedback` output carrying the current remaining value and
///   the original `goal_id`.
/// - When the countdown reaches 0 the server emits a `result` tagged with
///   `goal_status = SUCCEEDED`. The ordering invariant — feedback before
///   the terminal result — is the core action-pattern contract.
///
/// Liveness smoke would pass for a server that drops every feedback or
/// result. This test asserts the observable log shows at least one full
/// `goal → feedback(s) → result SUCCEEDED` sequence, in order, for the
/// same `goal_id`.
#[test]
fn contract_action_example_feedback_precedes_success_result() {
    let (status, stdout, stderr) = run_dora_capture(
        "contract-action-example",
        "examples/action-example/dataflow.yml",
        20,
        false,
    );
    let combined = format!("{stdout}\n{stderr}");

    // Lines from `dora run` are log-forwarded with a timestamp + stream
    // prefix: e.g. `09:48:50 stdout  action-server:  [server] result <gid>: succeeded`.
    // `GOAL_STATUS_SUCCEEDED` is the lowercase literal `"succeeded"` from
    // libraries/message/src/metadata.rs:128 (not the Rust constant name).
    //
    // Find the first `[server] result <gid>: succeeded` line and confirm a
    // matching feedback line for the same gid appeared earlier in the log.
    let result_marker = "[server] result ";
    let succeeded_suffix = ": succeeded";
    let (result_gid, result_idx) = combined
        .lines()
        .enumerate()
        .find_map(|(idx, line)| {
            let start = line.find(result_marker)? + result_marker.len();
            let end = line[start..].find(succeeded_suffix)? + start;
            Some((line[start..end].to_string(), idx))
        })
        .unwrap_or_else(|| {
            panic!(
                "action-example did not produce a single `[server] result … succeeded` \
                 line in {}s.\n---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}",
                20
            )
        });

    let feedback_needle = format!("[server] feedback {result_gid}:");
    let feedback_idx = combined
        .lines()
        .take(result_idx)
        .position(|l| l.contains(&feedback_needle));
    assert!(
        feedback_idx.is_some(),
        "action-example emitted SUCCEEDED result for goal {result_gid} without preceding feedback.\n\
         ---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
    assert!(
        status.success(),
        "dora run exited non-zero: {status:?}\n---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
}

/// Semantic contract test for `streaming-example` (#1630).
///
/// Contract:
///
/// - `generator` emits a sequence of tokens for each prompt, tagged with
///   `session_id` (constant across the stream) and `seq` (monotonic).
/// - The final token of each stream carries `fin=true`.
/// - `sink` accumulates tokens by `session_id` and, on `fin=true`, logs
///   `"[session <id>] Complete (<N> tokens): <reassembled text>"`.
///
/// The contract is: session_id groups tokens correctly AND `fin=true`
/// triggers reassembly. A regression that dropped the fin flag or mixed
/// sessions would never produce a `Complete` log line — the liveness
/// smoke would not notice.
#[test]
fn contract_streaming_example_reassembles_session_on_fin() {
    let (status, stdout, stderr) = run_dora_capture(
        "contract-streaming-example",
        "examples/streaming-example/dataflow.yml",
        15,
        true,
    );
    let combined = format!("{stdout}\n{stderr}");

    // "[session <id>] Complete (<N> tokens): …"
    let complete_lines: Vec<&str> = combined
        .lines()
        .filter(|l| l.contains("] Complete (") && l.contains(" tokens):"))
        .collect();

    assert!(
        !complete_lines.is_empty(),
        "streaming-example produced zero completed sessions in {}s — \
         the fin=true reassembly path did not fire.\n\
         ---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}",
        15
    );
    assert!(
        status.success(),
        "dora run exited non-zero: {status:?}\n---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
}

// ---------------------------------------------------------------------------
// Queue/timeout regression tests (timing-sensitive, local mode only)
// Guards against previously-fixed bugs in daemon event scheduling.
// ---------------------------------------------------------------------------

#[test]
fn smoke_local_queue_size_and_timeout() {
    run_smoke_test_local(
        "local-queue-size-and-timeout",
        "tests/queue_size_and_timeout_python/dataflow.yaml",
        20,
    );
}

#[test]
fn smoke_local_queue_size_latest_data_python() {
    run_smoke_test_local(
        "local-queue-size-latest-data-python",
        "tests/queue_size_latest_data_python/dataflow.yaml",
        20,
    );
}

fn ensure_queue_latest_rust_built() {
    BUILD_QUEUE_LATEST_RUST.call_once(|| {
        let status = Command::new("cargo")
            .args(["build", "-p", "receive_data"])
            .status()
            .expect("failed to run cargo build for receive_data");
        assert!(status.success(), "failed to build receive_data");
    });
}

#[test]
fn smoke_local_queue_size_latest_data_rust() {
    ensure_queue_latest_rust_built();
    run_smoke_test_local(
        "local-queue-size-latest-data-rust",
        "tests/queue_size_latest_data_rust/dataflow.yaml",
        20,
    );
}
