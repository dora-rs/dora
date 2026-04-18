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
