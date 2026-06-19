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
static BUILD_MAVLINK2_BRIDGE_NODES: Once = Once::new();

fn dora_bin() -> String {
    let manifest = env!("CARGO_MANIFEST_DIR");
    // Honor $CARGO_TARGET_DIR if set — devs and some CI setups redirect
    // builds out of the workspace `target/` dir. The previous fall-back
    // to a bare "dora" on PATH would silently let a stale globally-
    // installed CLI shadow whatever ensure_cli_built() just produced;
    // the test would then "pass" against the wrong binary.
    //
    // Limitations not handled here (would need cargo metadata or
    // --message-format=json parsing in ensure_cli_built):
    //   - `.cargo/config.toml` `[build] target-dir`
    //   - target-triple subdirs from `CARGO_BUILD_TARGET` / cargo
    //     config (`target/<triple>/debug/...`)
    // Neither is used in this project today; the panic below points at
    // the cause if a future config hits them.
    let target_root = std::env::var("CARGO_TARGET_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| Path::new(manifest).join("target"));
    // `EXE_SUFFIX` is `.exe` on Windows, empty elsewhere. Without this,
    // dora_bin() panics on Windows where the artifact is `dora.exe`,
    // breaking every smoke test on that platform.
    let exe_name = format!("dora{}", std::env::consts::EXE_SUFFIX);
    let candidate = target_root.join("debug").join(&exe_name);
    if candidate.exists() {
        return candidate.to_string_lossy().to_string();
    }
    panic!(
        "dora binary not found at {} after ensure_cli_built(); \
         if you use .cargo/config.toml or CARGO_BUILD_TARGET, ensure \
         CARGO_TARGET_DIR points at the resolved artifact directory",
        candidate.display()
    );
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
// Some examples also self-terminate after completing their advertised work.
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
fn smoke_python_zero_copy_send_example() {
    run_smoke_test(
        "python-zero-copy-send",
        "examples/python-zero-copy-send/dataflow.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_python_zero_copy_send_contract() {
    // Buffer-protocol contract tests for `node.send_output_raw`.
    // The contract node exits non-zero on any failure; smoke harness
    // surfaces the exit code as a test failure.
    run_smoke_test(
        "python-zero-copy-send-contract",
        "examples/python-zero-copy-send/contract_dataflow.yml",
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
fn contract_local_python_dataflow_sender_receives_stop_event() {
    let (status, stdout, stderr) = run_dora_capture(
        "local-python-dataflow-stop-contract",
        "examples/python-dataflow/dataflow.yml",
        // 5s, not 1s: the `--stop-after` timer is wall-clock from dataflow
        // start and does not wait for nodes to become ready. On a loaded CI
        // runner, `uv` + Python startup can consume the whole 1s budget before
        // the sender's `try_recv()` loop runs, so it gets torn down without
        // ever observing STOP and the contract assertion below flakes. 5s
        // leaves the sender ample loop time after startup while staying well
        // under its ~10s self-completion bound, so STOP is still `--stop-after`
        // driven.
        5,
        true,
    );
    let combined = format!("{stdout}\n{stderr}");
    assert!(
        status.success(),
        "dora run exited non-zero: {status:?}\n---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
    assert!(
        combined.contains("Sender stopping after receiving STOP"),
        "sender did not log the STOP path; it may have ignored Dora shutdown and exited by timing luck.\n\
         ---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
}

#[test]
fn smoke_local_python_zero_copy_send_contract() {
    // Local-mode variant of the buffer-protocol contract test.
    // The contract node exits when finished, so 30s is just an upper bound.
    run_smoke_test_local(
        "local-python-zero-copy-send-contract",
        "examples/python-zero-copy-send/contract_dataflow.yml",
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
// Memory-pool transport (CPU-only path, GPU-less CI runner safe)
//
// Exercises register/write/read/free lifecycle for the pinned shared-memory
// transport added in PR #2168. cpu2cpu.yml is the CPU-only scenario whose
// header comment explicitly notes it was designed for GPU-less CI runners.
// ---------------------------------------------------------------------------

#[test]
fn smoke_memory_pool_cpu2cpu() {
    run_smoke_test(
        "memory-pool-cpu2cpu",
        "examples/memory-pool/cpu2cpu.yml",
        Duration::from_secs(120),
    );
}

#[test]
fn smoke_local_memory_pool_cpu2cpu() {
    run_smoke_test_local(
        "local-memory-pool-cpu2cpu",
        "examples/memory-pool/cpu2cpu.yml",
        120,
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

/// Semantic cross-language contracts (#1658).
///
/// The `smoke_cross_language_*` tests above only check for non-Failed
/// state via `dora list` — they'd still pass if every payload arrived
/// with a wrong value as long as the receivers didn't `bail!`/`sys.exit`.
/// The receivers (`cross-language/python_receiver.py`,
/// `cross-language/rust-receiver/src/main.rs`) already validate each
/// payload against the sender's deterministic sequence `[0, 10, …, 90]`
/// and print a specific `SUCCESS - validated N messages` marker on
/// clean completion; the smoke lane just never looks.
///
/// These two tests run `dora run --stop-after`, capture combined
/// stdout+stderr, and assert the marker contains exactly the number
/// the senders emit (10). A regression in the Rust ↔ Python Arrow
/// payload path (for example: byte-order drift, length mismatch,
/// type-widening, or a message being dropped on the wire) fails
/// either the receiver's in-process validation (no marker) or the
/// count check here.
#[test]
fn contract_cross_language_rust_to_python_delivers_all_ten_values() {
    ensure_cross_language_nodes_built();
    let (status, stdout, stderr) = run_dora_capture(
        "contract-cross-language-rust-to-python",
        "examples/cross-language/rust-to-python.yml",
        15,
        true, // Python receiver needs uv-provisioned venv
    );
    let combined = format!("{stdout}\n{stderr}");
    assert!(
        combined.contains("python-receiver: SUCCESS - validated 10 messages"),
        "rust → python did not deliver exactly 10 validated messages.\n\
         ---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
    assert!(
        status.success(),
        "dora run exited non-zero: {status:?}\n---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
}

#[test]
fn contract_cross_language_python_to_rust_delivers_all_ten_values() {
    ensure_cross_language_nodes_built();
    let (status, stdout, stderr) = run_dora_capture(
        "contract-cross-language-python-to-rust",
        "examples/cross-language/python-to-rust.yml",
        15,
        true, // Python sender needs uv-provisioned venv
    );
    let combined = format!("{stdout}\n{stderr}");
    assert!(
        combined.contains("rust-receiver: SUCCESS - validated 10 messages"),
        "python → rust did not deliver exactly 10 validated messages.\n\
         ---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
    );
    assert!(
        status.success(),
        "dora run exited non-zero: {status:?}\n---- stdout ----\n{stdout}\n---- stderr ----\n{stderr}"
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

    // On a loaded CI runner the record-node spawn can be slow enough that
    // the body read times out before the node sends its first message,
    // surfacing as "TCP read body timed out" — an infrastructure stall, not a
    // semantic break in the recording. Retry the whole record+replay cycle a
    // bounded number of times on *that signature in the record step only*; the
    // replay step treats TCP timeouts as semantic failures (the recording is
    // already known-good, so a missing message is a real regression). A missing
    // SUCCESS marker on an otherwise-clean run fails immediately.
    const MAX_ATTEMPTS: u32 = 3;
    for attempt in 1..=MAX_ATTEMPTS {
        match record_replay_once(&dora) {
            Ok(()) => return,
            Err(RecordReplayFailure::InfraTimeout(detail)) if attempt < MAX_ATTEMPTS => {
                eprintln!(
                    "record/replay attempt {attempt}/{MAX_ATTEMPTS} hit a TCP read timeout \
                     (infra stall, not a semantic break); retrying.\n{detail}"
                );
            }
            Err(failure) => panic!("{failure}"),
        }
    }
}

/// Why a single record+replay cycle failed.
enum RecordReplayFailure {
    /// A TCP body timeout during the **record** step — an infrastructure stall
    /// under CI load (slow node spawn). Safe to retry.
    InfraTimeout(String),
    /// A real break: non-zero exit without a record-step TCP timeout, a missing
    /// SUCCESS marker, a missing/empty recording, or *any* failure in the replay
    /// step (the recording is known-good, so a TCP timeout there is semantic).
    /// Never retried.
    Semantic(String),
}

impl std::fmt::Display for RecordReplayFailure {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RecordReplayFailure::InfraTimeout(d) | RecordReplayFailure::Semantic(d) => {
                f.write_str(d)
            }
        }
    }
}

/// Classify a failed record/replay step.
///
/// `in_record_step` gates whether a TCP-timeout signature is treated as
/// retryable: during the **record** step a slow node spawn on a loaded CI runner
/// can trigger the body timeout before the node sends its first message, and
/// that is genuine infra noise. During the **replay** step the recording is
/// already known-good, so a TCP timeout means a message never arrived and is a
/// real replay regression — not safe to retry.
fn classify_record_replay(
    combined: &str,
    detail: String,
    in_record_step: bool,
) -> RecordReplayFailure {
    if in_record_step
        && (combined.contains("TCP read header timed out")
            || combined.contains("TCP read body timed out"))
    {
        RecordReplayFailure::InfraTimeout(detail)
    } else {
        RecordReplayFailure::Semantic(detail)
    }
}

/// One record+replay cycle: record the validated pipeline, then replay the
/// recording, asserting both reach the SUCCESS marker. Returns `Err` (rather
/// than panicking) so the caller can retry on an infra timeout.
fn record_replay_once(dora: &str) -> Result<(), RecordReplayFailure> {
    let success_marker = "sink: SUCCESS - validated 10 doubled values";

    // Use the absolute-path fixture instead of examples/validated-pipeline/dataflow.yml:
    // `dora record` writes its modified descriptor into a tempfile in
    // /tmp, so any `build: cargo build -p …` directive carried over
    // from the original YAML runs cargo from /tmp — where there's no
    // Cargo.toml. Stripping the build directives in the fixture (and
    // using absolute `path:`) sidesteps that.
    let yaml = write_absolute_path_validated_pipeline_fixture();

    // Stable per-test filename; --test-threads=1 means no cross-test
    // contention. Use std::env::temp_dir so macOS `/var/folders/...`
    // works alongside `/tmp` on Linux.
    let drec = std::env::temp_dir().join("dora-record-replay-validated-pipeline.drec");
    let _ = std::fs::remove_file(&drec);

    // ---- Step 1: record ----
    let rec = Command::new(dora)
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
    if !rec.status.success() {
        let _ = std::fs::remove_file(&drec);
        return Err(classify_record_replay(
            &format!("{rec_stdout}{rec_stderr}"),
            format!(
                "dora record exited non-zero: {:?}\n---- stdout ----\n{rec_stdout}\n---- stderr ----\n{rec_stderr}",
                rec.status
            ),
            true, // record step: TCP timeout may be infra noise
        ));
    }
    if !(rec_stdout.contains(success_marker) || rec_stderr.contains(success_marker)) {
        let _ = std::fs::remove_file(&drec);
        return Err(classify_record_replay(
            &format!("{rec_stdout}{rec_stderr}"),
            format!(
                "recorded run did not reach the SUCCESS marker — the baseline run is broken.\n\
                 ---- stdout ----\n{rec_stdout}\n---- stderr ----\n{rec_stderr}"
            ),
            true, // record step: TCP timeout may be infra noise
        ));
    }
    if !(drec.exists()
        && std::fs::metadata(&drec)
            .map(|m| m.len() > 0)
            .unwrap_or(false))
    {
        return Err(RecordReplayFailure::Semantic(format!(
            "dora record did not produce a non-empty .drec at {drec:?}"
        )));
    }

    // ---- Step 2: replay ----
    let rep = Command::new(dora)
        .args(["replay", drec.to_str().unwrap()])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("failed to spawn dora replay");

    let rep_stdout = String::from_utf8_lossy(&rep.stdout);
    let rep_stderr = String::from_utf8_lossy(&rep.stderr);
    let _ = std::fs::remove_file(&drec);

    if !rep.status.success() {
        return Err(classify_record_replay(
            &format!("{rep_stdout}{rep_stderr}"),
            format!(
                "dora replay exited non-zero: {:?}\n---- stdout ----\n{rep_stdout}\n---- stderr ----\n{rep_stderr}",
                rep.status
            ),
            false, // replay step: the recording is known-good; a TCP timeout is a real regression
        ));
    }
    // The contract: replay produces the exact same SUCCESS marker as
    // the recorded run. If the replay dropped/reordered/mutated any
    // value in the recording, sink's `expected = received * 2` check
    // would bail and this marker would never appear.
    if !(rep_stdout.contains(success_marker) || rep_stderr.contains(success_marker)) {
        return Err(classify_record_replay(
            &format!("{rep_stdout}{rep_stderr}"),
            format!(
                "replay did not reproduce the SUCCESS marker — semantic equivalence broken.\n\
                 ---- stdout ----\n{rep_stdout}\n---- stderr ----\n{rep_stderr}"
            ),
            false, // replay step: missing SUCCESS means a dropped/mutated message, not infra noise
        ));
    }
    Ok(())
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

fn ensure_mavlink2_bridge_nodes_built() {
    BUILD_MAVLINK2_BRIDGE_NODES.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "dora-mavlink2-bridge-node",
                "-p",
                "mavlink2-bridge-example-mavlink-sim",
                "-p",
                "mavlink2-bridge-example-heartbeat-emitter",
                "-p",
                "mavlink2-bridge-example-telemetry-printer-rust",
            ])
            .status()
            .expect("failed to run cargo build for mavlink2-bridge example");
        assert!(status.success(), "failed to build mavlink2-bridge nodes");
    });
}

// ---------------------------------------------------------------------------
// MAVLink 2 bridge example (#1786)
//
// Three variants share the same bridge + UDP simulator + Rust HEARTBEAT
// emitter; only the consumer that reads `bridge/heartbeat` differs:
//
//   * dataflow-rust.yml   -- pure Rust (no Python toolchain required)
//   * dataflow-python.yml -- Python printer via `--uv`
//   * dataflow-cxx.yml    -- C++ printer; built+run via the
//                            `mavlink2-bridge-cxx` cargo example
//                            (mirrors `cxx-arrow-dataflow`). NOT in
//                            the smoke harness — see audit table.
//
// UDP avoids `TIME_WAIT` between successive smoke runs and keeps these
// tests CI-friendly without a SITL/MAVProxy dependency.
// ---------------------------------------------------------------------------

#[test]
fn smoke_mavlink2_bridge_rust() {
    ensure_mavlink2_bridge_nodes_built();
    run_smoke_test(
        "mavlink2-bridge-rust",
        "examples/mavlink2-bridge/dataflow-rust.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_local_mavlink2_bridge_rust() {
    ensure_mavlink2_bridge_nodes_built();
    run_smoke_test_local(
        "local-mavlink2-bridge-rust",
        "examples/mavlink2-bridge/dataflow-rust.yml",
        10,
    );
}

#[test]
fn smoke_mavlink2_bridge_python() {
    ensure_mavlink2_bridge_nodes_built();
    run_smoke_test(
        "mavlink2-bridge-python",
        "examples/mavlink2-bridge/dataflow-python.yml",
        Duration::from_secs(30),
    );
}

#[test]
fn smoke_local_mavlink2_bridge_python() {
    ensure_mavlink2_bridge_nodes_built();
    run_smoke_test_local(
        "local-mavlink2-bridge-python",
        "examples/mavlink2-bridge/dataflow-python.yml",
        10,
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

// ---------------------------------------------------------------------------
// Daemon `--rt` smoke (issue #1701)
// ---------------------------------------------------------------------------
//
// Asserts the RT setup code path actually emits its diagnostic messages.
// We deliberately do NOT require CAP_IPC_LOCK / CAP_SYS_NICE — unprivileged
// CI runners hit the documented fallback branch, which is exactly the
// "useful error vs. silent fallback" guarantee #1701 calls out as untested.
//
// Linux exercises both mlockall and SCHED_FIFO branches; macOS exercises
// the mlockall + "SCHED_FIFO not available" branches. Windows is excluded
// via `#[cfg(unix)]` because the assertions below only cover unix RT
// branches — the Windows fallback at `binaries/cli/src/command/daemon.rs`
// (`#[cfg(not(unix))]` arm) is not exercised here.

#[test]
#[cfg(unix)]
fn smoke_daemon_rt_emits_setup_messages() {
    ensure_cli_built();
    let dora = dora_bin();

    // Stale coordinator on 6013 would let the daemon connect and run
    // indefinitely — we only need ~1s to capture the RT prints.
    cleanup_stale(&dora);

    let mut child = Command::new(&dora)
        .args(["daemon", "--rt"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn dora daemon --rt");

    // RT setup runs synchronously at startup, before the daemon tries to
    // dial the coordinator. ~1.5s is enough for the eprintln!s to land on
    // the captured stderr pipe.
    std::thread::sleep(Duration::from_millis(1500));

    let _ = child.kill();
    let output = child
        .wait_with_output()
        .expect("daemon wait_with_output failed");

    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        stderr.contains("RT: mlockall"),
        "expected 'RT: mlockall' line on stderr (success or failure variant).\n\
         stderr was:\n{stderr}"
    );

    #[cfg(target_os = "linux")]
    assert!(
        stderr.contains("RT: SCHED_FIFO priority 50 enabled")
            || stderr.contains("RT: sched_setscheduler failed:"),
        "expected SCHED_FIFO success-or-failure line on stderr.\n\
         stderr was:\n{stderr}"
    );

    #[cfg(not(target_os = "linux"))]
    assert!(
        stderr.contains("RT: SCHED_FIFO not available on this platform"),
        "expected non-Linux SCHED_FIFO fallback line on stderr.\n\
         stderr was:\n{stderr}"
    );
}

/// `--quiet` must NOT suppress RT setup diagnostics. The fix in #1701
/// deliberately uses `eprintln!` so a failed mlockall/SCHED_FIFO promotion
/// is always visible — silencing operational warnings via the existing log
/// filter would be unsafe.
#[test]
#[cfg(unix)]
fn smoke_daemon_rt_quiet_still_emits_setup_messages() {
    ensure_cli_built();
    let dora = dora_bin();

    cleanup_stale(&dora);

    let mut child = Command::new(&dora)
        .args(["daemon", "--rt", "--quiet"])
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn dora daemon --rt --quiet");

    std::thread::sleep(Duration::from_millis(1500));

    let _ = child.kill();
    let output = child
        .wait_with_output()
        .expect("daemon wait_with_output failed");

    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        stderr.contains("RT: mlockall"),
        "expected 'RT: mlockall' line on stderr under --quiet.\n\
         stderr was:\n{stderr}"
    );

    #[cfg(target_os = "linux")]
    assert!(
        stderr.contains("RT: SCHED_FIFO priority 50 enabled")
            || stderr.contains("RT: sched_setscheduler failed:"),
        "expected SCHED_FIFO line on stderr under --quiet.\n\
         stderr was:\n{stderr}"
    );

    #[cfg(not(target_os = "linux"))]
    assert!(
        stderr.contains("RT: SCHED_FIFO not available on this platform"),
        "expected non-Linux SCHED_FIFO fallback on stderr under --quiet.\n\
         stderr was:\n{stderr}"
    );
}

// ---------------------------------------------------------------------------
// Shell-node gate (issue #1702)
// ---------------------------------------------------------------------------
//
// `dora run --allow-shell-nodes` toggles a security gate enforced at
// `binaries/daemon/src/spawn/command.rs:24`. Both branches of that `if`
// matter: without the flag, dora MUST refuse to spawn shell nodes.
// The negative test is therefore load-bearing security coverage —
// a regression that silently allowed shell execution would bypass the
// security model. Linux + macOS only (`sh -c` form); Windows uses
// `cmd /C` and would need a separate fixture.

fn unique_temp_path(stem: &str) -> std::path::PathBuf {
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    std::env::temp_dir().join(format!(
        "dora-shell-gate-{stem}-{}-{nanos}",
        std::process::id()
    ))
}

#[cfg(unix)]
fn write_shell_dataflow(yaml_path: &std::path::Path, marker: &std::path::Path) {
    // Wrap the marker path in single quotes so spaces or most shell
    // metacharacters in `$TMPDIR` cannot break out of the `echo`. This
    // does NOT protect against a literal single quote in $TMPDIR — that
    // would corrupt the shell args and the test would fail loudly
    // rather than silently execute the wrong command. On any sane
    // CI/dev system $TMPDIR is a well-formed path; the residual risk
    // is acceptable for a test fixture in an environment we control.
    let yaml = format!(
        "nodes:\n  - id: shell-test\n    path: shell\n    args: \"echo hello > '{}'\"\n",
        marker.display()
    );
    std::fs::write(yaml_path, yaml).expect("failed to write shell dataflow YAML");
}

/// Guarantee the marker path is clean before the test runs. A silent
/// `remove_file` failure (permissions, symlink to unwritable target)
/// could otherwise let the positive test pass against a stale marker
/// without proving this run wrote it.
fn ensure_marker_absent(marker: &std::path::Path) {
    let _ = std::fs::remove_file(marker);
    assert!(
        !marker.exists(),
        "marker path {marker:?} pre-existed and could not be removed; \
         a stale file would make the positive test pass spuriously"
    );
}

/// Defense in depth, narrow scope: serializes ONLY these two shell-gate
/// tests against each other. The file is documented to run with
/// `--test-threads=1`, which is the canonical fix; the mutex is a
/// last-resort guard for accidental in-process parallel invocations.
/// It does NOT protect against multi-process runners like `cargo
/// nextest run` or sharded CI — those would need a file lock
/// (`fs2::FileExt::lock_exclusive` or similar).
static SHELL_GATE_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[test]
#[cfg(unix)]
fn smoke_shell_node_allowed_with_flag() {
    let _guard = SHELL_GATE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    ensure_cli_built();
    let dora = dora_bin();

    let yaml = unique_temp_path("allowed-yaml");
    let marker = unique_temp_path("allowed-marker");
    ensure_marker_absent(&marker);
    write_shell_dataflow(&yaml, &marker);

    let output = Command::new(&dora)
        .args([
            "run",
            yaml.to_str().unwrap(),
            "--allow-shell-nodes",
            "--stop-after",
            "10s",
        ])
        // Don't let an inherited DORA_ALLOW_SHELL_NODES=true bias the
        // result — the CLI flag must be the sole source of truth.
        .env_remove("DORA_ALLOW_SHELL_NODES")
        // Capture both streams: the audit-trail warning is emitted via
        // the daemon's tracing subscriber, which routes to stdout in
        // `dora run` (single-process mode), while CLI-level errors go
        // to stderr. We need both to verify the gate's side effects.
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .expect("failed to run dora run --allow-shell-nodes");

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    // Assert dora exited cleanly BEFORE checking the marker. Otherwise a
    // regression where the shell node runs successfully but a subsequent
    // lifecycle/result step fails (e.g. dataflow result handling returns
    // non-zero) would still produce the marker file and slip past the
    // test (review feedback on PR #1898).
    assert!(
        output.status.success(),
        "dora run --allow-shell-nodes exited non-zero (status={:?}); \
         marker file alone is not enough.\nstdout:\n{stdout}\nstderr:\n{stderr}",
        output.status.code()
    );
    // The audit-trail warning at binaries/daemon/src/spawn/command.rs:34
    // is the operator's only signal that shell execution is happening.
    // If the gate works but this warning is silenced, dora becomes
    // quietly unsafe — the marker check alone wouldn't catch it.
    assert!(
        stdout.contains("DORA_ALLOW_SHELL_NODES is set:")
            || stderr.contains("DORA_ALLOW_SHELL_NODES is set:"),
        "audit-trail warning missing — operators would lose the only \
         signal that shell execution is happening.\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
    assert!(
        marker.exists(),
        "shell command did not produce marker file at {marker:?}; \
         dora exit={:?}\nstdout:\n{stdout}\nstderr:\n{stderr}",
        output.status.code()
    );
    let body = std::fs::read_to_string(&marker).expect("read marker");
    assert!(
        body.contains("hello"),
        "marker contents unexpected: {body:?}\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );

    let _ = std::fs::remove_file(&marker);
    let _ = std::fs::remove_file(&yaml);
}

#[test]
#[cfg(unix)]
fn smoke_shell_node_blocked_without_flag() {
    let _guard = SHELL_GATE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    ensure_cli_built();
    let dora = dora_bin();

    let yaml = unique_temp_path("blocked-yaml");
    let marker = unique_temp_path("blocked-marker");
    ensure_marker_absent(&marker);
    write_shell_dataflow(&yaml, &marker);

    let output = Command::new(&dora)
        .args(["run", yaml.to_str().unwrap(), "--stop-after", "10s"])
        // Belt-and-suspenders: explicitly strip the env var from the
        // child. Without this, a parent shell that exported the flag
        // (or a leftover from a prior test) would silently let the
        // gate pass — and we'd never know.
        .env_remove("DORA_ALLOW_SHELL_NODES")
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .output()
        .expect("failed to run dora run (gate-blocked)");

    let stderr = String::from_utf8_lossy(&output.stderr);

    assert!(
        !output.status.success(),
        "dora run unexpectedly SUCCEEDED without --allow-shell-nodes — \
         the security gate failed.\nstderr:\n{stderr}"
    );
    assert!(
        stderr.contains("Shell nodes are disabled"),
        "error message missing the documented gate string \
         'Shell nodes are disabled'.\nstderr:\n{stderr}"
    );
    assert!(
        !marker.exists(),
        "shell command produced output at {marker:?} despite the gate — \
         the security gate failed.\nstderr:\n{stderr}"
    );

    let _ = std::fs::remove_file(&yaml);
}

// ---------------------------------------------------------------------------
// Examples under `examples/` that do NOT have a corresponding `smoke_*` or
// `contract_*` test in this file. Some are blocked (filed issue or external
// dep); others are intentionally covered by a DIFFERENT CI job. Keep this
// table in sync with `scripts/smoke-all.sh` and the CI workflows it cites,
// so the Rust suite, the shell helper, and CI all agree on what's covered
// where and why.
//
// Audit method: diff `ls examples/` against the `smoke_*`/`contract_*`
// function list in this file. Every example without a match here must have
// a row below — either as a blocker or as "covered elsewhere" with a link
// to the specific CI line that covers it.
//
// If you land a fix for a blocker below, write the corresponding smoke test
// here and remove the row. Consumers of this file: the QA policy in
// `docs/agentic-qa-policy.md` and the capability matrix in
// `docs/testing-capabilities.md`.
//
// | Example                   | Where it's tested / blocker                          | Tracking |
// |---------------------------|------------------------------------------------------|----------|
// | cuda-benchmark            | blocker: needs NVIDIA CUDA toolkit                   | —        |
// | dynamic-add-remove        | blocker: `dora node add` times out +                 | #1682    |
// |                           | corrupts dataflow state                              |          |
// | dynamic-agent-tools       | blocker: same as dynamic-add-remove                  | #1682    |
// | python-parquet-recorder   | blocker: no test written; low-priority               | —        |
// | python-yolo-detection     | blocker: needs YOLO model weights                    | —        |
// | ros2-bridge               | blocker: needs ROS2 runtime                          | —        |
// | ros2-comparison           | blocker: needs ROS2 runtime                          | —        |
// | c-dataflow                | covered: `cli` job (3 OS), ci.yml CLI tests          | covered  |
// | c++-dataflow              | covered: `cli` job (3 OS), ci.yml CLI tests          | covered  |
// | c++-arrow-dataflow        | covered: `cli` job (3 OS), ci.yml CLI tests          | covered  |
// | cmake-dataflow            | covered: `cli` job (3 OS), ci.yml CLI tests          | covered  |
// | cpu-affinity-probe        | covered: nightly `cpu-affinity-smoke`                | covered  |
// |                           | (.github/workflows/nightly.yml:535)                  |          |
// | error-propagation         | covered: `cli` job .github/workflows/ci.yml:327      | covered  |
// | multiple-daemons          | covered: `cargo check --all` (workspace members)     | covered  |
// | python-dataflow-builder   | covered: builder contract                            | covered  |
// |                           | .github/workflows/ci.yml:433 + docs/                 |          |
// |                           | testing-capabilities.md                              |          |
// | python-operator-dataflow  | covered: `cli` job .github/workflows/ci.yml:393      | covered  |
// | rust-dataflow-git         | covered: `examples` job (3 OS)                       | covered  |
// | mavlink2-bridge-cxx       | covered: `examples` job via                          | covered  |
// |                           | `[[example]] mavlink2-bridge-cxx` (cargo run         |          |
// |                           | --example), same shape as `cxx-arrow-dataflow`       |          |
// | mavlink2-bridge-sitl-     | blocker: needs ArduPilot SITL                        | —        |
// |   mission                 | (Ubuntu / macOS only, local-only by design;          |          |
// |                           | see examples/mavlink2-bridge-sitl-mission/README)    |          |
//
// "Covered" rows are listed so future refactors don't assume the examples
// are entirely unexercised — they run in other CI jobs, just not this file.
