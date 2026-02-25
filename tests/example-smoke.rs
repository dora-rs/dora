//! Smoke tests for examples using the WS control plane (adora up + adora start).
//!
//! Each test starts a coordinator+daemon via `adora up`, runs a dataflow,
//! waits for it to complete or times out, then cleans up.
//!
//! Run with: `cargo test --test example-smoke -- --test-threads=1`

use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::Once;
use std::time::Duration;

static BUILD_NODES: Once = Once::new();
static BUILD_CLI: Once = Once::new();

fn adora_bin() -> String {
    let manifest = env!("CARGO_MANIFEST_DIR");
    let target_dir = Path::new(manifest).join("target/debug/adora");
    if target_dir.exists() {
        return target_dir.to_string_lossy().to_string();
    }
    "adora".to_string()
}

fn ensure_cli_built() {
    BUILD_CLI.call_once(|| {
        let status = Command::new("cargo")
            .args(["build", "-p", "adora-cli"])
            .status()
            .expect("failed to run cargo build for CLI");
        assert!(status.success(), "failed to build adora CLI");
    });
}

fn ensure_rust_nodes_built() {
    BUILD_NODES.call_once(|| {
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

/// Ensure no leftover coordinator/daemon from a previous test or manual run.
fn cleanup_stale(adora: &str) {
    let _ = Command::new(adora)
        .arg("destroy")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    std::thread::sleep(Duration::from_millis(500));
}

/// Run an example dataflow through the full WS control plane lifecycle.
fn run_smoke_test(name: &str, yaml_path: &str, timeout: Duration) {
    ensure_cli_built();

    let adora = adora_bin();
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let full_yaml = Path::new(manifest_dir).join(yaml_path);
    assert!(
        full_yaml.exists(),
        "{name}: dataflow YAML not found at {full_yaml:?}"
    );

    cleanup_stale(&adora);

    // `adora up` starts coordinator + daemon and returns when both are ready.
    // Use Stdio::null() for all streams to prevent child processes from
    // keeping inherited pipe fds open.
    let up_status = Command::new(&adora)
        .arg("up")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .unwrap_or_else(|e| panic!("{name}: failed to run adora up: {e}"));

    assert!(up_status.success(), "{name}: adora up failed");

    // Start dataflow (detach so we get control back immediately)
    let start_status = Command::new(&adora)
        .args(["start", full_yaml.to_str().unwrap(), "--detach"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .unwrap_or_else(|e| panic!("{name}: failed to start dataflow: {e}"));

    assert!(start_status.success(), "{name}: adora start failed");

    // Wait for completion or timeout
    let start_time = std::time::Instant::now();
    while start_time.elapsed() < timeout {
        std::thread::sleep(Duration::from_secs(2));

        // Check if any dataflows are still running
        let list_result = Command::new(&adora)
            .args(["list", "--json"])
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .output()
            .ok();
        if let Some(output) = list_result {
            let stdout = String::from_utf8_lossy(&output.stdout);
            if stdout.trim().is_empty() || !stdout.contains("Running") {
                break;
            }
        }
    }

    // Clean up: stop all dataflows and destroy coordinator+daemon
    let _ = Command::new(&adora)
        .args(["stop", "--all"])
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
    std::thread::sleep(Duration::from_millis(500));
    let _ = Command::new(&adora)
        .arg("destroy")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}

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
fn smoke_rust_dataflow_socket() {
    ensure_rust_nodes_built();
    run_smoke_test(
        "rust-dataflow-socket",
        "examples/rust-dataflow/dataflow_socket.yml",
        Duration::from_secs(30),
    );
}
