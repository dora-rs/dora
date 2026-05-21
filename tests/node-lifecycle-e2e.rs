//! End-to-end coverage of the full `dora node` subcommand matrix
//! (`list`, `info`, `add`, `remove`, `connect`, `disconnect`, `stop`,
//! `restart`) — issue #1703.
//!
//! Per the issue's "expected coverage" shape: one long-running dataflow
//! per language, then exercise every subcommand against it and assert
//! via `dora node list` / observable side effects, then tear down.
//!
//! This file covers **Python** via `examples/dynamic-add-remove`. Rust
//! and C++ variants will follow as separate tests / fixtures once their
//! dynamic-add candidates land.
//!
//! Run with `cargo test --test node-lifecycle-e2e -- --test-threads=1`.
//! Each test spawns a coordinator + daemon on the hard-coded port 6013
//! (`binaries/cli/src/command/run.rs:226` for `dora run`,
//! `dora_core::topics::DORA_COORDINATOR_PORT_WS_DEFAULT` for `dora up`),
//! so the in-file `LIFECYCLE_LOCK` serializes the tests within this
//! binary and `--test-threads=1` is required for safety against other
//! integration-test binaries that also use the default port.

use std::path::Path;
use std::process::{Command, Stdio};
use std::sync::{Mutex, Once};
use std::time::Duration;

static BUILD_CLI: Once = Once::new();
/// Defense in depth: `dora node *` operates against the coordinator on
/// `DORA_COORDINATOR_PORT_WS_DEFAULT` (port 6013). Running these tests
/// in parallel with each other (or any other test that spawns its own
/// coordinator) corrupts shared state. The file is documented to run
/// with `--test-threads=1`, but if someone forgets, the lock at least
/// keeps the tests in this file from racing each other.
static LIFECYCLE_LOCK: Mutex<()> = Mutex::new(());

fn dora_bin() -> String {
    let manifest = env!("CARGO_MANIFEST_DIR");
    // Honor $CARGO_TARGET_DIR + EXE_SUFFIX (Windows .exe). Without this
    // a stale globally-installed `dora` on PATH would silently shadow
    // the freshly-built CLI, see #1701 / example-smoke.rs.
    let target_root = std::env::var("CARGO_TARGET_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|_| Path::new(manifest).join("target"));
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

/// Tear down any leftover coordinator/daemon so tests don't pile up
/// state from a prior crash. Safe to call when nothing is running —
/// the commands error silently.
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
    // Wait for port 6013 to clear (TCP TIME_WAIT).
    std::thread::sleep(Duration::from_secs(1));
}

/// Capture both streams as UTF-8 strings.
fn run_capture(cmd: &mut Command, label: &str) -> (bool, String, String) {
    let output = cmd
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .output()
        .unwrap_or_else(|e| panic!("failed to run {label}: {e}"));
    let stdout = String::from_utf8_lossy(&output.stdout).into_owned();
    let stderr = String::from_utf8_lossy(&output.stderr).into_owned();
    (output.status.success(), stdout, stderr)
}

/// Parse the line-delimited JSON output of `dora node list --format json`
/// into a map of node_id → (status, pid, restarts).
fn parse_node_list(stdout: &str) -> std::collections::BTreeMap<String, (String, String, String)> {
    let mut out = std::collections::BTreeMap::new();
    for line in stdout.lines() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let v: serde_json::Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(_) => continue,
        };
        let node = v
            .get("node")
            .and_then(|x| x.as_str())
            .unwrap_or("")
            .to_string();
        let status = v
            .get("status")
            .and_then(|x| x.as_str())
            .unwrap_or("")
            .to_string();
        let pid = v
            .get("pid")
            .and_then(|x| x.as_str())
            .unwrap_or("")
            .to_string();
        let restarts = v
            .get("restarts")
            .and_then(|x| x.as_str())
            .unwrap_or("")
            .to_string();
        if !node.is_empty() {
            out.insert(node, (status, pid, restarts));
        }
    }
    out
}

/// Poll `dora node list` until `predicate` returns true or `timeout`
/// elapses. Used to wait for slow lifecycle transitions (`stop` waits
/// up to `DEFAULT_STOP_GRACE` = 10s, `restart` waits up to
/// `DEFAULT_RESTART_GRACE` = 5s before the actual SIGTERM lands).
fn wait_for_list<F>(dora: &str, dataflow: &str, timeout: Duration, mut predicate: F) -> String
where
    F: FnMut(&std::collections::BTreeMap<String, (String, String, String)>) -> bool,
{
    let deadline = std::time::Instant::now() + timeout;
    loop {
        let (_, stdout, _) = run_capture(
            Command::new(dora).args(["node", "list", "--dataflow", dataflow, "--format", "json"]),
            "dora node list",
        );
        let parsed = parse_node_list(&stdout);
        if predicate(&parsed) {
            return stdout;
        }
        if std::time::Instant::now() >= deadline {
            return stdout;
        }
        std::thread::sleep(Duration::from_millis(500));
    }
}

#[test]
#[cfg(unix)]
fn lifecycle_python_dynamic_add_remove() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    ensure_cli_built();
    let dora = dora_bin();
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let dataflow = Path::new(manifest_dir).join("examples/dynamic-add-remove/dataflow.yml");
    let filter_yml = Path::new(manifest_dir).join("examples/dynamic-add-remove/filter-node.yml");
    let name = "pylc";

    cleanup_stale(&dora);

    // `dora up` starts coordinator + daemon on port 6013.
    let (ok, _, stderr) = run_capture(Command::new(&dora).arg("up"), "dora up");
    assert!(ok, "dora up failed.\nstderr:\n{stderr}");

    // `dora build` ensures the Python venv is provisioned before start.
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args(["build", dataflow.to_str().unwrap(), "--uv"]),
        "dora build",
    );
    assert!(ok, "dora build failed.\nstderr:\n{stderr}");

    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "start",
            dataflow.to_str().unwrap(),
            "--detach",
            "--name",
            name,
            "--uv",
        ]),
        "dora start",
    );
    assert!(ok, "dora start failed.\nstderr:\n{stderr}");

    // Make sure both base nodes are Running before exercising the matrix.
    let list_out = wait_for_list(&dora, name, Duration::from_secs(20), |m| {
        m.get("sender").is_some_and(|(s, _, _)| s == "Running")
            && m.get("receiver").is_some_and(|(s, _, _)| s == "Running")
    });
    let nodes = parse_node_list(&list_out);
    let sender_initial_pid = nodes
        .get("sender")
        .map(|(_, pid, _)| pid.clone())
        .expect("sender missing after start");
    let receiver_initial_pid = nodes
        .get("receiver")
        .map(|(_, pid, _)| pid.clone())
        .expect("receiver missing after start");

    // --- 1. dora node list ----------------------------------------------
    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list",
    );
    assert!(ok, "dora node list failed.\nstderr:\n{stderr}");
    let parsed = parse_node_list(&stdout);
    assert!(
        parsed.contains_key("sender") && parsed.contains_key("receiver"),
        "list missing base nodes: {parsed:?}"
    );

    // --- 2. dora node info ----------------------------------------------
    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["node", "info", "--dataflow", name, "sender"]),
        "dora node info sender",
    );
    assert!(ok, "dora node info failed.\nstderr:\n{stderr}");
    assert!(
        stdout.contains("Path: sender.py")
            && stdout.contains("Outputs:")
            && stdout.contains("value"),
        "info output missing expected fields:\n{stdout}"
    );

    // --- 3. dora node add (dynamic filter) ------------------------------
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "add",
            "--dataflow",
            name,
            "--from-yaml",
            filter_yml.to_str().unwrap(),
        ]),
        "dora node add filter",
    );
    assert!(ok, "dora node add failed.\nstderr:\n{stderr}");
    let list_out = wait_for_list(&dora, name, Duration::from_secs(10), |m| {
        m.get("filter").is_some_and(|(s, _, _)| s == "Running")
    });
    let nodes = parse_node_list(&list_out);
    assert!(
        nodes.contains_key("filter"),
        "filter missing after add: {nodes:?}"
    );

    // --- 4 + 5. dora node connect (two edges) ---------------------------
    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "connect",
            "--dataflow",
            name,
            "sender/value",
            "filter/input",
        ]),
        "dora node connect 1",
    );
    assert!(ok, "dora node connect 1 failed.\nstderr:\n{stderr}");
    assert!(
        stdout.contains("sender/value -> filter/input"),
        "connect 1 missing success line:\n{stdout}"
    );

    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "connect",
            "--dataflow",
            name,
            "filter/output",
            "receiver/filtered",
        ]),
        "dora node connect 2",
    );
    assert!(ok, "dora node connect 2 failed.\nstderr:\n{stderr}");
    assert!(
        stdout.contains("filter/output -> receiver/filtered"),
        "connect 2 missing success line:\n{stdout}"
    );

    // --- 6. dora node disconnect ----------------------------------------
    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "disconnect",
            "--dataflow",
            name,
            "filter/output",
            "receiver/filtered",
        ]),
        "dora node disconnect",
    );
    assert!(ok, "dora node disconnect failed.\nstderr:\n{stderr}");
    assert!(
        stdout.contains("filter/output -x- receiver/filtered"),
        "disconnect missing success line:\n{stdout}"
    );

    // A second disconnect of the same edge must now error with
    // "mapping ... not found" — proves the prior disconnect actually
    // tore down the routing entry on the daemon (regression guard for
    // PR #1900's idempotence-removal fix).
    let (ok2, _, stderr2) = run_capture(
        Command::new(&dora).args([
            "node",
            "disconnect",
            "--dataflow",
            name,
            "filter/output",
            "receiver/filtered",
        ]),
        "dora node disconnect (duplicate)",
    );
    assert!(
        !ok2 && stderr2.contains("not found"),
        "duplicate disconnect should error with 'not found' \
         (ok={ok2}, stderr=\n{stderr2})"
    );

    // --- 7. dora node remove --------------------------------------------
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args(["node", "remove", "--dataflow", name, "filter"]),
        "dora node remove filter",
    );
    assert!(ok, "dora node remove failed.\nstderr:\n{stderr}");
    let list_out = wait_for_list(&dora, name, Duration::from_secs(15), |m| {
        // After remove, filter should either be missing or transition to
        // `Stopped` (with the 60s grace before disappearing). Either is OK.
        m.get("filter")
            .map(|(s, _, _)| s == "Stopped")
            .unwrap_or(true)
    });
    let nodes = parse_node_list(&list_out);
    let filter_state = nodes.get("filter").map(|(s, _, _)| s.as_str());
    assert!(
        matches!(filter_state, None | Some("Stopped")),
        "filter should be removed or Stopped after remove, got {filter_state:?}"
    );

    // --- 8. dora node stop ----------------------------------------------
    // `stop_single_node` schedules SIGTERM at +10s, SIGKILL at +15s.
    // Wait up to 25s for the Stopped status to land.
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args(["node", "stop", "--dataflow", name, "sender"]),
        "dora node stop sender",
    );
    assert!(ok, "dora node stop failed.\nstderr:\n{stderr}");
    let list_out = wait_for_list(&dora, name, Duration::from_secs(25), |m| {
        m.get("sender").is_some_and(|(s, _, _)| s == "Stopped")
    });
    let nodes = parse_node_list(&list_out);
    let sender_after_stop = nodes
        .get("sender")
        .map(|(s, _, _)| s.clone())
        .unwrap_or_default();
    assert_eq!(
        sender_after_stop, "Stopped",
        "sender should be Stopped after `dora node stop` + grace; got {sender_after_stop:?} (initial pid was {sender_initial_pid})\nlist:\n{list_out}"
    );

    // --- 9. dora node restart -------------------------------------------
    // `restart_single_node` schedules SIGTERM at +5s, SIGKILL at +7.5s,
    // and the restart loop respawns on the next event. Wait up to 20s
    // for the new PID (and restart_count >= 1) to land.
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args(["node", "restart", "--dataflow", name, "receiver"]),
        "dora node restart receiver",
    );
    assert!(ok, "dora node restart failed.\nstderr:\n{stderr}");
    let list_out = wait_for_list(&dora, name, Duration::from_secs(20), |m| {
        m.get("receiver").is_some_and(|(status, pid, restarts)| {
            status == "Running" && pid != &receiver_initial_pid && restarts != "0"
        })
    });
    let nodes = parse_node_list(&list_out);
    let (status, new_pid, restarts) = nodes
        .get("receiver")
        .cloned()
        .expect("receiver missing after restart");
    assert_eq!(
        status, "Running",
        "receiver should be Running after restart; got status={status:?}\nlist:\n{list_out}"
    );
    assert_ne!(
        new_pid, receiver_initial_pid,
        "receiver PID should change after restart; was {receiver_initial_pid}, still {new_pid}\nlist:\n{list_out}"
    );
    assert_ne!(
        restarts, "0",
        "receiver restart_count should be > 0 after restart; got {restarts:?}\nlist:\n{list_out}"
    );

    // Teardown: stop the dataflow, then `dora down` (destroys daemon+coordinator).
    cleanup_stale(&dora);
}
