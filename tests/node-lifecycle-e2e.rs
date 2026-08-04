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
//! (`dora_core::topics::DORA_COORDINATOR_PORT_WS_DEFAULT` for `dora up`),
//! so the in-file `LIFECYCLE_LOCK` serializes the tests within this
//! binary and `--test-threads=1` is required for safety against other
//! integration-test binaries that also use the default port.

use std::fs;
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

static BUILD_RUST_FILTER: Once = Once::new();
static BUILD_FIRE_AND_FORGET: Once = Once::new();

fn ensure_fire_and_forget_built() {
    BUILD_FIRE_AND_FORGET.call_once(|| {
        let dora_root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let target_dir = dora_root.join("target");
        let status = Command::new("cargo")
            .args(["build", "-p", "fire-and-forget-source-node"])
            .arg("--target-dir")
            .arg(&target_dir)
            .status()
            .expect("failed to run cargo build for fire-and-forget-source-node");
        assert!(
            status.success(),
            "failed to build fire-and-forget-source-node"
        );
    });
}

/// Build the `rust-dynamic-add-remove-filter` workspace binary that
/// `lifecycle_rust_dynamic_add_remove` adds via `dora node add
/// --from-yaml filter-node.yml`. Unlike `sender`/`receiver`, the
/// filter is NOT listed in `dataflow.yml` — `dora build` never sees
/// it — and `dora node add` only reads the descriptor + dispatches
/// `AddNode`; it does NOT execute the `build:` field. Without this
/// helper the test panics on `dora node add filter` with `No such
/// file or directory` for `target/debug/rust-dynamic-add-remove-filter`.
///
/// `--target-dir` is pinned to the workspace-relative `target/` to
/// match `filter-node.yml`'s `path: ../../target/debug/...`, so a
/// `CARGO_TARGET_DIR` override on the test invocation can't desync
/// the build location from where the daemon will look at spawn time.
fn ensure_rust_filter_built() {
    BUILD_RUST_FILTER.call_once(|| {
        let dora_root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let target_dir = dora_root.join("target");
        let status = Command::new("cargo")
            .args(["build", "-p", "rust-dynamic-add-remove-filter"])
            .arg("--target-dir")
            .arg(&target_dir)
            .status()
            .expect("failed to run cargo build for rust-dynamic-add-remove-filter");
        assert!(
            status.success(),
            "failed to build rust-dynamic-add-remove-filter"
        );
    });
}

#[cfg(not(windows))]
static BUILD_CXX: Once = Once::new();

/// Drive the CMake build for the `examples/cxx-dynamic-add-remove`
/// fixture. The cmake invocation passes `-DDORA_ROOT_DIR=<root>` so
/// the fixture's `DoraTargets.cmake` knows where to find the dora
/// workspace (it runs `cargo build -p dora-node-api-c` internally
/// via `ExternalProject_Add`). Idempotent — `cmake --build` is a
/// no-op if everything's up to date.
#[cfg(not(windows))]
fn ensure_cxx_built(fixture_dir: &Path) {
    BUILD_CXX.call_once(|| {
        let dora_root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let build_dir = fixture_dir.join("build");
        let configure = Command::new("cmake")
            .arg("-S")
            .arg(fixture_dir)
            .arg("-B")
            .arg(&build_dir)
            .arg(format!("-DDORA_ROOT_DIR={}", dora_root.display()))
            .status()
            .expect("failed to run `cmake -S ... -B ...`");
        assert!(
            configure.success(),
            "cmake configure failed for {}",
            fixture_dir.display()
        );
        let build = Command::new("cmake")
            .arg("--build")
            .arg(&build_dir)
            .status()
            .expect("failed to run `cmake --build`");
        assert!(
            build.success(),
            "cmake --build failed for {}",
            fixture_dir.display()
        );
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

/// Parameters that differ between language variants of the lifecycle test.
struct LifecycleFixture<'a> {
    /// Path to the main dataflow.yml.
    dataflow_path: &'a Path,
    /// Path to the dynamic-add filter-node.yml.
    filter_yml_path: &'a Path,
    /// `dora start`/`stop` --name handle.
    name: &'a str,
    /// `Path: foo.py` (Python) or `Path: ../../target/...` (Rust). Matched
    /// in the `dora node info sender` output assertion to confirm the
    /// info path resolved to the correct fixture.
    sender_path_marker: &'a str,
    /// Pass `--uv` to `dora build`/`start`. Python fixtures need it;
    /// Rust fixtures don't (cargo build is invoked via the `build:`
    /// command in the dataflow.yml).
    use_uv: bool,
}

/// RAII teardown: ensure coordinator/daemon/dataflows are torn down
/// even if a downstream assertion panics. Without this, an early
/// failure leaves the cluster running on port 6013 and contaminates
/// the next test binary (or the next CI retry) with stale state.
struct CleanupGuard<'a> {
    dora: &'a str,
}

impl Drop for CleanupGuard<'_> {
    fn drop(&mut self) {
        cleanup_stale(self.dora);
    }
}

struct StartedLifecycle {
    dora: String,
    sender_initial_pid: String,
    receiver_initial_pid: String,
}

fn start_lifecycle(fixture: &LifecycleFixture<'_>) -> StartedLifecycle {
    ensure_cli_built();
    let dora = dora_bin();
    let dataflow = fixture.dataflow_path;
    let name = fixture.name;

    cleanup_stale(&dora);

    let (ok, _, stderr) = run_capture(Command::new(&dora).arg("up"), "dora up");
    assert!(ok, "dora up failed.\nstderr:\n{stderr}");

    let mut build_args = vec!["build", dataflow.to_str().unwrap()];
    if fixture.use_uv {
        build_args.push("--uv");
    }
    let (ok, _, stderr) = run_capture(Command::new(&dora).args(&build_args), "dora build");
    assert!(ok, "dora build failed.\nstderr:\n{stderr}");

    let mut start_args = vec![
        "start",
        dataflow.to_str().unwrap(),
        "--detach",
        "--name",
        name,
    ];
    if fixture.use_uv {
        start_args.push("--uv");
    }
    let (ok, _, stderr) = run_capture(Command::new(&dora).args(&start_args), "dora start");
    assert!(ok, "dora start failed.\nstderr:\n{stderr}");

    let list_out = wait_for_list(&dora, name, Duration::from_secs(30), |m| {
        m.get("sender").is_some_and(|(s, _, _)| s == "Running")
            && m.get("receiver").is_some_and(|(s, _, _)| s == "Running")
    });
    let nodes = parse_node_list(&list_out);
    let sender_state = nodes.get("sender").cloned();
    let receiver_state = nodes.get("receiver").cloned();
    assert!(
        matches!(&sender_state, Some((s, _, _)) if s == "Running"),
        "sender did not reach Running within 30s; got {sender_state:?}\nlist:\n{list_out}"
    );
    assert!(
        matches!(&receiver_state, Some((s, _, _)) if s == "Running"),
        "receiver did not reach Running within 30s; got {receiver_state:?}\nlist:\n{list_out}"
    );

    StartedLifecycle {
        dora,
        sender_initial_pid: sender_state.unwrap().1,
        receiver_initial_pid: receiver_state.unwrap().1,
    }
}

fn run_lifecycle(fixture: LifecycleFixture<'_>) {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let filter_yml = fixture.filter_yml_path;
    let name = fixture.name;
    let StartedLifecycle {
        dora,
        sender_initial_pid,
        receiver_initial_pid,
    } = start_lifecycle(&fixture);
    let _cleanup = CleanupGuard { dora: &dora };

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
        stdout.contains(fixture.sender_path_marker)
            && stdout.contains("Outputs:")
            && stdout.contains("value"),
        "info output missing expected fields (expected path marker {:?}):\n{stdout}",
        fixture.sender_path_marker
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
    // Tight assertion: a Failed filter (e.g. spawn failure because the
    // binary couldn't be found) would still be in the list, so check
    // status rather than just existence.
    let nodes = parse_node_list(&list_out);
    let filter_state = nodes.get("filter").cloned();
    assert!(
        matches!(&filter_state, Some((s, _, _)) if s == "Running"),
        "filter did not reach Running within 10s after `dora node add`; got {filter_state:?}\nlist:\n{list_out}"
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

    // --- 8. dora node restart -------------------------------------------
    // Run restart BEFORE stop so the sender is still alive and the
    // receiver's input stays open. Without this ordering, dora-rs's
    // Rust `EventStream` closes the stream on any `Event::Stop`
    // (including the synthesized `AllInputsClosed` it generates when
    // sender's output closes — see
    // `apis/rust/node/src/event_stream/mod.rs:484`), and the receiver
    // exits before we can restart it. Python receivers don't share
    // that gate, but ordering this way keeps the matrix consistent
    // across languages.
    //
    // `restart_single_node` schedules SIGTERM at +5s, SIGKILL at
    // +7.5s, and the restart loop respawns on the next exit. Wait up
    // to 20s for the new PID (and `restart_count >= 1`) to land.
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

    // --- 9. dora node stop (terminal) -----------------------------------
    // `stop_single_node` schedules SIGTERM at +10s, SIGKILL at +15s.
    // Wait up to 25s for one of two valid post-stop observable states:
    //   (a) sender row marked `Stopped` and still visible. This is the
    //       normal case for fixtures whose other nodes keep at least
    //       one input open (Rust + C++ variants subscribe to a timer
    //       tick so AllInputsClosed never cascades).
    //   (b) the dataflow finished entirely. Some fixtures (notably the
    //       Python `examples/dynamic-add-remove/dataflow.yml`) wire
    //       receiver's only input to sender, so when sender exits, the
    //       `AllInputsClosed` cascade gates dora-rs's EventStream and
    //       the receiver also exits cleanly. With `running_nodes`
    //       empty, `handle_node_stop_inner`'s `should_finish` triggers
    //       and the dataflow leaves `running_dataflows` — `dora node
    //       list --dataflow X` then returns empty, but `dora list`
    //       reports Status=Finished. Either outcome confirms `dora
    //       node stop` was observable end-to-end.
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args(["node", "stop", "--dataflow", name, "sender"]),
        "dora node stop sender",
    );
    assert!(ok, "dora node stop failed.\nstderr:\n{stderr}");
    let list_out = wait_for_list(&dora, name, Duration::from_secs(25), |m| {
        m.get("sender").is_some_and(|(s, _, _)| s == "Stopped") || m.is_empty()
    });
    let nodes = parse_node_list(&list_out);
    let sender_after_stop = nodes.get("sender").map(|(s, _, _)| s.clone());
    assert!(
        // Sender either visibly Stopped, or the entire list is empty
        // (dataflow finished — verified via `dora list` Status=Finished
        // when this fires, see comment above).
        matches!(&sender_after_stop, Some(s) if s == "Stopped") || nodes.is_empty(),
        "sender should be Stopped after `dora node stop` (or the dataflow should have finished); got {sender_after_stop:?} (initial pid was {sender_initial_pid})\nlist:\n{list_out}"
    );

    // Teardown is handled by `CleanupGuard` on scope exit, so it
    // runs whether we reach this point normally or panic earlier.
}

#[test]
// Python variant is Unix-only because its `--uv` venv path has never
// been exercised on Windows. Tool availability is no longer the
// blocker — the nightly CLI Tests lane installs uv on all three
// platforms — so this gate can be relaxed once someone confirms the
// `--uv` path actually works there.
#[cfg(unix)]
fn lifecycle_python_dynamic_add_remove() {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let dataflow = Path::new(manifest_dir).join("examples/dynamic-add-remove/dataflow.yml");
    let filter_yml = Path::new(manifest_dir).join("examples/dynamic-add-remove/filter-node.yml");
    run_lifecycle(LifecycleFixture {
        dataflow_path: &dataflow,
        filter_yml_path: &filter_yml,
        name: "pylc",
        sender_path_marker: "Path: sender.py",
        use_uv: true,
    });
}

#[test]
fn lifecycle_rust_dynamic_add_remove() {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let dataflow = Path::new(manifest_dir).join("examples/rust-dynamic-add-remove/dataflow.yml");
    let filter_yml =
        Path::new(manifest_dir).join("examples/rust-dynamic-add-remove/filter-node.yml");
    // The `path:` field in the dataflow.yml is a relative target/
    // binary path (`../../target/debug/rust-dynamic-add-remove-sender`).
    // The `dora node info` output prints `Path: <as-specified>`, so we
    // match the suffix portion that's stable across CARGO_TARGET_DIR
    // overrides. dora's `resolve_path` (libraries/core/src/descriptor/
    // mod.rs) automatically appends `EXE_EXTENSION` (.exe) on Windows
    // when the YAML path has no extension, so the same fixture works
    // cross-platform without per-OS YAML.
    //
    // Build the filter binary before run_lifecycle — `dora node add`
    // does not honor `build:` in the dynamic-node yaml, so unlike
    // sender/receiver (built by `dora build dataflow.yml`) the filter
    // has to be compiled explicitly. Mirrors `ensure_cxx_built`.
    ensure_rust_filter_built();
    run_lifecycle(LifecycleFixture {
        dataflow_path: &dataflow,
        filter_yml_path: &filter_yml,
        name: "rustlc",
        sender_path_marker: "rust-dynamic-add-remove-sender",
        use_uv: false,
    });
}

#[test]
fn rust_dynamic_node_readd_same_id_ignores_stale_exit() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let dataflow = Path::new(manifest_dir).join("examples/rust-dynamic-add-remove/dataflow.yml");
    let filter_yml = std::env::temp_dir().join(format!(
        "dora-fire-and-forget-filter-{}.yml",
        std::process::id()
    ));
    fs::write(
        &filter_yml,
        format!(
            "id: filter\npath: {}/target/debug/fire-and-forget-source-node\noutputs:\n  - value\n",
            manifest_dir
        ),
    )
    .expect("failed to write dynamic node spec");

    ensure_fire_and_forget_built();
    let name = "rustlc-readd";
    let fixture = LifecycleFixture {
        dataflow_path: &dataflow,
        filter_yml_path: &filter_yml,
        name,
        sender_path_marker: "rust-dynamic-add-remove-sender",
        use_uv: false,
    };
    let StartedLifecycle { dora, .. } = start_lifecycle(&fixture);
    let _cleanup = CleanupGuard { dora: &dora };

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
        matches!(nodes.get("filter"), Some((s, _, _)) if s == "Running"),
        "filter did not reach Running before remove/re-add; list:\n{list_out}"
    );

    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "remove",
            "--dataflow",
            name,
            "--grace",
            "3",
            "filter",
        ]),
        "dora node remove filter",
    );
    assert!(ok, "dora node remove failed.\nstderr:\n{stderr}");

    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "add",
            "--dataflow",
            name,
            "--from-yaml",
            filter_yml.to_str().unwrap(),
        ]),
        "dora node re-add filter",
    );
    assert!(ok, "dora node re-add failed.\nstderr:\n{stderr}");

    std::thread::sleep(Duration::from_secs(6));
    let (ok, list_out, stderr) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list after stale exit",
    );
    assert!(ok, "dora node list failed.\nstderr:\n{stderr}");
    let nodes = parse_node_list(&list_out);
    assert!(
        matches!(nodes.get("filter"), Some((s, _, _)) if s == "Running"),
        "re-added filter must remain Running after the previous incarnation exits; list:\n{list_out}"
    );
}

static BUILD_TICK_RECORDER: Once = Once::new();

/// Build the `timer-tick-recorder-node` fixture used by the #2585
/// regression test. Same rationale as `ensure_rust_filter_built`: `dora
/// node add` never runs a `build:` field, so the binary has to exist
/// beforehand.
fn ensure_tick_recorder_built() {
    BUILD_TICK_RECORDER.call_once(|| {
        let dora_root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let status = Command::new("cargo")
            .args(["build", "-p", "timer-tick-recorder-node"])
            .arg("--target-dir")
            .arg(dora_root.join("target"))
            .status()
            .expect("failed to run cargo build for timer-tick-recorder-node");
        assert!(status.success(), "failed to build timer-tick-recorder-node");
    });
}

/// Regression for dora-rs/dora#2585: a node added to an already-running
/// dataflow must actually *receive* ticks on a timer input, even when it
/// is the first subscriber of that interval.
///
/// Per-interval tick-emitting tasks are only ever spawned by
/// `RunningDataflow::start()`, which the daemon calls once, on the
/// all-nodes-ready transition. `AddNode` registered the new node in
/// `dataflow.timers` but did not re-invoke `start()`, so an interval no
/// already-running node subscribed to never got a task and the added
/// node's timer input was silent forever. The node still spawns and
/// still reports `Running`, which is why the pre-existing lifecycle
/// coverage (`lifecycle_rust_dynamic_add_remove` adds a node wired to
/// the *sender's output*, and asserts only that it reaches `Running`)
/// could not see this: the only observable is a delivery reported by the
/// node itself.
///
/// The interval is deliberately one the running dataflow does not
/// already use — reusing an existing interval passes even with the bug,
/// because the task spawned at start-up keeps firing and the delivery
/// path re-reads the (now larger) subscriber set on every tick.
#[test]
fn rust_dynamic_node_receives_ticks_on_fresh_timer_interval() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let dataflow = Path::new(manifest_dir).join("examples/rust-dynamic-add-remove/dataflow.yml");

    // Distinct from every interval `examples/rust-dynamic-add-remove`
    // already subscribes to (sender 200ms, receiver 500ms), so the added
    // node is the *first* subscriber of this one and a task genuinely has
    // to be spawned for it.
    const FRESH_INTERVAL_MS: u64 = 300;
    // More than one, so a single stray delivery can't pass for a working
    // timer; small enough to land well inside the poll deadline at 300ms
    // apart.
    const REQUIRED_TICKS: usize = 3;

    // Before writing the spec below: the build both produces the binary
    // the spec points at and guarantees `<manifest>/target/` exists,
    // which a `CARGO_TARGET_DIR` override would otherwise leave absent.
    ensure_tick_recorder_built();

    // Artifacts under `target/` rather than the system temp dir, matching
    // `write_stop_delay_yml`: gitignored, cleaned by `cargo clean`, and
    // worth keeping after a failed run.
    let dir = Path::new(manifest_dir).join("target");
    let stem = format!("dora-timer-tick-{}", std::process::id());
    let ticker_yml = dir.join(format!("{stem}.yml"));
    let marker = dir.join(format!("{stem}.ticks"));
    // A rerun in the same process would otherwise count the previous
    // run's ticks.
    let _ = fs::remove_file(&marker);
    fs::write(
        &ticker_yml,
        format!(
            "id: ticker\n\
             path: {manifest_dir}/target/debug/timer-tick-recorder-node\n\
             env:\n  \
               DORA_TEST_MARKER_FILE: {marker}\n\
             inputs:\n  \
               tick: dora/timer/millis/{FRESH_INTERVAL_MS}\n",
            marker = marker.display()
        ),
    )
    .expect("failed to write ticker node spec");

    let name = "rustlc-timer";
    let fixture = LifecycleFixture {
        dataflow_path: &dataflow,
        filter_yml_path: &ticker_yml,
        name,
        sender_path_marker: "rust-dynamic-add-remove-sender",
        use_uv: false,
    };
    let StartedLifecycle { dora, .. } = start_lifecycle(&fixture);
    let _cleanup = CleanupGuard { dora: &dora };

    add_node(&dora, name, &ticker_yml, "dora node add ticker");
    let list_out = wait_for_list(&dora, name, Duration::from_secs(10), |m| {
        m.get("ticker").is_some_and(|(s, _, _)| s == "Running")
    });
    assert!(
        matches!(parse_node_list(&list_out).get("ticker"), Some((s, _, _)) if s == "Running"),
        "ticker did not reach Running within 10s after `dora node add`; list:\n{list_out}"
    );

    let count_ticks = || {
        fs::read_to_string(&marker)
            .unwrap_or_default()
            .lines()
            .filter(|line| line.trim() == "tick")
            .count()
    };
    // 15s covers ~50 ticks at 300ms, so this only expires if the timer is
    // genuinely dead rather than merely slow to start (the node subscribes
    // a moment after the task begins ticking, and ticks that predate the
    // subscribe are dropped by the daemon's delivery path).
    let deadline = std::time::Instant::now() + Duration::from_secs(15);
    let mut ticks = count_ticks();
    while ticks < REQUIRED_TICKS && std::time::Instant::now() < deadline {
        std::thread::sleep(Duration::from_millis(200));
        ticks = count_ticks();
    }

    assert!(
        marker.exists(),
        "ticker never created its marker file at {}, so it never connected — \
         this test can't say anything about timer delivery",
        marker.display()
    );
    assert!(
        ticks >= REQUIRED_TICKS,
        "dynamically added node got {ticks} ticks on the previously-unused \
         interval {FRESH_INTERVAL_MS}ms within 15s, expected at least \
         {REQUIRED_TICKS} (dora-rs/dora#2585: no tick-emitting task was \
         spawned for an interval registered after the dataflow started)"
    );
}

static BUILD_STOP_DELAY: Once = Once::new();

/// Build the `stop-delay-node` fixture used by the #2916 hot-swap
/// tests. Same rationale as `ensure_rust_filter_built`: `dora node add`
/// never runs a `build:` field, so the binary has to exist beforehand.
fn ensure_stop_delay_built() {
    BUILD_STOP_DELAY.call_once(|| {
        let dora_root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let status = Command::new("cargo")
            .args(["build", "-p", "stop-delay-node"])
            .arg("--target-dir")
            .arg(dora_root.join("target"))
            .status()
            .expect("failed to run cargo build for stop-delay-node");
        assert!(status.success(), "failed to build stop-delay-node");
    });
}

/// A `stop-delay-node` spec plus the marker file its incarnation writes
/// its exit code to.
struct StopDelaySpec {
    yml: std::path::PathBuf,
    marker: std::path::PathBuf,
}

impl StopDelaySpec {
    /// The exit codes recorded by every incarnation spawned from this
    /// spec, oldest first. Empty until one actually exits.
    fn recorded_exits(&self) -> Vec<String> {
        fs::read_to_string(&self.marker)
            .unwrap_or_default()
            .lines()
            .map(|l| l.trim().to_string())
            .filter(|l| !l.is_empty())
            .collect()
    }
}

/// Write a `dora node add --from-yaml` spec for the `stop-delay-node`
/// fixture, wiring its shutdown knobs through the descriptor's `env:`
/// block.
///
/// Artifacts land under `target/` rather than the system temp dir: the
/// tests never delete them (a failed run's spec is worth keeping), and
/// `target/` is both gitignored and cleaned by `cargo clean`, so they
/// don't accumulate in `/tmp` run after run.
fn write_stop_delay_yml(file_stem: &str, delay_ms: u64, exit_code: i32) -> StopDelaySpec {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let dir = Path::new(manifest_dir).join("target");
    let stem = format!("dora-stop-delay-{file_stem}-{}", std::process::id());
    let yml = dir.join(format!("{stem}.yml"));
    let marker = dir.join(format!("{stem}.exits"));
    // A rerun in the same process would otherwise append to the
    // previous run's exit records.
    let _ = fs::remove_file(&marker);
    fs::write(
        &yml,
        format!(
            "id: filter\n\
             path: {manifest_dir}/target/debug/stop-delay-node\n\
             env:\n  \
               DORA_TEST_STOP_DELAY_MS: {delay_ms}\n  \
               DORA_TEST_STOP_EXIT_CODE: {exit_code}\n  \
               DORA_TEST_MARKER_FILE: {marker}\n\
             outputs:\n  - value\n",
            marker = marker.display()
        ),
    )
    .expect("failed to write stop-delay node spec");
    StopDelaySpec { yml, marker }
}

/// Run `dora node add` for the node described by `spec`. Returns once
/// the CLI call completes, without waiting for the node to come up, so
/// a caller can observe state that is only true in the moments right
/// after the add.
fn add_node(dora: &str, name: &str, spec: &Path, label: &str) {
    let (ok, _, stderr) = run_capture(
        Command::new(dora).args([
            "node",
            "add",
            "--dataflow",
            name,
            "--from-yaml",
            spec.to_str().unwrap(),
        ]),
        label,
    );
    assert!(ok, "{label} failed.\nstderr:\n{stderr}");
}

/// Wait for `filter` to report `Running` and return its pid.
fn wait_for_filter_pid(dora: &str, name: &str, label: &str) -> String {
    let list_out = wait_for_list(dora, name, Duration::from_secs(10), |m| {
        m.get("filter").is_some_and(|(s, _, _)| s == "Running")
    });
    parse_node_list(&list_out)
        .get("filter")
        .cloned()
        .filter(|(s, _, _)| s == "Running")
        .unwrap_or_else(|| panic!("filter did not reach Running after {label}; list:\n{list_out}"))
        .1
}

/// Add the `filter` node from `spec`, then wait for it to report
/// `Running` and return its pid.
fn add_filter_and_wait(dora: &str, name: &str, spec: &Path, label: &str) -> String {
    add_node(dora, name, spec, label);
    wait_for_filter_pid(dora, name, label)
}

/// Whether `pid` still names a live process.
///
/// Used to prove a test actually entered the window it claims to cover,
/// rather than passing because the race never happened.
///
/// Deliberately NOT `kill -0`: that succeeds for a zombie — a process
/// that has already exited but whose parent (here the daemon) hasn't
/// reaped it yet. Under load the daemon's reaping task is exactly what
/// falls behind, so `kill -0` would report a dead predecessor as alive
/// and let a test claim a window it never had. `ps -o state=` reports
/// `Z` for that case, which is what makes this check honest.
///
/// A failure to run `ps` at all panics rather than reading as "dead" —
/// silently degrading the liveness check to a no-op would turn the
/// anti-vacuity guards below back into the thing they exist to prevent.
#[cfg(unix)]
fn process_alive(pid: &str) -> bool {
    let out = Command::new("ps")
        .args(["-o", "state=", "-p", pid])
        .output()
        .unwrap_or_else(|e| panic!("failed to run `ps -o state= -p {pid}`: {e}"));
    if !out.status.success() {
        return false; // no such process
    }
    let state = String::from_utf8_lossy(&out.stdout);
    let state = state.trim();
    !state.is_empty() && !state.starts_with('Z')
}

/// Block until `pid` is gone, panicking if it outlives `timeout`.
///
/// `dora node remove` returns once the daemon has scheduled `Stop`, not
/// once the process is reaped, so a test that needs the predecessor's
/// exit to be accounted *before* its next CLI call has to wait for it
/// explicitly. `dora node list` is no help: the row disappears the
/// moment the node is removed and never reappears to report the later
/// exit, so process liveness is the observable.
#[cfg(unix)]
fn wait_for_process_exit(pid: &str, timeout: Duration) {
    let deadline = std::time::Instant::now() + timeout;
    while process_alive(pid) {
        assert!(
            std::time::Instant::now() < deadline,
            "predecessor pid {pid} was still alive {timeout:?} after `node remove`"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

/// Shared setup for the #2916 hot-swap tests: bring up the rust
/// dynamic-add-remove dataflow, with the `stop-delay-node` fixture
/// built and ready to be added as `filter`. The caller owns the
/// returned `dora` path so it can attach its own `CleanupGuard`.
fn start_hot_swap_fixture(name: &str) -> String {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let dataflow = Path::new(manifest_dir).join("examples/rust-dynamic-add-remove/dataflow.yml");
    // `start_lifecycle` reads neither `filter_yml_path` nor
    // `sender_path_marker` (only `run_lifecycle` does), so this path is
    // a placeholder to satisfy the struct — the hot-swap tests add
    // their own `stop-delay-node` spec instead, and the example's
    // filter binary is never built or spawned here.
    let unused_filter_yml =
        Path::new(manifest_dir).join("examples/rust-dynamic-add-remove/filter-node.yml");
    ensure_stop_delay_built();
    let fixture = LifecycleFixture {
        dataflow_path: &dataflow,
        filter_yml_path: &unused_filter_yml,
        name,
        sender_path_marker: "rust-dynamic-add-remove-sender",
        use_uv: false,
    };
    start_lifecycle(&fixture).dora
}

/// Hot-swap regression guard for dora-rs/dora#2916: the predecessor's
/// exit lands *after* the same-id re-add.
///
/// This is the ordering the issue reported. The removed Python node's
/// interpreter teardown outlived the CLI's `node add` round trip, so by
/// the time its exit reached the daemon, `running_nodes["filter"]` had
/// already been replaced by the new incarnation. Every lifecycle
/// bookkeeping path is keyed by node id alone, so the dead
/// predecessor's exit was accounted against the live successor:
/// `handle_node_stop` removed the *new* `running_nodes` entry, whose
/// `ProcessHandle::drop` SIGKILLed the new process ("process was killed
/// on drop because it was still running"), and the resulting `Signal(9)`
/// was recorded under the same id so `dora stop` failed the dataflow.
///
/// `DORA_TEST_STOP_DELAY_MS` pins the ordering rather than relying on a
/// language runtime happening to be slow, and the test asserts the
/// predecessor was still alive when the re-add returned — so a runner
/// slow enough to close the window fails the test instead of passing it
/// vacuously. An explicit long `--grace` on the remove decouples that
/// window from `DEFAULT_STOP_GRACE`, so the delay can carry real margin
/// over a slow CLI round trip while the predecessor still exits on its
/// own rather than being SIGTERMed (which is the shape
/// `rust_dynamic_node_readd_same_id_ignores_stale_exit` already covers).
///
/// No sleep between remove and add — the gap is the bug, and the
/// workaround the issue reports shipping (sleep ~2s) is exactly what
/// this must not need.
#[test]
#[cfg(unix)]
fn hot_swap_survives_predecessor_exit_after_readd() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-swap-late";
    // The predecessor must outlive the `node add` round trip (two debug
    // CLI process lifecycles plus a daemon spawn the daemon awaits
    // before replying). 10s is generous for a cold 2-vCPU runner; the
    // `--grace 60` below keeps it well clear of the kill escalation.
    let spec = write_stop_delay_yml("late", 10_000, 0);
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    let old_pid = add_filter_and_wait(&dora, name, &spec.yml, "dora node add filter");

    // --- the hot swap: remove immediately followed by add ---------------
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "remove",
            "--dataflow",
            name,
            "--grace",
            "60",
            "filter",
        ]),
        "dora node remove filter",
    );
    assert!(ok, "dora node remove failed.\nstderr:\n{stderr}");
    add_node(&dora, name, &spec.yml, "dora node re-add filter");

    // Prove the window this test exists to cover was actually open: the
    // predecessor must still be alive at the moment the re-add returns,
    // so its exit is necessarily accounted *after* the successor was
    // installed. Without this the test would pass vacuously on a runner
    // slow enough that `node add` outlasts the fixture's shutdown delay
    // — it would then be exercising the pre-add ordering, which the
    // sibling test already covers.
    assert!(
        process_alive(&old_pid),
        "predecessor pid {old_pid} already exited before `node add` returned, so this \
         run did not exercise the exit-after-re-add ordering; raise \
         DORA_TEST_STOP_DELAY_MS in the fixture spec"
    );

    let new_pid = wait_for_filter_pid(&dora, name, "dora node re-add filter");
    assert_ne!(
        new_pid, old_pid,
        "re-added filter should be a new process; the daemon reused pid {old_pid}"
    );

    // Wait for the predecessor to actually exit rather than sleeping
    // past its configured delay and hoping. A blind sleep would assert
    // the successor survived an exit that may not have happened yet,
    // which is the same vacuous pass the liveness check above exists to
    // prevent. `process_alive` returns false once the daemon has reaped
    // it, which is exactly when the exit event reaches the event loop.
    wait_for_process_exit(&old_pid, Duration::from_secs(60));
    assert_eq!(
        spec.recorded_exits(),
        vec!["0"],
        "the predecessor should have exited 0 exactly once; \
         marker file {:?} says otherwise",
        spec.marker
    );
    // Settle: let the daemon account the exit (and, if the pid guard
    // regressed, kill the successor) before observing.
    std::thread::sleep(Duration::from_secs(2));
    let (ok, list_out, stderr) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list after swap",
    );
    assert!(ok, "dora node list failed.\nstderr:\n{stderr}");
    let filter_state = parse_node_list(&list_out).get("filter").cloned();
    assert!(
        matches!(&filter_state, Some((s, pid, _)) if s == "Running" && pid == &new_pid),
        "the re-added filter (pid {new_pid}) must still be Running after the \
         predecessor's exit is accounted; got {filter_state:?}\nlist:\n{list_out}"
    );

    // The predecessor's exit must not poison the dataflow result either:
    // `dora stop` fails with "Dataflow <uuid> failed: Node filter failed:
    // exited because of signal SIGKILL" when it does.
    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["stop", "--name", name, "--grace-duration", "5s"]),
        "dora stop",
    );
    assert!(
        ok,
        "dora stop reported the dataflow as failed after a hot swap.\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
}

/// The other ordering in dora-rs/dora#2916: the predecessor's exit
/// lands *before* the same-id re-add.
///
/// The daemon's stale-exit guard compares the exiting pid against
/// `running_nodes[node_id]`, so it can only fire once the successor is
/// installed. Here there is nothing to compare against — `RemoveNode`
/// took the entry out and `AddNode` has not arrived yet — so the
/// removed incarnation's failure *is* recorded under the bare node id
/// (`dataflow_node_results[node_id] = Err`), and nothing clears it when
/// the id is re-added. What keeps this benign today is incidental: the
/// successor's own clean exit overwrites that map entry at teardown.
/// The assertion pins the survivable outcome so a future change to how
/// node results are keyed or aggregated can't quietly turn it into the
/// contradiction the issue reports — `dora node list` saying `filter`
/// is Running while `dora stop` blames `filter` for a dead
/// predecessor's exit code.
///
/// Modelled on the workflow the issue describes: swap a node that fails
/// on the way out for a fixed build. The predecessor exits 1 the
/// instant it sees `Stop` (`DORA_TEST_STOP_DELAY_MS: 0`) and the test
/// waits for that exit before re-adding, so the ordering is enforced
/// rather than assumed; the replacement exits cleanly, so the only
/// failure in the dataflow's history belongs to an incarnation the
/// operator explicitly removed.
#[test]
#[cfg(unix)]
fn hot_swap_survives_predecessor_exit_before_readd() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-swap-early";
    let broken = write_stop_delay_yml("early-broken", 0, 1);
    let fixed = write_stop_delay_yml("early-fixed", 0, 0);
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    let old_pid = add_filter_and_wait(&dora, name, &broken.yml, "dora node add filter (broken)");

    // --- the hot swap: remove the broken build, add the fixed one -------
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args(["node", "remove", "--dataflow", name, "filter"]),
        "dora node remove filter",
    );
    assert!(ok, "dora node remove failed.\nstderr:\n{stderr}");

    // `node remove` returns once `Stop` is scheduled, not once the
    // process is gone, so re-adding immediately would leave the
    // ordering to chance: if the daemon handled `AddNode` first, the
    // pid guard would discard the predecessor's failure and this test
    // would silently re-run the sibling test's ordering instead of the
    // one it documents. Waiting for the process to exit rules that out
    // — the exit event is queued before the replacement's CLI round
    // trip even begins, and a slower runner only widens that margin.
    wait_for_process_exit(&old_pid, Duration::from_secs(30));

    // Pin the premise: this test is only meaningful if the removed
    // incarnation exited NON-ZERO, i.e. there is a real failure that
    // could be misattributed to the successor. Without this the
    // scenario degrades silently into a plain remove/add — the
    // `DORA_TEST_STOP_EXIT_CODE` knob getting dropped (env denylisted,
    // renamed, typo'd) would leave every other assertion below
    // unchanged and still green.
    assert_eq!(
        broken.recorded_exits(),
        vec!["1"],
        "the removed incarnation must have exited 1 for this test to mean \
         anything; marker file {:?} says otherwise",
        broken.marker
    );

    let new_pid = add_filter_and_wait(&dora, name, &fixed.yml, "dora node re-add filter (fixed)");
    assert_ne!(
        new_pid, old_pid,
        "re-added filter should be a new process; the daemon reused pid {old_pid}"
    );

    std::thread::sleep(Duration::from_secs(3));
    let (ok, list_out, stderr) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list after swap",
    );
    assert!(ok, "dora node list failed.\nstderr:\n{stderr}");
    let filter_state = parse_node_list(&list_out).get("filter").cloned();
    assert!(
        matches!(&filter_state, Some((s, pid, _)) if s == "Running" && pid == &new_pid),
        "the re-added filter (pid {new_pid}) must be Running; got {filter_state:?}\nlist:\n{list_out}"
    );

    // The removed incarnation's exit code must not be attributed to the
    // healthy successor that now owns the id.
    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["stop", "--name", name, "--grace-duration", "5s"]),
        "dora stop",
    );
    assert!(
        ok,
        "`dora node list` reports filter Running, but `dora stop` blames it for \
         the removed incarnation's exit.\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
}

/// The dynamic-successor ordering from dora-rs/dora#2926 (scenario 2):
/// a removed incarnation's failure lands while the id has no entry, so
/// it is recorded under the bare node id — and a *dynamic* successor
/// never emits a `SpawnedNodeResult` that would incidentally overwrite
/// it. Only the `AddNode` handler's `clear_node_result` keeps the dead
/// predecessor's exit code from being attributed to the successor when
/// the dataflow stops.
///
/// (The issue's other named scenario — a `restart_loop` respawn racing
/// a same-id re-add — has no deterministic e2e without a spawn-hold
/// test hook in the daemon; both of its sides are pinned at unit level
/// instead: `stale_handle_replacement_preserves_live_process` for the
/// daemon-side rejection and `cancelled_restart_settles_terminal_exit`
/// for the loop side.)
#[test]
#[cfg(unix)]
fn hot_swap_dynamic_successor_not_blamed_for_predecessor_exit() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-swap-dynamic";
    let broken = write_stop_delay_yml("dynamic-broken", 0, 1);
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    let old_pid = add_filter_and_wait(&dora, name, &broken.yml, "dora node add filter (broken)");

    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args(["node", "remove", "--dataflow", name, "filter"]),
        "dora node remove filter",
    );
    assert!(ok, "dora node remove failed.\nstderr:\n{stderr}");

    // Enforce the ordering this scenario is about: the failure must be
    // recorded while the id has no running_nodes entry.
    wait_for_process_exit(&old_pid, Duration::from_secs(30));
    assert_eq!(
        broken.recorded_exits(),
        vec!["1"],
        "the removed incarnation must have exited 1 for this test to mean \
         anything; marker file {:?} says otherwise",
        broken.marker
    );

    // Re-add the same id as a DYNAMIC node: it never spawns a process
    // and never reports a result, so nothing ever overwrites the
    // predecessor's recorded failure — only `AddNode`'s result clearing
    // protects the successor.
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let dynamic_spec = Path::new(manifest_dir)
        .join("target")
        .join(format!("dora-dynamic-successor-{}.yml", std::process::id()));
    fs::write(
        &dynamic_spec,
        "id: filter\npath: dynamic\noutputs:\n  - value\n",
    )
    .expect("failed to write dynamic successor spec");
    add_node(
        &dora,
        name,
        &dynamic_spec,
        "dora node re-add filter (dynamic)",
    );

    // Let the daemon settle the add before stopping.
    std::thread::sleep(Duration::from_secs(2));

    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["stop", "--name", name, "--grace-duration", "5s"]),
        "dora stop",
    );
    assert!(
        ok,
        "`dora stop` blames the dynamic successor for the removed \
         incarnation's exit 1 — the stale node result was not cleared on \
         re-add.\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
}

/// Pins the deliberate keep-alive semantics of the generation guard
/// (dora-rs/dora#2926 review, D2): an exit event whose entry was removed
/// runs no finish accounting, so removing every node leaves a zero-node
/// dataflow alive — for live editing and same-id re-adds — until an
/// explicit `dora stop`, which must then succeed cleanly.
#[test]
#[cfg(unix)]
fn removing_all_nodes_keeps_dataflow_alive_until_explicit_stop() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-remove-all";
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    for node in ["receiver", "sender"] {
        let (ok, _, stderr) = run_capture(
            Command::new(&dora).args(["node", "remove", "--dataflow", name, node]),
            "dora node remove",
        );
        assert!(ok, "dora node remove {node} failed.\nstderr:\n{stderr}");
    }

    // Give the removed nodes' exit events time to reach the daemon; the
    // generation guard must drop them without finishing the dataflow.
    std::thread::sleep(Duration::from_secs(3));

    let (ok, list_out, stderr) = run_capture(
        Command::new(&dora).args(["list", "--format", "json"]),
        "dora list after removing all nodes",
    );
    assert!(ok, "dora list failed.\nstderr:\n{stderr}");
    let entry = list_out
        .lines()
        .find(|l| l.contains(&format!("\"name\":\"{name}\"")))
        .unwrap_or_else(|| panic!("dataflow {name} missing from list:\n{list_out}"));
    assert!(
        entry.contains("\"status\":\"Running\""),
        "a fully-emptied dataflow must stay alive until an explicit stop \
         (live-editing keep-alive); got:\n{entry}"
    );

    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["stop", "--name", name, "--grace-duration", "5s"]),
        "dora stop",
    );
    assert!(
        ok,
        "`dora stop` on an intentionally-emptied dataflow must succeed.\
         \nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
}

/// The atomic swap from dora-rs/dora#2927: one `dora node replace`
/// command swaps a running node for a new definition under the same id.
/// The outgoing incarnation here exits 1 on stop, so the test also pins
/// that its exit is not attributed to the replacement: `dora node list`
/// shows the successor Running and `dora stop` reports a clean dataflow.
#[test]
#[cfg(unix)]
fn replace_swaps_running_node_without_blaming_successor() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-replace";
    // The outgoing incarnation exits 1 the moment it is stopped — a real
    // failure that must stay attributed to the incarnation, not the id.
    let outgoing = write_stop_delay_yml("replace-outgoing", 0, 1);
    let replacement = write_stop_delay_yml("replace-fixed", 0, 0);
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    let old_pid = add_filter_and_wait(&dora, name, &outgoing.yml, "dora node add filter");

    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "replace",
            "--dataflow",
            name,
            "filter",
            "--from-yaml",
            replacement.yml.to_str().unwrap(),
            "--grace",
            "5s",
        ]),
        "dora node replace filter",
    );
    assert!(ok, "dora node replace failed.\nstderr:\n{stderr}");

    // The replacement is spawned before the outgoing incarnation is
    // stopped, so by the time the command returns a new pid exists.
    let new_pid = wait_for_filter_pid(&dora, name, "dora node replace filter");
    assert_ne!(
        new_pid, old_pid,
        "replace must spawn a new process; the daemon kept pid {old_pid}"
    );

    // The outgoing incarnation received Stop and exits (with code 1).
    wait_for_process_exit(&old_pid, Duration::from_secs(30));
    // Let the daemon account that exit before observing.
    std::thread::sleep(Duration::from_secs(2));

    let (ok, list_out, stderr) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list after replace",
    );
    assert!(ok, "dora node list failed.\nstderr:\n{stderr}");
    let filter_state = parse_node_list(&list_out).get("filter").cloned();
    assert!(
        matches!(&filter_state, Some((s, pid, _)) if s == "Running" && pid == &new_pid),
        "the replacement (pid {new_pid}) must be Running after the outgoing \
         incarnation's exit; got {filter_state:?}\nlist:\n{list_out}"
    );

    // Replace the replacement: successive swaps must keep working (each
    // incarnation gets a fresh generation and clean bookkeeping).
    let third = write_stop_delay_yml("replace-third", 0, 0);
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "replace",
            "--dataflow",
            name,
            "filter",
            "--from-yaml",
            third.yml.to_str().unwrap(),
        ]),
        "dora node replace filter (second swap)",
    );
    assert!(ok, "second replace failed.\nstderr:\n{stderr}");
    let third_pid = wait_for_filter_pid(&dora, name, "second replace");
    assert_ne!(third_pid, new_pid, "second swap must spawn a third process");
    wait_for_process_exit(&new_pid, Duration::from_secs(30));
    std::thread::sleep(Duration::from_secs(2));

    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["stop", "--name", name, "--grace-duration", "5s"]),
        "dora stop",
    );
    assert!(
        ok,
        "`dora stop` blames the replacement for the outgoing incarnation's \
         exit 1.\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
}

/// The core hot-swap promise (dora-rs/dora#2927): replacing a producer
/// must not disturb its consumers. The outgoing incarnation's graceful
/// shutdown emits close-outputs/outputs-done under the shared node id;
/// without generation attribution on node-connection events those would
/// close the consumer's only input and terminate it (the P1 this test
/// pins red/green).
#[test]
#[cfg(unix)]
fn replace_does_not_disturb_downstream_consumers() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-replace-consumer";
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let target = Path::new(manifest_dir).join("target");
    let run_id = std::process::id();
    let producer = write_stop_delay_yml("replace-producer", 0, 0);
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    let old_pid = add_filter_and_wait(&dora, name, &producer.yml, "dora node add filter");

    // A sink whose ONLY input comes from the node under replacement:
    // if the swap leaks the outgoing incarnation's close-outputs, the
    // sink sees AllInputsClosed and exits — the red observable.
    let sink_spec = target.join(format!("dora-replace-sink-{run_id}.yml"));
    fs::write(
        &sink_spec,
        format!(
            "id: sink\n\
             path: {manifest_dir}/target/debug/stop-delay-node\n\
             inputs:\n  value: filter/value\n"
        ),
    )
    .expect("write sink spec");
    add_node(&dora, name, &sink_spec, "dora node add sink");
    let list_out = wait_for_list(&dora, name, Duration::from_secs(10), |m| {
        m.get("sink").is_some_and(|(s, _, _)| s == "Running")
    });
    let sink_pid = parse_node_list(&list_out)
        .get("sink")
        .cloned()
        .expect("sink row")
        .1;

    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "replace",
            "--dataflow",
            name,
            "filter",
            "--from-yaml",
            producer.yml.to_str().unwrap(),
            "--grace",
            "5s",
        ]),
        "dora node replace filter (with consumer)",
    );
    assert!(ok, "dora node replace failed.\nstderr:\n{stderr}");

    let new_pid = wait_for_filter_pid(&dora, name, "replace with consumer");
    assert_ne!(new_pid, old_pid);
    wait_for_process_exit(&old_pid, Duration::from_secs(30));
    // Settle: let the outgoing incarnation's teardown events (if any
    // leaked past the generation guard) reach the sink.
    std::thread::sleep(Duration::from_secs(3));

    let (ok, list_out, stderr) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list after consumer-preserving replace",
    );
    assert!(ok, "dora node list failed.\nstderr:\n{stderr}");
    let nodes = parse_node_list(&list_out);
    assert!(
        matches!(nodes.get("sink"), Some((s, pid, _)) if s == "Running" && pid == &sink_pid),
        "the consumer must survive its producer's replacement; got {:?}\nlist:\n{list_out}",
        nodes.get("sink")
    );
    assert!(
        matches!(nodes.get("filter"), Some((s, pid, _)) if s == "Running" && pid == &new_pid),
        "replacement must be Running; got {:?}\nlist:\n{list_out}",
        nodes.get("filter")
    );

    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["stop", "--name", name, "--grace-duration", "5s"]),
        "dora stop",
    );
    assert!(
        ok,
        "dora stop failed.\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
}

/// Edge and coverage rejections against live topology
/// (dora-rs/dora#2927): with a timer input on the node and a wired
/// consumer, a replacement that drops the input, remaps it, or stops
/// declaring the consumed output must be rejected with the original
/// incarnation untouched.
#[test]
#[cfg(unix)]
fn replace_rejects_edge_and_output_changes() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-replace-edges";
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let target = Path::new(manifest_dir).join("target");
    let run_id = std::process::id();
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    // Original filter: a timer input and a `value` output.
    let with_input = target.join(format!("dora-replace-edges-orig-{run_id}.yml"));
    fs::write(
        &with_input,
        format!(
            "id: filter\n\
             path: {manifest_dir}/target/debug/stop-delay-node\n\
             inputs:\n  tick: dora/timer/millis/200\n\
             outputs:\n  - value\n"
        ),
    )
    .expect("write original spec");
    let pid = add_filter_and_wait(&dora, name, &with_input, "dora node add filter");

    // Wire a consumer so the output-coverage branch is live.
    let sink_spec = target.join(format!("dora-replace-edges-sink-{run_id}.yml"));
    fs::write(
        &sink_spec,
        format!(
            "id: sink\n\
             path: {manifest_dir}/target/debug/stop-delay-node\n\
             inputs:\n  value: filter/value\n"
        ),
    )
    .expect("write sink spec");
    add_node(&dora, name, &sink_spec, "dora node add sink");

    let cases: [(&str, String, &str); 3] = [
        (
            "drops-input",
            format!(
                "id: filter\npath: {manifest_dir}/target/debug/stop-delay-node\noutputs:\n  - value\n"
            ),
            "drops input",
        ),
        (
            "remaps-input",
            format!(
                "id: filter\npath: {manifest_dir}/target/debug/stop-delay-node\n\
                 inputs:\n  tick: dora/timer/millis/500\noutputs:\n  - value\n"
            ),
            "remaps input",
        ),
        (
            "missing-output",
            format!(
                "id: filter\npath: {manifest_dir}/target/debug/stop-delay-node\n\
                 inputs:\n  tick: dora/timer/millis/200\n"
            ),
            "does not declare output",
        ),
    ];
    for (label, spec_yaml, expected) in cases {
        let spec = target.join(format!("dora-replace-edges-{label}-{run_id}.yml"));
        fs::write(&spec, spec_yaml).expect("write case spec");
        let (ok, _, stderr) = run_capture(
            Command::new(&dora).args([
                "node",
                "replace",
                "--dataflow",
                name,
                "filter",
                "--from-yaml",
                spec.to_str().unwrap(),
            ]),
            "dora node replace (edge case)",
        );
        assert!(!ok, "case `{label}` must be rejected");
        assert!(
            stderr.contains(expected),
            "case `{label}` should mention `{expected}`; stderr:\n{stderr}"
        );
    }

    // Identical-edges replacement (timer input + output) must succeed —
    // this also exercises the with-inputs swap path end to end.
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "replace",
            "--dataflow",
            name,
            "filter",
            "--from-yaml",
            with_input.to_str().unwrap(),
        ]),
        "dora node replace (identical edges)",
    );
    assert!(ok, "identical-edges replace failed.\nstderr:\n{stderr}");
    let new_pid = wait_for_filter_pid(&dora, name, "identical-edges replace");
    assert_ne!(
        new_pid, pid,
        "identical-edges replace must swap the process"
    );

    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["stop", "--name", name, "--grace-duration", "5s"]),
        "dora stop",
    );
    assert!(
        ok,
        "dora stop failed.\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
}

/// Failure semantics from dora-rs/dora#2927: a replacement that cannot
/// spawn (bad path) or that changes the node's edges must be rejected
/// with the current incarnation left running.
#[test]
#[cfg(unix)]
fn replace_failure_keeps_current_incarnation_running() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-replace-fail";
    let good = write_stop_delay_yml("replace-good", 5_000, 0);
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    let pid = add_filter_and_wait(&dora, name, &good.yml, "dora node add filter");

    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let target = Path::new(manifest_dir).join("target");
    let run_id = std::process::id();

    // Spawn failure: the binary does not exist.
    let bad_path_spec = target.join(format!("dora-replace-badpath-{run_id}.yml"));
    fs::write(
        &bad_path_spec,
        format!(
            "id: filter\npath: {manifest_dir}/target/debug/does-not-exist\noutputs:\n  - value\n"
        ),
    )
    .expect("write bad-path spec");
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "replace",
            "--dataflow",
            name,
            "filter",
            "--from-yaml",
            bad_path_spec.to_str().unwrap(),
        ]),
        "dora node replace (bad path)",
    );
    assert!(
        !ok,
        "replace with a nonexistent binary must fail instead of killing the \
         running node"
    );
    assert!(
        stderr.contains("replacement"),
        "the failure must name the replacement spawn (anchoring that the \
         spawn-first path was exercised, not an unrelated error); stderr:\n{stderr}"
    );

    // Edge change: the replacement adds an input the current node lacks.
    let edge_spec = target.join(format!("dora-replace-edges-{run_id}.yml"));
    fs::write(
        &edge_spec,
        format!(
            "id: filter\npath: {manifest_dir}/target/debug/stop-delay-node\n\
             inputs:\n  tick: dora/timer/millis/100\noutputs:\n  - value\n"
        ),
    )
    .expect("write edge-change spec");
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "node",
            "replace",
            "--dataflow",
            name,
            "filter",
            "--from-yaml",
            edge_spec.to_str().unwrap(),
        ]),
        "dora node replace (edge change)",
    );
    assert!(
        !ok,
        "replace that changes the node's edges must be rejected"
    );
    assert!(
        stderr.contains("edges") || stderr.contains("input"),
        "edge-change rejection should name the offending input; stderr:\n{stderr}"
    );

    // Both failed replaces must have left the current incarnation alone.
    let (ok, list_out, stderr) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list after failed replaces",
    );
    assert!(ok, "dora node list failed.\nstderr:\n{stderr}");
    let filter_state = parse_node_list(&list_out).get("filter").cloned();
    assert!(
        matches!(&filter_state, Some((s, p, _)) if s == "Running" && p == &pid),
        "failed replaces must leave the original incarnation (pid {pid}) \
         running; got {filter_state:?}\nlist:\n{list_out}"
    );

    let (ok, stdout, stderr) = run_capture(
        Command::new(&dora).args(["stop", "--name", name, "--grace-duration", "10s"]),
        "dora stop",
    );
    assert!(
        ok,
        "dora stop failed.\nstdout:\n{stdout}\nstderr:\n{stderr}"
    );
}

/// A binary that exits non-zero immediately without ever connecting to
/// the daemon — the "crashed at import" shape from dora-rs/dora#2917.
///
/// `false(1)` rather than a fixture crate: it needs to do nothing at
/// all, and adding another workspace member would put another nested
/// `cargo build` on the critical path of this already-slow CI job.
#[cfg(unix)]
fn never_subscribes_binary() -> &'static str {
    ["/usr/bin/false", "/bin/false"]
        .into_iter()
        .find(|p| Path::new(p).exists())
        .expect("no `false` binary found at /usr/bin/false or /bin/false")
}

/// Write a `dora node add --from-yaml` spec for an arbitrary id/path.
#[cfg(unix)]
fn write_node_yml(file_stem: &str, id: &str, path: &str) -> std::path::PathBuf {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
    let yml = dir.join(format!("dora-{file_stem}-{}.yml", std::process::id()));
    fs::write(
        &yml,
        format!("id: {id}\npath: {path}\noutputs:\n  - value\n"),
    )
    .expect("failed to write node spec");
    yml
}

/// Regression guard for dora-rs/dora#2917: one node that dies before
/// subscribing must not fail an unrelated node's `Node()` init.
///
/// `PendingNodes` is a dataflow-wide *startup* barrier. When a node
/// exits before subscribing it lands in `exited_before_subscribe`, and
/// `PendingNodes::answer_subscribe_requests` hands every waiting
/// subscriber one shared `Err` naming that node. That is correct while
/// the dataflow is starting — if a node dies then, nothing can start.
///
/// Nodes added later *used to* join the same barrier via
/// `DaemonCoordinatorEvent::AddNode`, so the error reached nodes that
/// had nothing to do with it. They are no longer enrolled, and only
/// members of the startup cohort inherit its failures.
///
/// This pins the *sticky* variant rather than the concurrent race the
/// issue reported, because it is deterministic: `exited_before_subscribe`
/// is only ever pushed to, never cleared, so a single crashed node
/// poisons every later `dora node add` on that dataflow for as long as
/// it runs. The reporter's workaround (sequence the adds) does not help
/// here — the crash and the healthy add are already fully sequenced.
#[test]
#[cfg(unix)]
fn dynamic_add_survives_earlier_crash_before_subscribe() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-crosstalk";
    let crasher_yml = write_node_yml("crasher", "crasher", never_subscribes_binary());
    let healthy = write_stop_delay_yml("crosstalk-healthy", 0, 0);
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    // 0. Let the dataflow's own nodes finish subscribing before adding
    //    anything. `Running` in `dora node list` only means the process
    //    is in `running_nodes`; it does not mean the node has subscribed
    //    yet, and there is no CLI observable for that. Adding the
    //    crasher while the initial cohort is still mid-subscribe hits a
    //    harsher variant of the same bug (asserted at step 3 below), and
    //    this test is pinning the sticky one.
    std::thread::sleep(Duration::from_secs(8));

    // 1. A node that exits non-zero without ever reaching the daemon.
    //    `node add` still succeeds: the daemon replies once the spawn
    //    succeeds, and this process only fails afterwards.
    add_node(&dora, name, &crasher_yml, "dora node add crasher");

    // 2. Let the daemon account that exit. Fully sequenced — no overlap
    //    with the healthy add below, so nothing here is a race.
    let list_out = wait_for_list(&dora, name, Duration::from_secs(20), |m| {
        m.get("crasher").is_none_or(|(s, _, _)| s != "Running")
    });
    assert!(
        parse_node_list(&list_out)
            .get("crasher")
            .is_none_or(|(s, _, _)| s != "Running"),
        "crasher should not be Running; it exits immediately\nlist:\n{list_out}"
    );
    std::thread::sleep(Duration::from_secs(2));

    // 3. The dataflow's own nodes must be untouched. If they are gone,
    //    the crasher landed while they were still subscribing and took
    //    the whole startup cohort down with it — the same
    //    `exited_before_subscribe` broadcast, one step earlier. That is
    //    a real defect too, but it is not what this test pins, and
    //    letting it fall through would fail the healthy add below with
    //    a misleading "no dataflow is running".
    let (_, list_out, _) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list after crasher",
    );
    let nodes = parse_node_list(&list_out);
    assert!(
        matches!(nodes.get("sender"), Some((s, _, _)) if s == "Running")
            && matches!(nodes.get("receiver"), Some((s, _, _)) if s == "Running"),
        "the crasher took down the dataflow's own nodes, so it landed during their \
         subscribe window rather than after it — raise the settle at step 0 to pin the \
         sticky variant (#2917)\nlist:\n{list_out}"
    );

    // 4. A healthy, unrelated node added afterwards must come up.
    //    Before #2917 its `Node()` init was answered with the crasher's
    //    failure ("Node crasher exited before initializing dora"), so it
    //    exited and never reached Running.
    add_node(&dora, name, &healthy.yml, "dora node add healthy");

    let list_out = wait_for_list(&dora, name, Duration::from_secs(20), |m| {
        m.get("filter").is_some_and(|(s, _, _)| s == "Running")
    });
    let filter_state = parse_node_list(&list_out).get("filter").cloned();
    assert!(
        matches!(&filter_state, Some((s, _, _)) if s == "Running"),
        "a healthy node added after an unrelated node crashed before subscribing \
         must still initialize; got {filter_state:?}. Check `dora logs {name} --node filter` \
         for the other node's error delivered into this one's `Node()` init (#2917).\
         \nlist:\n{list_out}"
    );
}

/// The harsher half of dora-rs/dora#2917: a runtime-added node that
/// crashes *while the dataflow's own nodes are still subscribing* must
/// not take that whole cohort down with it.
///
/// Same broadcast as the sibling test, one step earlier. The crasher
/// used to land in `local_nodes` (`AddNode`, `lib.rs`), so its exit was
/// recorded in `exited_before_subscribe` and delivered as the subscribe
/// result of the dataflow's own sender and receiver — killing a
/// dataflow that was running fine, rather than only poisoning later
/// additions.
///
/// Deliberately adds the crasher with no settle, which is what makes
/// this the startup-window case: `start_lifecycle` returns as soon as
/// both nodes report `Running`, and `Running` only means "in
/// `running_nodes`", not "subscribed".
///
/// Verified to discriminate: against the pre-fix daemon this fails with
/// `sender=None receiver=None` — the cohort is gone — while the sibling
/// test fails on its own healthy node instead. Entry into the window is
/// still timing-dependent and unobserved; if the cohort has already
/// subscribed by the time the crasher lands, this degrades to asserting
/// that a dead runtime addition is harmless, which is worth asserting
/// anyway.
#[test]
#[cfg(unix)]
fn crash_before_subscribe_does_not_kill_the_startup_cohort() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let name = "rustlc-cohort";
    let crasher_yml = write_node_yml("cohort-crasher", "crasher", never_subscribes_binary());
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };

    add_node(&dora, name, &crasher_yml, "dora node add crasher");

    // Outlast the crasher's exit and the daemon accounting it.
    std::thread::sleep(Duration::from_secs(6));

    let (ok, list_out, stderr) = run_capture(
        Command::new(&dora).args(["node", "list", "--dataflow", name, "--format", "json"]),
        "dora node list after crasher",
    );
    assert!(
        ok,
        "dora node list failed — the dataflow itself is gone, so the crasher took \
         the startup cohort with it (#2917).\nstderr:\n{stderr}"
    );
    let nodes = parse_node_list(&list_out);
    assert!(
        matches!(nodes.get("sender"), Some((s, _, _)) if s == "Running")
            && matches!(nodes.get("receiver"), Some((s, _, _)) if s == "Running"),
        "the dataflow's own nodes must survive a runtime-added node crashing before \
         it subscribed; got sender={:?} receiver={:?}\nlist:\n{list_out}",
        nodes.get("sender"),
        nodes.get("receiver")
    );
}

/// dora-rs/dora#2918: a node whose `path:` is a virtualenv interpreter
/// must actually run inside that venv.
///
/// A venv's `bin/python` is a symlink, and CPython finds `pyvenv.cfg`
/// relative to the path it was *invoked* as — that is the entire venv
/// mechanism. `resolve_path` used to `canonicalize()` before exec, which
/// resolved the symlink and ran the base interpreter instead: imports
/// that worked in a shell (`.venv/bin/python -c "import numpy"`) failed
/// under dora with `ModuleNotFoundError`, and the documented escape
/// hatch for giving a dynamic node an explicit environment didn't work.
///
/// The probe is not a dora node (that would need dora-rs installed into
/// the scratch venv — network, plus the PyPI/workspace drift from
/// #1710). It writes `sys.prefix` to a marker and exits; the daemon
/// marks it Failed, which post-#2933 harms nothing, and the marker tells
/// us which interpreter really ran. The venv is created with
/// `--without-pip`, so this needs only a bare `python3`.
#[test]
#[cfg(unix)]
fn venv_interpreter_path_runs_inside_the_venv() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let target = Path::new(manifest_dir).join("target");
    let stem = format!("dora-venv-2918-{}", std::process::id());

    // The artifacts are pid-suffixed, so a previous run's venv (a few MB)
    // is never reused — reclaim any stale ones instead of accumulating
    // one per invocation until `cargo clean`.
    if let Ok(entries) = fs::read_dir(&target) {
        for entry in entries.flatten() {
            if entry
                .file_name()
                .to_string_lossy()
                .starts_with("dora-venv-2918-")
            {
                let path = entry.path();
                let _ = if path.is_dir() {
                    fs::remove_dir_all(&path)
                } else {
                    fs::remove_file(&path)
                };
            }
        }
    }

    let venv = target.join(format!("{stem}-venv"));
    let status = Command::new("python3")
        .args(["-m", "venv", "--without-pip"])
        .arg(&venv)
        .status()
        .expect("failed to run python3 -m venv");
    assert!(status.success(), "python3 -m venv failed");
    let venv_python = venv.join("bin/python");
    assert!(
        venv_python
            .symlink_metadata()
            .expect("venv python missing")
            .file_type()
            .is_symlink(),
        "test premise: {} should be a symlink (venvs on unix)",
        venv_python.display()
    );

    let probe = target.join(format!("{stem}-probe.py"));
    let marker = target.join(format!("{stem}-prefix.txt"));
    let _ = fs::remove_file(&marker);
    fs::write(
        &probe,
        "import sys, os\nopen(os.environ['PROBE_OUT'], 'w').write(sys.prefix)\n",
    )
    .expect("failed to write probe script");

    // The probe interpreter itself is not a `.py` path, so the yaml's
    // `path:` is the venv python and the script rides in `args:` —
    // exactly the shape from the issue report.
    let spec = target.join(format!("{stem}.yml"));
    fs::write(
        &spec,
        format!(
            "id: venvprobe\n\
             path: {python}\n\
             args: {probe}\n\
             env:\n  \
               PROBE_OUT: {marker}\n\
             outputs:\n  - value\n",
            python = venv_python.display(),
            probe = probe.display(),
            marker = marker.display(),
        ),
    )
    .expect("failed to write node spec");

    let name = "rustlc-venv";
    let dora = start_hot_swap_fixture(name);
    let _cleanup = CleanupGuard { dora: &dora };
    add_node(&dora, name, &spec, "dora node add venvprobe");

    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    while !marker.exists() {
        assert!(
            std::time::Instant::now() < deadline,
            "probe never ran — check `dora logs {name} --node venvprobe`"
        );
        std::thread::sleep(Duration::from_millis(200));
    }
    let prefix = fs::read_to_string(&marker).expect("failed to read marker");
    // Compare on canonicalized forms: sys.prefix reflects how CPython
    // resolved the venv dir, which may differ from our path in /tmp
    // symlinkage (e.g. /tmp vs /private/tmp on macOS).
    let got = Path::new(prefix.trim())
        .canonicalize()
        .unwrap_or_else(|_| std::path::PathBuf::from(prefix.trim()));
    let want = venv.canonicalize().expect("venv path should canonicalize");
    assert_eq!(
        got, want,
        "the spawned node ran OUTSIDE its venv: sys.prefix is {got:?} instead of the \
         venv — the interpreter symlink was resolved before exec, bypassing pyvenv.cfg \
         discovery (#2918)"
    );
}

/// Removing a node that has not yet subscribed must release the startup
/// barrier — the `RemoveNode` wiring, which no other test covers.
///
/// The unit tests in `pending.rs` prove `handle_node_removal` reports
/// the cohort ready; nothing proved the daemon actually calls it and
/// starts the dataflow. Deleting that whole block from the `RemoveNode`
/// handler left every other test in this PR green.
///
/// The dataflow here is two independent source nodes: `quick` connects
/// immediately and parks on the barrier, `slow` sleeps well past the
/// removal before it would connect, so it is provably still pending
/// when it is removed. That removal is the only thing that can complete
/// the cohort — the removed process's own exit cannot, since its id is
/// gone from `local_nodes` by then — so if the barrier is not
/// re-evaluated, `quick` stays parked forever and never reports
/// `Running`.
#[test]
#[cfg(unix)]
fn removing_an_unsubscribed_node_releases_the_startup_barrier() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    ensure_cli_built();
    ensure_stop_delay_built();
    let dora = dora_bin();
    cleanup_stale(&dora);
    let _cleanup = CleanupGuard { dora: &dora };

    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let node_bin = format!("{manifest_dir}/target/debug/stop-delay-node");
    let target = Path::new(manifest_dir).join("target");
    let stem = format!("dora-barrier-release-{}", std::process::id());
    let dataflow = target.join(format!("{stem}.yml"));
    // `quick` writes this once its `Node()` init returns. That is the
    // only observable for the barrier having released: `dora node list`
    // reports `Running` for any spawned process, including one still
    // parked inside `Node()`.
    let ready = target.join(format!("{stem}.ready"));
    let _ = fs::remove_file(&ready);
    fs::write(
        &dataflow,
        format!(
            "nodes:\n  \
             - id: quick\n    \
               path: {node_bin}\n    \
               env:\n      \
                 DORA_TEST_READY_FILE: {ready}\n    \
               outputs:\n      - value\n  \
             - id: slow\n    \
               path: {node_bin}\n    \
               env:\n      \
                 DORA_TEST_INIT_DELAY_MS: 60000\n    \
               outputs:\n      - value\n",
            ready = ready.display()
        ),
    )
    .expect("failed to write barrier-release dataflow");

    let (ok, _, stderr) = run_capture(Command::new(&dora).arg("up"), "dora up");
    assert!(ok, "dora up failed.\nstderr:\n{stderr}");
    let name = "rustlc-barrier";
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args([
            "start",
            dataflow.to_str().unwrap(),
            "--detach",
            "--name",
            name,
        ]),
        "dora start",
    );
    assert!(ok, "dora start failed.\nstderr:\n{stderr}");

    // `slow` is sleeping for a full minute before it would connect, so
    // it is unambiguously still pending here — no timing guess.
    let (ok, _, stderr) = run_capture(
        Command::new(&dora).args(["node", "remove", "--dataflow", name, "slow"]),
        "dora node remove slow",
    );
    assert!(ok, "dora node remove failed.\nstderr:\n{stderr}");

    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    while !ready.exists() {
        assert!(
            std::time::Instant::now() < deadline,
            "removing the last pending node did not release the barrier: `quick` never \
             got past its `Node()` init, so the dataflow never started. (`dora node list` \
             would show it `Running` regardless — that is why this waits on {ready:?}.)",
            ready = ready
        );
        std::thread::sleep(Duration::from_millis(200));
    }
}

#[test]
// C++ fixture is deliberately Unix-only: the existing cmake-dataflow
// example in this repo explicitly skips Windows
// (`examples/cmake-dataflow/run.rs` bails on `cfg!(windows)` with
// "The c++ example does not work on Windows currently because of a
// linker error"). Honoring that ground truth keeps this test honest;
// a Windows C++ fixture would need separate linker work tracked
// outside #1703's scope.
#[cfg(not(windows))]
fn lifecycle_cxx_dynamic_add_remove() {
    let manifest_dir = env!("CARGO_MANIFEST_DIR");
    let fixture_dir = Path::new(manifest_dir).join("examples/cxx-dynamic-add-remove");
    let dataflow = fixture_dir.join("dataflow.yml");
    let filter_yml = fixture_dir.join("filter-node.yml");
    // C++ binaries aren't built by `dora build` — drive cmake before
    // the dataflow lifecycle starts. The fixture's `DoraTargets.cmake`
    // runs `cargo build -p dora-node-api-c` internally so we don't
    // double-build that library.
    ensure_cxx_built(&fixture_dir);
    run_lifecycle(LifecycleFixture {
        dataflow_path: &dataflow,
        filter_yml_path: &filter_yml,
        name: "cxxlc",
        sender_path_marker: "build/cxx_sender",
        use_uv: false,
    });
}

/// dora-rs/dora#2920: SIGTERM to `dora run` must actually tear the
/// dataflow down and let the CLI exit, even when the node cooperates
/// with nothing.
///
/// The teardown chain is: signal -> the ctrl-c ladder -> `stop_all` ->
/// `Stop`, then SIGTERM after the grace period, then SIGKILL after
/// another half-grace. Each node is a process-group leader, so a kill is
/// a `killpg` over its whole group, and the daemon's per-node
/// `child.wait()` reaps that group before reporting the node finished.
///
/// Because `run_dataflow` returns only once every node has reported, the
/// CLI cannot exit while a node is still alive. That makes the realistic
/// failure mode a WEDGE rather than an orphan: if teardown breaks, the
/// run never returns, and this test fails on its exit deadline.
///
/// Note on what a single-rung mutation proves here, which is less than
/// it looks: `stop_all` moves the `ProcessHandle`s into the ladder task
/// (`n.process.take()`), so when that task ends they drop and
/// `ProcessHandle::drop` submits one more `Kill`. That is a second kill
/// in the SAME task, not an independent path — disabling only the
/// ladder's SIGKILL rung still leaves the drop-kill, and the test
/// passes. Disabling both is what fails it.
///
/// The fixture ignores BOTH the cooperative `Stop` and SIGTERM, so only
/// a SIGKILL can end it; a node that died to SIGTERM would be cleaned up
/// either way and could not tell a working ladder from a broken one.
///
/// Unix-only: it signals the CLI and inspects process state.
#[test]
#[cfg(unix)]
fn run_killed_by_sigterm_terminates_nodes_and_exits() {
    // Not for port 6013 (`dora run` binds none) but for CPU: this test
    // builds a fixture and then sits through a 15s teardown, and the
    // liveness windows in sibling tests are tuned for an unloaded runner.
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());

    let mut run = StubbornRun::start("dora-2920-orphan", false);
    let node_pid = run.wait_for_node();

    let signalled = Command::new("kill")
        .args(["-TERM", &run.cli.id().to_string()])
        .status()
        .expect("failed to run `kill`");
    // Checked: an undelivered signal would make the wait below time out
    // and report a teardown wedge that never actually happened.
    assert!(signalled.success(), "failed to SIGTERM `dora run`");

    // The default ladder hard-kills at grace + grace/2 = 15s; measured
    // end-to-end teardown is ~18s. 60s leaves headroom for a cold runner
    // while still bounding the ladder, so a regression that merely
    // inflates teardown is caught rather than passing silently.
    let deadline = std::time::Instant::now() + Duration::from_secs(60);
    let status = loop {
        match run.cli.try_wait().expect("try_wait failed") {
            Some(status) => {
                run.cli_reaped = true;
                break status;
            }
            None if std::time::Instant::now() >= deadline => panic!(
                "`dora run` did not exit within 60s of SIGTERM — teardown \
                 wedged, so the run never returns (#2920)\nstderr tail:\n{}",
                run.stderr_tail()
            ),
            None => std::thread::sleep(Duration::from_millis(250)),
        }
    };

    // Identity-checked: a bare pid could have been recycled between the
    // node's death and this check, which would both report a false orphan
    // and SIGKILL an unrelated process.
    assert!(
        !node_is_fixture(node_pid),
        "node {node_pid} survived `dora run` exiting with {status}: the stop \
         ladder never reached its SIGKILL rung, so killing the CLI strands \
         nodes (#2920)"
    );
}

/// dora-rs/dora#2856: `SIGKILL` to `dora run` must not strand its nodes.
///
/// The sibling test above covers the signals the CLI can still *handle*.
/// `SIGKILL` is the case where it cannot: nothing of ours runs after it, so
/// the stop ladder, the destroy reaper and every other daemon-side teardown
/// path are unavailable by construction. And because nodes are spawned as
/// process-group leaders — deliberately, so a terminal `Ctrl-C` cannot kill
/// them out from under the daemon — the orphan ends up with `ppid 1` in a
/// group of its own, which neither inherited signal delivery nor a group-kill
/// of the CLI reaches.
///
/// The containment therefore has to live in the node: `dora run` tells each
/// node it spawns which pid it may not outlive (`DORA_RUN_PARENT_PID`), and
/// the node's guard `SIGKILL`s its own process group once that pid is gone.
///
/// The fixture is the point of the test. It sets `SIGTERM` to `SIG_IGN` and
/// never polls its event stream, so it survives every cooperative path AND a
/// containment scheme that only manages to deliver `SIGTERM`. A node that died
/// to either would pass this test without the fix.
///
/// Unix-only: it signals the CLI and inspects process state.
#[test]
#[cfg(unix)]
fn run_killed_by_sigkill_does_not_orphan_nodes() {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());

    // Its own process group, so that nothing this test does -- and nothing the
    // daemon or a node does on the way down -- can be delivered to the test
    // harness's own group by accident.
    let mut run = StubbornRun::start("dora-2856-orphan", true);
    let node_pid = run.wait_for_node();

    let killed = Command::new("kill")
        .args(["-KILL", &run.cli.id().to_string()])
        .status()
        .expect("failed to run `kill`");
    // Checked: an undelivered signal would make the wait below time out and
    // report an orphan that was never actually orphaned.
    assert!(killed.success(), "failed to SIGKILL `dora run`");
    // Reaped immediately: the CLI cannot run teardown code after SIGKILL, so
    // there is nothing to wait for, and leaving it unreaped would keep a
    // zombie around for the rest of the suite.
    let status = run.cli.wait().expect("failed to reap `dora run`");
    run.cli_reaped = true;

    // The guard re-checks the parent twice a second, so a working containment
    // lands in about a second. 30s is slack for a loaded runner while still
    // bounding it: an orphan is forever, not slow.
    let deadline = std::time::Instant::now() + Duration::from_secs(30);
    while node_is_fixture(node_pid) {
        assert!(
            std::time::Instant::now() < deadline,
            "node {node_pid} outlived `dora run` ({status}) by 30s: killing the \
             CLI outright strands its nodes (#2856)\nstderr tail:\n{}",
            run.stderr_tail()
        );
        std::thread::sleep(Duration::from_millis(250));
    }
}

/// A `dora run` of the stubborn fixture, plus the RAII teardown both orphan
/// tests need.
///
/// The fixture ignores `SIGTERM` and never polls its event stream, so it
/// outlives a failing test by design: a bail-out that only killed the CLI
/// would strand it on the runner for the rest of the suite. Every panic path
/// therefore has to tear down the CLI *and* the node, which is why this is a
/// `Drop` type rather than cleanup code at the end of each test.
#[cfg(unix)]
struct StubbornRun {
    cli: std::process::Child,
    /// Set once `try_wait`/`wait` has reaped the CLI. After that its pid is
    /// free for reuse and must never be signalled again.
    cli_reaped: bool,
    node_pid: Option<u32>,
    /// The CLI's captured stderr. The failures these tests exist to catch
    /// happen on remote runners, where this is the only forensic artifact.
    log: std::path::PathBuf,
    /// Where the fixture publishes its own pid, so the tests assert on THAT
    /// process rather than pattern-matching the process table (which would
    /// also catch a leftover from an earlier run).
    pid_file: std::path::PathBuf,
    files: Vec<std::path::PathBuf>,
}

#[cfg(unix)]
impl Drop for StubbornRun {
    fn drop(&mut self) {
        // Signal the CLI only while it is UNREAPED: an unreaped child is a
        // zombie at worst, so the kernel still holds its pid and it cannot have
        // been recycled. Once reaped the pid is free, and a blind `kill` here
        // would eventually hit a stranger.
        if !self.cli_reaped {
            let _ = Command::new("kill")
                .args(["-KILL", &self.cli.id().to_string()])
                .status();
        }
        // The node is the daemon's child, never ours, so we can never reap it
        // and cannot lean on the rule above. Confirm identity instead -- and
        // note the target is a process GROUP, so a recycled pgid would take out
        // an unrelated group, not just a stray process.
        if let Some(pid) = self.node_pid
            && still_our_fixture(pid)
        {
            let _ = Command::new("kill")
                .args(["-KILL", &format!("-{pid}")])
                .status();
        }
        for f in &self.files {
            let _ = fs::remove_file(f);
        }
    }
}

#[cfg(unix)]
impl StubbornRun {
    /// Build the fixture, write a one-node dataflow for it, and start
    /// `dora run` on it. `own_process_group` puts the CLI in a group of its
    /// own.
    ///
    /// `--target-dir` is pinned for the same reason as the sibling helpers: the
    /// descriptor hard-codes the workspace-relative binary path, so a
    /// `CARGO_TARGET_DIR` override would build the fixture somewhere the daemon
    /// will not look, and the failure would surface later as a misleading "node
    /// never reported its pid".
    fn start(prefix: &str, own_process_group: bool) -> Self {
        use std::os::unix::process::CommandExt as _;

        ensure_cli_built();
        let dora = dora_bin();
        let target = Path::new(env!("CARGO_MANIFEST_DIR")).join("target");
        let built = Command::new("cargo")
            .args(["build", "-p", "sigterm-ignoring-node"])
            .arg("--target-dir")
            .arg(&target)
            .status()
            .expect("failed to run cargo build for sigterm-ignoring-node");
        assert!(built.success(), "failed to build sigterm-ignoring-node");

        // Sweep artifacts from earlier runs: a stale pid file would be read as
        // this run's node, and the test would assert against a process that
        // exited long ago.
        if let Ok(entries) = fs::read_dir(&target) {
            for entry in entries.flatten() {
                if entry.file_name().to_string_lossy().starts_with(prefix) {
                    let _ = fs::remove_file(entry.path());
                }
            }
        }
        let scratch = |ext: &str| target.join(format!("{prefix}-{}.{ext}", std::process::id()));
        let (yaml, pid_file, log) = (scratch("yml"), scratch("pid"), scratch("log"));
        fs::write(
            &yaml,
            format!(
                "nodes:\n  \
                 - id: stubborn\n    \
                   path: \"{node}\"\n    \
                   env:\n      \
                     DORA_TEST_PID_FILE: \"{pid_file}\"\n    \
                   inputs:\n      \
                     tick: dora/timer/millis/100\n",
                node = target.join("debug/sigterm-ignoring-node").display(),
                pid_file = pid_file.display(),
            ),
        )
        .expect("failed to write dataflow yaml");

        let log_file = fs::File::create(&log).expect("failed to create log file");
        let mut command = Command::new(&dora);
        command
            .args(["run", yaml.to_str().unwrap()])
            .stdout(Stdio::null())
            .stderr(Stdio::from(log_file));
        if own_process_group {
            command.process_group(0);
        }
        let cli = command.spawn().expect("failed to spawn dora run");

        Self {
            cli,
            cli_reaped: false,
            node_pid: None,
            files: vec![yaml, pid_file.clone(), log.clone()],
            log,
            pid_file,
        }
    }

    fn stderr_tail(&self) -> String {
        fs::read_to_string(&self.log)
            .map(|s| s.lines().rev().take(20).collect::<Vec<_>>().join("\n"))
            .unwrap_or_else(|e| format!("<could not read {}: {e}>", self.log.display()))
    }

    /// Block until the node reports its pid, and record it for teardown.
    ///
    /// Both tests must know the node is actually up before they signal;
    /// otherwise a teardown that ran before it ever spawned would "pass"
    /// having proved nothing.
    fn wait_for_node(&mut self) -> u32 {
        let deadline = std::time::Instant::now() + Duration::from_secs(60);
        let node_pid = loop {
            // If the CLI already died, say so with its status and stderr rather
            // than blaming the pid file 60s from now.
            if let Some(status) = self.cli.try_wait().expect("try_wait failed") {
                self.cli_reaped = true;
                panic!(
                    "`dora run` exited early with {status} before the node reported \
                     its pid.\nstderr tail:\n{}",
                    self.stderr_tail()
                );
            }
            if let Ok(contents) = fs::read_to_string(&self.pid_file)
                && let Ok(pid) = contents.trim().parse::<u32>()
            {
                break pid;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "node never reported its pid to {}\nstderr tail:\n{}",
                self.pid_file.display(),
                self.stderr_tail()
            );
            std::thread::sleep(Duration::from_millis(200));
        };
        self.node_pid = Some(node_pid);
        assert!(
            node_is_fixture(node_pid),
            "precondition: node {node_pid} should be running our fixture before we signal"
        );
        node_pid
    }
}

/// Whether `pid` is alive AND is our fixture, so a recycled pid is not
/// mistaken for a surviving node.
#[cfg(unix)]
fn node_is_fixture(pid: u32) -> bool {
    // `expect`, not a silent false: this feeds the assertions, and the
    // final one is `!node_is_fixture(..)`. If a failure to RUN `ps` were
    // folded into "not our process", an unavailable `ps` would satisfy
    // that assertion and the test would report success having verified
    // nothing.
    let args = ps_args(pid).expect("failed to run `ps -o args=`");
    process_alive(&pid.to_string()) && args.contains(FIXTURE_BIN)
}

/// Matched with a leading `/` so it hits the spawned binary's path and not
/// a process that merely *names* it in its arguments — `cargo build -p
/// sigterm-ignoring-node`, which this test itself runs.
const FIXTURE_BIN: &str = "/sigterm-ignoring-node";

/// The full command line of `pid`. `None` if `ps` could not be run at all;
/// `Some("")` if it ran and the pid is gone.
///
/// `args=`, NOT `comm=`: on Linux `comm` is `/proc/<pid>/comm`, which the
/// kernel caps at `TASK_COMM_LEN - 1` = 15 characters. `sigterm-ignoring-node`
/// is 21, so a `comm` match silently fails there while passing on macOS,
/// where `comm` carries the full path — which is exactly how this shipped
/// green from a Mac and turned CI red (dora-rs/dora#2961).
///
/// The two states are kept apart so callers can choose: assertions must
/// fail loudly when the check itself is broken, while `Drop` must not.
#[cfg(unix)]
fn ps_args(pid: u32) -> Option<String> {
    Command::new("ps")
        .args(["-ww", "-o", "args=", "-p", &pid.to_string()])
        .output()
        .ok()
        .map(|out| String::from_utf8_lossy(&out.stdout).into_owned())
}

/// Same question, but safe to ask from a `Drop`: never panics.
///
/// `node_is_fixture` and `process_alive` both `expect` on `ps`. Calling
/// either while unwinding from a failed assertion would panic inside a
/// panic and abort the process, replacing the real diagnostic with
/// nothing. Returning false on any doubt is the safe default -- it skips
/// a kill rather than risking one against the wrong target.
#[cfg(unix)]
fn still_our_fixture(pid: u32) -> bool {
    ps_args(pid).is_some_and(|args| args.contains(FIXTURE_BIN))
}
