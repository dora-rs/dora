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

static BUILD_RUST_FILTER: Once = Once::new();

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

fn run_lifecycle(fixture: LifecycleFixture<'_>) {
    let _guard = LIFECYCLE_LOCK.lock().unwrap_or_else(|p| p.into_inner());
    ensure_cli_built();
    let dora = dora_bin();
    let dataflow = fixture.dataflow_path;
    let filter_yml = fixture.filter_yml_path;
    let name = fixture.name;

    cleanup_stale(&dora);
    // Arm panic-safe teardown BEFORE `dora up` so any assertion
    // between here and the normal end of the function still leaves
    // the cluster torn down.
    let _cleanup = CleanupGuard { dora: &dora };

    // `dora up` starts coordinator + daemon on port 6013.
    let (ok, _, stderr) = run_capture(Command::new(&dora).arg("up"), "dora up");
    assert!(ok, "dora up failed.\nstderr:\n{stderr}");

    // `dora build` — provisions the Python venv (Python) or runs
    // the per-node `build:` cargo commands (Rust).
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

    // Make sure both base nodes are Running before exercising the matrix.
    let list_out = wait_for_list(&dora, name, Duration::from_secs(30), |m| {
        m.get("sender").is_some_and(|(s, _, _)| s == "Running")
            && m.get("receiver").is_some_and(|(s, _, _)| s == "Running")
    });
    // Tight assertion: `wait_for_list` returns the last stdout on
    // timeout regardless of whether the predicate held, so check the
    // actual status here rather than just node existence — otherwise a
    // Failed or never-spawned base node would pass this checkpoint and
    // surface a misleading failure several subcommands later.
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
    let sender_initial_pid = sender_state.unwrap().1;
    let receiver_initial_pid = receiver_state.unwrap().1;

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
// Python variant is Unix-only because it uses `--uv` for the Python
// venv and the CircleCI Windows lane (`.circleci/config.yml`
// test-windows job) does NOT install uv. Linux + macOS CI install it
// explicitly. If a Windows uv path lands later (e.g. via
// `winget install astral-sh.uv` in the runner setup), this gate can
// be relaxed.
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
