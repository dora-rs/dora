//! End-to-end regression test for dora-rs/dora#2029.
//!
//! The daemon's coordinator-reconnect path used to drop the whole `Daemon`
//! on a heartbeat timeout / coordinator-send failure, which killed every
//! running node (each `ProcessHandle` kills its process on drop). That
//! silently defeated the transparent-reconnect design of #1996/#1998.
//!
//! This test drives the real CLI stack: it starts a coordinator and a daemon
//! as separate processes, launches a long-running node, then SIGKILLs the
//! coordinator and restarts it on the same port (a persistent redb store lets
//! the new coordinator reload the dataflow record). It asserts that across the
//! reconnect the node process is **the same pid** (never killed + respawned)
//! and **still alive**, and that the coordinator re-adopts the dataflow.
//!
//! Heavyweight (spawns processes, ~30-60s). It lives in the `dora-examples`
//! crate, which the fast PR `cargo test --all` step excludes; run it
//! explicitly with `cargo test -p dora-examples --test daemon-reconnect-e2e`
//! (wired into the nightly daemon-reconnect job).

#![cfg(unix)]

use std::net::{SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Once;
use std::time::{Duration, Instant};

use dora_cli::WsSession;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use uuid::Uuid;

static BUILD: Once = Once::new();

fn ensure_built() {
    BUILD.call_once(|| {
        let status = Command::new("cargo")
            .args(["build", "-p", "dora-cli", "-p", "reconnect-survivor-node"])
            .status()
            .expect("failed to run cargo build");
        assert!(status.success(), "failed to build test prerequisites");
    });
}

fn target_dir() -> PathBuf {
    let manifest = env!("CARGO_MANIFEST_DIR");
    std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| Path::new(manifest).join("target"))
}

fn bin(name: &str) -> PathBuf {
    let exe = format!("{name}{}", std::env::consts::EXE_SUFFIX);
    let path = target_dir().join("debug").join(exe);
    assert!(
        path.exists(),
        "binary not found: {} (did ensure_built run?)",
        path.display()
    );
    path
}

/// Grab a currently-free TCP port. There is a small TOCTOU window between
/// closing this listener and the coordinator binding it, but it is more than
/// adequate to keep parallel test binaries off each other's ports.
fn free_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
        .expect("failed to bind ephemeral port")
        .local_addr()
        .expect("failed to read local addr")
        .port()
}

fn port_open(port: u16) -> bool {
    TcpStream::connect(("127.0.0.1", port)).is_ok()
}

/// `kill -0`: succeeds iff the process exists (and we may signal it).
fn pid_alive(pid: u32) -> bool {
    Command::new("kill")
        .arg("-0")
        .arg(pid.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
}

fn read_pids(path: &Path) -> Vec<u32> {
    std::fs::read_to_string(path)
        .unwrap_or_default()
        .lines()
        .filter_map(|l| l.trim().parse().ok())
        .collect()
}

/// Send `signal` (e.g. `-STOP`, `-CONT`) to `pid` via `kill(1)`.
fn signal(pid: u32, sig: &str) {
    let _ = Command::new("kill")
        .arg(sig)
        .arg(pid.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}

fn log_contains(path: &Path, needle: &str) -> bool {
    std::fs::read_to_string(path)
        .map(|s| s.contains(needle))
        .unwrap_or(false)
}

fn wait_until(mut f: impl FnMut() -> bool, timeout: Duration, what: &str) {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if f() {
            return;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    panic!("timed out after {timeout:?} waiting for: {what}");
}

fn spawn_coordinator(dora: &Path, port: u16, redb: &Path, log: &Path) -> Child {
    // Append so the restarted coordinator's log accumulates alongside the
    // first one's — the reconcile assertion reads the second instance's output.
    let out = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(log)
        .expect("open coord log");
    let err = out.try_clone().expect("clone coord log");
    Command::new(dora)
        .arg("coordinator")
        .arg("--port")
        .arg(port.to_string())
        .arg("--store")
        .arg(format!("redb:{}", redb.display()))
        .stdout(Stdio::from(out))
        .stderr(Stdio::from(err))
        .spawn()
        .expect("failed to spawn coordinator")
}

/// Query the coordinator for the set of `Running` dataflow uuids. Returns an
/// error if the coordinator is unreachable (used while polling for reconnect).
fn list_active(port: u16) -> eyre::Result<Vec<Uuid>> {
    let addr: SocketAddr = format!("127.0.0.1:{port}").parse().unwrap();
    let session = WsSession::connect(addr)?;
    let data = serde_json::to_vec(&ControlRequest::List)?;
    let reply_bytes = session.request(&data)?;
    let reply: ControlRequestReply = serde_json::from_slice(&reply_bytes)?;
    match reply {
        ControlRequestReply::DataflowList(list) => {
            Ok(list.get_active().into_iter().map(|d| d.uuid).collect())
        }
        other => eyre::bail!("unexpected list reply: {other:?}"),
    }
}

fn dump_logs(coord_log: &Path, daemon_log: &Path) {
    for (label, path) in [("coordinator", coord_log), ("daemon", daemon_log)] {
        eprintln!("--- {label} log ({}) ---", path.display());
        eprintln!("{}", std::fs::read_to_string(path).unwrap_or_default());
    }
}

/// Best-effort teardown that always runs, even on panic.
struct Cleanup {
    coordinator: Option<Child>,
    daemon: Option<Child>,
}

impl Drop for Cleanup {
    fn drop(&mut self) {
        for child in [self.daemon.as_mut(), self.coordinator.as_mut()]
            .into_iter()
            .flatten()
        {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// A node survives a coordinator drop+restore: same pid, still alive, dataflow
/// re-adopted. Regression test for dora-rs/dora#2029.
#[test]
fn node_survives_coordinator_reconnect() {
    ensure_built();

    let dora = bin("dora");
    let node = bin("reconnect-survivor-node");

    let tmp = tempfile::tempdir().expect("create tempdir");
    let pid_file = tmp.path().join("survivor.pid");
    let redb = tmp.path().join("coordinator.redb");
    let coord_log = tmp.path().join("coordinator.log");
    let daemon_log = tmp.path().join("daemon.log");
    let dataflow_yml = tmp.path().join("dataflow.yml");

    std::fs::write(
        &dataflow_yml,
        format!(
            "nodes:\n  \
             - id: survivor\n    \
             path: {node}\n    \
             inputs:\n      \
             tick: dora/timer/millis/300\n    \
             env:\n      \
             DORA_TEST_PID_FILE: {pid}\n",
            node = node.display(),
            pid = pid_file.display(),
        ),
    )
    .expect("write dataflow yml");

    let port = free_port();
    let daemon_listen_port = free_port();

    let mut cleanup = Cleanup {
        coordinator: None,
        daemon: None,
    };

    // 1. Start coordinator + daemon.
    cleanup.coordinator = Some(spawn_coordinator(&dora, port, &redb, &coord_log));
    wait_until(
        || port_open(port),
        Duration::from_secs(20),
        "coordinator to accept connections",
    );

    let daemon_out = std::fs::File::create(&daemon_log).expect("create daemon log");
    let daemon_err = daemon_out.try_clone().expect("clone daemon log");
    cleanup.daemon = Some(
        // Intentionally an *unnamed* daemon: nodes without an explicit `deploy`
        // placement are only scheduled onto unnamed daemons, so `dora start`
        // would fail ("no unnamed daemon connections") against a named one.
        Command::new(&dora)
            .arg("daemon")
            .arg("--coordinator-port")
            .arg(port.to_string())
            .arg("--local-listen-port")
            .arg(daemon_listen_port.to_string())
            .stdout(Stdio::from(daemon_out))
            .stderr(Stdio::from(daemon_err))
            .spawn()
            .expect("failed to spawn daemon"),
    );

    // 2. Start the dataflow and wait for the node to record its pid.
    let start = Command::new(&dora)
        .arg("start")
        .arg(&dataflow_yml)
        .arg("--detach")
        .arg("--coordinator-port")
        .arg(port.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .expect("failed to run dora start");
    if !start.success() {
        dump_logs(&coord_log, &daemon_log);
        panic!("dora start failed");
    }

    wait_until(
        || read_pids(&pid_file).len() == 1,
        Duration::from_secs(45),
        "survivor node to record its pid",
    );
    let pids = read_pids(&pid_file);
    let node_pid = pids[0];
    assert!(
        pid_alive(node_pid),
        "node pid {node_pid} should be alive after start"
    );

    // Baseline: the dataflow is Running and uniquely identifies our node.
    wait_until(
        || list_active(port).map(|v| v.len() == 1).unwrap_or(false),
        Duration::from_secs(30),
        "dataflow to register as Running before the reconnect",
    );
    let dataflow_id = list_active(port).expect("list active")[0];

    // 3. Kill the coordinator (simulate a crash) and confirm it is gone.
    {
        let coord = cleanup.coordinator.as_mut().unwrap();
        coord.kill().expect("failed to kill coordinator");
        coord.wait().expect("failed to reap coordinator");
    }
    cleanup.coordinator = None;
    wait_until(
        || !port_open(port),
        Duration::from_secs(10),
        "coordinator port to close",
    );

    // 4. Keep the coordinator down long enough for the daemon to notice the
    //    disconnect (heartbeat send fails within ~5s) and enter its reconnect
    //    loop. The node must stay alive throughout — it is a separate process
    //    driven by the daemon's local timers, independent of the coordinator.
    std::thread::sleep(Duration::from_secs(12));
    assert!(
        pid_alive(node_pid),
        "node pid {node_pid} must stay alive while the coordinator is down"
    );
    assert_eq!(
        read_pids(&pid_file),
        vec![node_pid],
        "node must not be killed + respawned while the coordinator is down"
    );

    // 5. Restart the coordinator on the same port; the daemon reconnects within
    //    its 90s reconnect window (#1996) and re-reports the running dataflow.
    cleanup.coordinator = Some(spawn_coordinator(&dora, port, &redb, &coord_log));
    wait_until(
        || port_open(port),
        Duration::from_secs(20),
        "coordinator to restart",
    );

    // Re-adoption: when the reconnected daemon re-reports the running dataflow,
    // the coordinator must put it back into its LIVE state so it is visible and
    // manageable again — `dora list` shows it `Running` (the #2029 P1 fix), not
    // merely flip the persisted store record. Poll `dora list` for it.
    let readopted = || {
        list_active(port)
            .map(|v| v.contains(&dataflow_id))
            .unwrap_or(false)
    };
    let deadline = Instant::now() + Duration::from_secs(60);
    while Instant::now() < deadline && !readopted() {
        std::thread::sleep(Duration::from_millis(500));
    }

    // 6. Core assertions for #2029: the node is the SAME process (never
    //    killed + respawned) and is still alive after the reconnect.
    let pids_after = read_pids(&pid_file);
    if pids_after != vec![node_pid] || !pid_alive(node_pid) {
        dump_logs(&coord_log, &daemon_log);
    }
    assert_eq!(
        pids_after,
        vec![node_pid],
        "node must survive the reconnect as the same pid (no kill + respawn)"
    );
    assert!(
        pid_alive(node_pid),
        "node pid {node_pid} must still be alive after the reconnect"
    );

    // 7. The coordinator must re-adopt the dataflow into its live state so it is
    //    manageable again (visible in `dora list`), not just left in the store.
    if !readopted() {
        dump_logs(&coord_log, &daemon_log);
        panic!(
            "coordinator did not re-adopt dataflow {dataflow_id} into `dora list` after reconnect"
        );
    }

    // Teardown: stop the dataflow before Cleanup kills the processes.
    let _ = Command::new(&dora)
        .arg("stop")
        .arg("--all")
        .arg("--coordinator-port")
        .arg(port.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}

/// Companion to the coordinator-restart case: here the **coordinator stays up**
/// and the daemon is frozen (SIGSTOP) past the coordinator's 30s heartbeat
/// watchdog, then resumed. This exercises the #2028 disconnect path, which #2060
/// changes from a terminal teardown to a reclaim: the coordinator marks the
/// dataflow `Recovering` (not `Failed`+archived), and when the daemon thaws and
/// reconnects with its nodes still running, the coordinator reconciles it back
/// to `Running` instead of orphaning it. Regression for dora-rs/dora#2029 + the
/// #2028 reclaim adjustment.
#[test]
fn node_survives_daemon_watchdog_disconnect() {
    ensure_built();

    let dora = bin("dora");
    let node = bin("reconnect-survivor-node");

    let tmp = tempfile::tempdir().expect("create tempdir");
    let pid_file = tmp.path().join("survivor.pid");
    let redb = tmp.path().join("coordinator.redb");
    let coord_log = tmp.path().join("coordinator.log");
    let daemon_log = tmp.path().join("daemon.log");
    let dataflow_yml = tmp.path().join("dataflow.yml");

    std::fs::write(
        &dataflow_yml,
        format!(
            "nodes:\n  \
             - id: survivor\n    \
             path: {node}\n    \
             inputs:\n      \
             tick: dora/timer/millis/300\n    \
             env:\n      \
             DORA_TEST_PID_FILE: {pid}\n",
            node = node.display(),
            pid = pid_file.display(),
        ),
    )
    .expect("write dataflow yml");

    let port = free_port();
    let daemon_listen_port = free_port();

    let mut cleanup = Cleanup {
        coordinator: None,
        daemon: None,
    };

    cleanup.coordinator = Some(spawn_coordinator(&dora, port, &redb, &coord_log));
    wait_until(
        || port_open(port),
        Duration::from_secs(20),
        "coordinator to accept connections",
    );

    let daemon_out = std::fs::File::create(&daemon_log).expect("create daemon log");
    let daemon_err = daemon_out.try_clone().expect("clone daemon log");
    cleanup.daemon = Some(
        Command::new(&dora)
            .arg("daemon")
            .arg("--coordinator-port")
            .arg(port.to_string())
            .arg("--local-listen-port")
            .arg(daemon_listen_port.to_string())
            .stdout(Stdio::from(daemon_out))
            .stderr(Stdio::from(daemon_err))
            .spawn()
            .expect("failed to spawn daemon"),
    );
    let daemon_pid = cleanup.daemon.as_ref().unwrap().id();

    let start = Command::new(&dora)
        .arg("start")
        .arg(&dataflow_yml)
        .arg("--detach")
        .arg("--coordinator-port")
        .arg(port.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .expect("failed to run dora start");
    if !start.success() {
        dump_logs(&coord_log, &daemon_log);
        panic!("dora start failed");
    }

    wait_until(
        || read_pids(&pid_file).len() == 1,
        Duration::from_secs(45),
        "survivor node to record its pid",
    );
    let node_pid = read_pids(&pid_file)[0];
    wait_until(
        || list_active(port).map(|v| v.len() == 1).unwrap_or(false),
        Duration::from_secs(30),
        "dataflow to register as Running",
    );
    let dataflow_id = list_active(port).expect("list active")[0];

    // Freeze the daemon so it misses heartbeats; the coordinator's watchdog
    // (30s) disconnects it. The node is a separate process and keeps running.
    signal(daemon_pid, "-STOP");

    wait_until(
        || log_contains(&coord_log, "failed watchdog"),
        Duration::from_secs(50),
        "coordinator watchdog to disconnect the frozen daemon",
    );
    // The disconnect must open a reclaim window, NOT terminally fail the dataflow.
    if !log_contains(&coord_log, "entering reclaim window") {
        signal(daemon_pid, "-CONT");
        dump_logs(&coord_log, &daemon_log);
        panic!("coordinator did not enter the reclaim window on daemon disconnect");
    }
    assert!(
        pid_alive(node_pid),
        "node pid {node_pid} must stay alive while the daemon is frozen"
    );
    assert_eq!(
        read_pids(&pid_file),
        vec![node_pid],
        "node must not be killed + respawned while the daemon is frozen"
    );

    // Thaw the daemon; it reconnects and re-reports the running dataflow. The
    // coordinator must reclaim it back into its live state — visible + manageable
    // in `dora list` again (#2029 P1), not merely flipped in the store.
    signal(daemon_pid, "-CONT");

    wait_until(
        || {
            list_active(port)
                .map(|v| v.contains(&dataflow_id))
                .unwrap_or(false)
        },
        Duration::from_secs(60),
        "coordinator to re-adopt the dataflow into `dora list` after reconnect",
    );

    let pids_after = read_pids(&pid_file);
    if pids_after != vec![node_pid] || !pid_alive(node_pid) {
        dump_logs(&coord_log, &daemon_log);
    }
    assert_eq!(
        pids_after,
        vec![node_pid],
        "node must survive the daemon freeze as the same pid (no kill + respawn)"
    );
    assert!(
        pid_alive(node_pid),
        "node pid {node_pid} must still be alive after the daemon reconnects"
    );

    let _ = Command::new(&dora)
        .arg("stop")
        .arg("--all")
        .arg("--coordinator-port")
        .arg(port.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}
