//! End-to-end regression test for the coordinator `Stop` vs. `Restart`
//! race.
//!
//! `initiate_restart` sends `StopDataflow` to the daemon and registers a
//! `PendingRestart` for the dataflow's UUID, but leaves the UUID in
//! `running_dataflows` until the daemon reports `DataflowFinishedOnDaemon`.
//! A `Stop`/`StopByName` request that lands in that window used to fall
//! through to `stop_dataflow` and "succeed" against the old UUID, while the
//! restart went on to spawn a fresh incarnation under a new UUID that kept
//! running — the caller saw a clean success with no indication the
//! dataflow was still alive. The fix rejects `Stop`/`StopByName` outright
//! while `pending_restarts` still holds the UUID.
//!
//! The window between `StopDataflow` being sent and `DataflowFinishedOnDaemon`
//! arriving is normally just tens of milliseconds (as long as the node takes
//! to exit), which is too timing-dependent for a deterministic test. This
//! test widens it on purpose using the shared `stop-delay-node` fixture's
//! `DORA_TEST_STOP_DELAY_MS` knob, so `stop` reliably lands while the
//! restart is still pending instead of only doing so most of the time.
//!
//! Heavyweight (spawns processes). Run with
//! `cargo test -p dora-examples --test stop-restart-race-e2e`.

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
            .args(["build", "-p", "dora-cli", "-p", "stop-delay-node"])
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
/// closing this listener and the coordinator binding it, but it is more
/// than adequate to keep parallel test binaries off each other's ports.
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
    let out = std::fs::File::create(log).expect("create coordinator log");
    let err = out.try_clone().expect("clone coordinator log");
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

/// Query the coordinator for the set of `Running` dataflow uuids.
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

/// A `Stop` racing a `Restart` on the same dataflow must be rejected, not
/// silently reported as successful while a new incarnation keeps running.
#[test]
fn stop_racing_pending_restart_is_rejected() {
    ensure_built();

    let dora = bin("dora");
    let node = bin("stop-delay-node");

    let tmp = tempfile::tempdir().expect("create tempdir");
    let redb = tmp.path().join("coordinator.redb");
    let coord_log = tmp.path().join("coordinator.log");
    let daemon_log = tmp.path().join("daemon.log");
    let dataflow_yml = tmp.path().join("dataflow.yml");

    // The node lingers 5s after receiving `Stop` before actually exiting,
    // which keeps the restart's `DataflowFinishedOnDaemon` from arriving
    // for that long — guaranteeing the coordinator's `pending_restarts`
    // entry is still present when the concurrent `stop` reaches it below.
    std::fs::write(
        &dataflow_yml,
        format!(
            "nodes:\n  \
             - id: slow-stop\n    \
             path: {node}\n    \
             env:\n      \
             DORA_TEST_STOP_DELAY_MS: \"5000\"\n",
            node = node.display(),
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
        || list_active(port).map(|v| v.len() == 1).unwrap_or(false),
        Duration::from_secs(30),
        "dataflow to register as Running",
    );
    let dataflow_id = list_active(port).expect("list active")[0];

    // Fire `restart` and `stop` on the same UUID: `restart` first (so its
    // `StopDataflow` send + `pending_restarts` insert — near-instant —
    // reliably happens before `stop` is issued), then `stop` while the
    // node's 5s shutdown delay keeps the restart pending.
    let restart_child = Command::new(&dora)
        .arg("restart")
        .arg(dataflow_id.to_string())
        .arg("--coordinator-port")
        .arg(port.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn dora restart");

    std::thread::sleep(Duration::from_millis(300));

    let stop_output = Command::new(&dora)
        .arg("stop")
        .arg(dataflow_id.to_string())
        .arg("--coordinator-port")
        .arg(port.to_string())
        .output()
        .expect("failed to run dora stop");

    let restart_output = restart_child
        .wait_with_output()
        .expect("failed to wait on dora restart");

    if !restart_output.status.success() {
        dump_logs(&coord_log, &daemon_log);
        panic!(
            "dora restart failed: stdout={} stderr={}",
            String::from_utf8_lossy(&restart_output.stdout),
            String::from_utf8_lossy(&restart_output.stderr),
        );
    }

    // Core assertion: `stop` must be rejected, not report success while a
    // restart is in flight for the same UUID (the silent-loss race).
    let stop_stderr = String::from_utf8_lossy(&stop_output.stderr);
    if stop_output.status.success() {
        dump_logs(&coord_log, &daemon_log);
        panic!(
            "dora stop unexpectedly succeeded while a restart was pending for {dataflow_id} \
             (stdout={:?}); this is the silent-loss race the fix guards against",
            String::from_utf8_lossy(&stop_output.stdout),
        );
    }
    assert!(
        stop_stderr.contains("is being restarted"),
        "expected the pending-restart rejection message, got stderr={stop_stderr:?}"
    );

    // The restart itself must still complete normally: exactly one
    // dataflow running, under a fresh UUID (not the one `stop` targeted).
    wait_until(
        || {
            list_active(port)
                .map(|v| v.len() == 1 && !v.contains(&dataflow_id))
                .unwrap_or(false)
        },
        Duration::from_secs(30),
        "restarted dataflow to come back up under a new UUID",
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
