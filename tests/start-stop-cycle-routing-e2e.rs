//! Regression anchor for dora-rs/dora#3201: "event routing silently breaks
//! for some subscribers after repeated dataflow start/stop cycles".
//!
//! The original report (dora-cli 0.5.0 era) ran one long-lived coordinator +
//! daemon, started and stopped the same dataflow many times, and at some
//! point — after ~10+ cycles — a sender's outputs silently stopped reaching
//! two of its three subscribers while a newly-added subscriber still
//! received them.
//!
//! On current main the reported failure class is not reachable through a
//! plain start/stop loop: local payloads travel direct zenoh (dora-rs/dora
//! #1787), never through daemon-side `mappings`, and every `Spawn` builds a
//! fresh `RunningDataflow` whose output routing is recomputed from the
//! descriptor at spawn time. The residual silent-loss diagnostic that used
//! to apply to this shape (a `mappings` entry referencing a receiver with no
//! live `subscribe_channels` channel) now surfaces as a daemon warning, the
//! deliverable shipped in dora-rs/dora#3520.
//!
//! This test pins the contract the report otherwise exercises against the
//! single long-lived coordinator + daemon pair: N consecutive
//! `dora start`/`dora stop` cycles of a dataflow with one sender and two
//! subscribers, asserting on **every** cycle — in particular the last — that
//! both subscribers received both of the sender's outputs. On a regression
//! it fails with the coordinator/daemon logs dumped.
//!
//! Heavyweight (spawns processes + a repeated start/stop loop). Run with
//! `cargo test -p dora-examples --test start-stop-cycle-routing-e2e`.
//! The sender is `timed-burst-source-node` (exactly two `value` outputs per
//! run, driven by a `dora/timer` tick), each subscriber an
//! `event-log-observer-node` appending `Input:<id>` to a fresh marker file
//! per cycle.

use std::net::{SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Once;
use std::time::{Duration, Instant};

use dora_cli::WsSession;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use uuid::Uuid;

/// Cycles against the one coordinator + daemon. The report saw the failure
/// only after ~10+ cycles; 12 keeps the runtime bounded while exercising the
/// "repeated start/stop against the same daemon" shape.
const CYCLES: usize = 12;

static BUILD: Once = Once::new();

fn ensure_built() {
    BUILD.call_once(|| {
        let status = Command::new("cargo")
            .args([
                "build",
                "-p",
                "dora-cli",
                "-p",
                "timed-burst-source-node",
                "-p",
                "event-log-observer-node",
            ])
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

/// The `Input:<id>` lines the observer has recorded so far.
fn received(marker: &Path) -> Vec<String> {
    std::fs::read_to_string(marker)
        .map(|s| s.lines().map(str::to_owned).collect())
        .unwrap_or_default()
}

fn write_dataflow(
    dataflow_yml: &Path,
    source: &Path,
    observer: &Path,
    marker_a: &Path,
    marker_b: &Path,
) {
    std::fs::write(
        dataflow_yml,
        format!(
            r#"nodes:
  - id: source
    path: {source}
    inputs:
      tick: dora/timer/millis/200
    outputs:
      - value
    env:
      DORA_TEST_BURST_GAP_MS: "50"
  - id: observer-a
    path: {observer}
    env:
      DORA_TEST_MARKER_FILE: {marker_a}
    inputs:
      value:
        source: source/value
  - id: observer-b
    path: {observer}
    env:
      DORA_TEST_MARKER_FILE: {marker_b}
    inputs:
      value:
        source: source/value
"#,
            source = source.display(),
            observer = observer.display(),
            marker_a = marker_a.display(),
            marker_b = marker_b.display(),
        ),
    )
    .expect("write dataflow yml");
}

/// Repeatedly start/stop a one-sender/two-subscriber dataflow against a
/// single long-lived coordinator + daemon, asserting both subscribers
/// receive both outputs on every cycle.
#[test]
fn repeated_start_stop_keeps_delivering_to_all_subscribers() {
    ensure_built();

    let dora = bin("dora");
    let source = bin("timed-burst-source-node");
    let observer = bin("event-log-observer-node");

    let tmp = tempfile::tempdir().expect("create tempdir");
    let redb = tmp.path().join("coordinator.redb");
    let coord_log = tmp.path().join("coordinator.log");
    let daemon_log = tmp.path().join("daemon.log");

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
    wait_until(
        || port_open(daemon_listen_port),
        Duration::from_secs(20),
        "daemon to accept connections",
    );

    for cycle in 0..CYCLES {
        // Fresh dataflow file + fresh marker files per cycle, so delivery
        // counts can never bleed from one run into the next.
        let dataflow_yml = tmp.path().join(format!("dataflow-{cycle}.yml"));
        let marker_a = tmp.path().join(format!("observer-a-{cycle}.log"));
        let marker_b = tmp.path().join(format!("observer-b-{cycle}.log"));
        write_dataflow(&dataflow_yml, &source, &observer, &marker_a, &marker_b);

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
            panic!("cycle {cycle}: dora start failed");
        }

        wait_until(
            || list_active(port).map(|v| v.len() == 1).unwrap_or(false),
            Duration::from_secs(30),
            "dataflow to register as Running",
        );
        let dataflow_id = list_active(port).expect("list active")[0];

        // Wait for both observers to record both inputs. A dedicated loop
        // (not `wait_until`) so a timeout can dump the coordinator/daemon
        // logs — the whole point of this test is diagnosing *silent* loss.
        let deadline = Instant::now() + Duration::from_secs(20);
        while received(&marker_a).len() < 2 || received(&marker_b).len() < 2 {
            if Instant::now() >= deadline {
                dump_logs(&coord_log, &daemon_log);
                panic!("cycle {cycle}: timed out waiting for both observers to log both inputs");
            }
            std::thread::sleep(Duration::from_millis(200));
        }
        let expected: Vec<String> = (0..2).map(|_| "Input:value".to_owned()).collect();
        assert_eq!(
            received(&marker_a),
            expected,
            "observer-a cycle {cycle}: sender output must be delivered"
        );
        assert_eq!(
            received(&marker_b),
            expected,
            "observer-b cycle {cycle}: sender output must be delivered"
        );

        let stop = Command::new(&dora)
            .arg("stop")
            .arg(dataflow_id.to_string())
            .arg("--coordinator-port")
            .arg(port.to_string())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
            .expect("failed to run dora stop");
        if !stop.success() {
            dump_logs(&coord_log, &daemon_log);
            panic!("cycle {cycle}: dora stop failed for {dataflow_id}");
        }

        wait_until(
            || list_active(port).map(|v| v.is_empty()).unwrap_or(false),
            Duration::from_secs(30),
            "dataflow to be fully stopped",
        );
    }

    let _ = Command::new(&dora)
        .arg("stop")
        .arg("--all")
        .arg("--coordinator-port")
        .arg(port.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status();
}
