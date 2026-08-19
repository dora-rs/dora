//! End-to-end regression test for dora-rs/dora#2938.
//!
//! On a multi-daemon dataflow the start barrier is resolved by the
//! coordinator, which broadcasts `AllNodesReady` at most once per
//! dataflow. The daemon used to treat that broadcast as the *only* thing
//! that ever answers a parked subscribe request, so every node that
//! subscribed afterwards — a runtime `dora node add`, a dynamic node,
//! a restarted node — parked forever and never returned from `Node()`
//! init. Single-machine dataflows were unaffected (they answer each
//! subscribe inline), which is why this went unnoticed.
//!
//! The latch that fixed it landed in #2990, verified by unit test only —
//! nothing in the suite covered subscribe-after-startup on the external
//! path, and `tests/node-lifecycle-e2e.rs` is single-daemon throughout.
//! This is that missing coverage: it fails on the parent of #2990 and
//! passes with it.
//!
//! The test runs a real two-daemon deployment (coordinator + daemons `A`
//! and `B` on loopback, all on ephemeral ports), starts a dataflow with
//! a node on each machine, and then adds a node at runtime. The added
//! node records its pid *after* `init_from_env()` returns, so that pid
//! appearing is proof that its subscribe was answered.
//!
//! No cross-daemon *data* delivery is needed: the barrier travels over
//! the coordinator's WebSocket, and the added node talks to its own
//! local daemon. That keeps the test independent of the multicast/zenoh
//! peering quirks of CI runners and dev containers.
//!
//! Heavyweight (spawns processes, ~30-60s). It lives in the
//! `dora-examples` crate, which the fast PR `cargo test --all` step
//! excludes; run it explicitly with
//! `cargo test -p dora-examples --test multi-daemon-e2e`.

use std::net::{SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Once;
use std::time::{Duration, Instant};

use dora_cli::WsSession;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};

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

/// Grab `n` currently-free, distinct TCP ports. There is a small TOCTOU
/// window between closing these listeners and the servers binding them,
/// but it is more than adequate to keep parallel test binaries off each
/// other's ports. Held open until all `n` are picked, so the four
/// servers here (coordinator, two daemons, zenoh rendezvous) can never
/// be handed the same port twice.
fn free_ports(n: usize) -> Vec<u16> {
    let listeners: Vec<_> = (0..n)
        .map(|_| std::net::TcpListener::bind("127.0.0.1:0").expect("failed to bind ephemeral port"))
        .collect();
    listeners
        .iter()
        .map(|l| l.local_addr().expect("failed to read local addr").port())
        .collect()
}

fn port_open(port: u16) -> bool {
    TcpStream::connect(("127.0.0.1", port)).is_ok()
}

fn wait_until(mut f: impl FnMut() -> bool, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if f() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
    false
}

/// Machine ids of the daemons currently registered with the coordinator.
/// `Err` while the coordinator is not answering yet.
fn connected_machines(coordinator_port: u16) -> eyre::Result<Vec<String>> {
    let addr: SocketAddr = format!("127.0.0.1:{coordinator_port}").parse().unwrap();
    let session = WsSession::connect(addr)?;
    let data = serde_json::to_vec(&ControlRequest::ConnectedMachines)?;
    let reply: ControlRequestReply = serde_json::from_slice(&session.request(&data)?)?;
    match reply {
        ControlRequestReply::ConnectedDaemons(daemons) => Ok(daemons
            .into_iter()
            .filter_map(|d| d.daemon_id.machine_id().map(ToOwned::to_owned))
            .collect()),
        other => eyre::bail!("unexpected reply to ConnectedMachines: {other:?}"),
    }
}

/// The pid the `reconnect-survivor-node` fixture records once
/// `init_from_env()` has returned — the signal that its subscribe was
/// answered. Absent (or not yet written) while the node is still in init.
fn recorded_pid(path: &Path) -> Option<u32> {
    std::fs::read_to_string(path)
        .ok()?
        .lines()
        .next()?
        .trim()
        .parse()
        .ok()
}

/// A bare descriptor for the `reconnect-survivor-node` fixture, which
/// records its pid to `$DORA_TEST_PID_FILE` once `init_from_env()` has
/// returned and then idles until it is stopped.
fn node_yaml(id: &str, machine: Option<&str>, pid_file: &Path) -> String {
    let deploy = match machine {
        Some(machine) => format!("deploy:\n  machine: {machine}\n"),
        None => String::new(),
    };
    format!(
        "id: {id}\n\
         {deploy}\
         path: {path}\n\
         inputs:\n  \
         tick: dora/timer/millis/300\n\
         env:\n  \
         DORA_TEST_PID_FILE: {pid_file}\n",
        path = bin("reconnect-survivor-node").display(),
        pid_file = pid_file.display(),
    )
}

/// Fold a bare node descriptor into an item of a `nodes:` list.
fn as_list_item(node: &str) -> String {
    let indented: String = node.lines().map(|line| format!("    {line}\n")).collect();
    format!("  - {}", indented.trim_start())
}

struct Deployment {
    dora: PathBuf,
    coordinator_port: u16,
    children: Vec<Child>,
    logs: Vec<PathBuf>,
}

impl Deployment {
    /// Fail the test with every process log attached — a hang here is
    /// only diagnosable from the daemon and coordinator output.
    fn fail(&self, message: impl std::fmt::Display) -> ! {
        for path in &self.logs {
            eprintln!("--- {} ---", path.display());
            eprintln!("{}", std::fs::read_to_string(path).unwrap_or_default());
        }
        panic!("{message}");
    }
}

/// Best-effort teardown that always runs, even on panic. `destroy` goes
/// first so the daemons stop their own node processes; killing the
/// daemons outright would leave those nodes behind.
impl Drop for Deployment {
    fn drop(&mut self) {
        let _ = Command::new(&self.dora)
            .arg("destroy")
            .arg("--coordinator-port")
            .arg(self.coordinator_port.to_string())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        for child in &mut self.children {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// A node added to a *running distributed* dataflow must come up, i.e.
/// its subscribe must be answered even though the coordinator's one-shot
/// `AllNodesReady` broadcast is long past. Regression test for
/// dora-rs/dora#2938.
#[test]
fn node_added_to_distributed_dataflow_finishes_init() {
    ensure_built();

    let dora = bin("dora");
    let tmp = tempfile::tempdir().expect("create tempdir");
    let pid_a = tmp.path().join("node-a.pid");
    let pid_b = tmp.path().join("node-b.pid");
    let pid_late = tmp.path().join("late-joiner.pid");
    let dataflow_yml = tmp.path().join("dataflow.yml");
    let late_node_yml = tmp.path().join("late-joiner.yml");

    // One node per machine, so both daemons see the dataflow as
    // distributed (`external_nodes`) and the barrier becomes the
    // coordinator's to resolve.
    std::fs::write(
        &dataflow_yml,
        format!(
            "nodes:\n{}{}",
            as_list_item(&node_yaml("node-a", Some("A"), &pid_a)),
            as_list_item(&node_yaml("node-b", Some("B"), &pid_b)),
        ),
    )
    .expect("write dataflow yml");
    // `dora node add --from-yaml` takes a single bare node, and places
    // it itself — either daemon exercises the same barrier.
    std::fs::write(&late_node_yml, node_yaml("late-joiner", None, &pid_late))
        .expect("write late-joiner yml");

    let ports = free_ports(4);
    let (coordinator_port, zenoh_port) = (ports[0], ports[1]);
    let mut daemon_listen_ports = [ports[2], ports[3]].into_iter();
    let zenoh_peer = format!("tcp/127.0.0.1:{zenoh_port}");
    let coord_log = tmp.path().join("coordinator.log");
    let coord_out = std::fs::File::create(&coord_log).expect("create coordinator log");
    let coord_err = coord_out.try_clone().expect("clone coordinator log");

    let mut deployment = Deployment {
        dora: dora.clone(),
        coordinator_port,
        children: vec![
            Command::new(&dora)
                .arg("coordinator")
                .arg("--port")
                .arg(coordinator_port.to_string())
                .arg("--store")
                .arg(format!(
                    "redb:{}",
                    tmp.path().join("coordinator.redb").display()
                ))
                .stdout(Stdio::from(coord_out))
                .stderr(Stdio::from(coord_err))
                .spawn()
                .expect("failed to spawn coordinator"),
        ],
        logs: vec![coord_log],
    };
    if !wait_until(|| port_open(coordinator_port), Duration::from_secs(20)) {
        deployment.fail("coordinator did not accept connections within 20s");
    }

    // Two named daemons on one host: distinct local listen ports, and an
    // explicit zenoh rendezvous endpoint so they find each other without
    // multicast (CI runners and dev containers have none).
    for machine in ["A", "B"] {
        let log = tmp.path().join(format!("daemon-{machine}.log"));
        let out = std::fs::File::create(&log).expect("create daemon log");
        let err = out.try_clone().expect("clone daemon log");
        deployment.children.push(
            Command::new(&dora)
                .arg("daemon")
                .arg("--machine-id")
                .arg(machine)
                .arg("--coordinator-port")
                .arg(coordinator_port.to_string())
                .arg("--local-listen-port")
                .arg(
                    daemon_listen_ports
                        .next()
                        .expect("one listen port per daemon")
                        .to_string(),
                )
                .arg("--zenoh-peer")
                .arg(&zenoh_peer)
                .stdout(Stdio::from(out))
                .stderr(Stdio::from(err))
                .spawn()
                .expect("failed to spawn daemon"),
        );
        deployment.logs.push(log);
    }

    // `dora start` rejects a `deploy.machine` whose daemon has
    // not registered yet, so wait for both rather than race them.
    if !wait_until(
        || {
            connected_machines(coordinator_port)
                .map(|m| m.iter().any(|id| id == "A") && m.iter().any(|id| id == "B"))
                .unwrap_or(false)
        },
        Duration::from_secs(30),
    ) {
        deployment.fail("daemons A and B did not both register within 30s");
    }

    let start = Command::new(&dora)
        .arg("start")
        .arg(&dataflow_yml)
        .arg("--name")
        .arg("multi-daemon-late-join")
        .arg("--detach")
        .arg("--coordinator-port")
        .arg(coordinator_port.to_string())
        .output()
        .expect("failed to run dora start");
    if !start.status.success() {
        deployment.fail(format!(
            "dora start failed: {}",
            String::from_utf8_lossy(&start.stderr)
        ));
    }

    // Both descriptor nodes past init means the coordinator's one-shot
    // broadcast has fired, which is the precondition under test.
    if !wait_until(
        || recorded_pid(&pid_a).is_some() && recorded_pid(&pid_b).is_some(),
        Duration::from_secs(60),
    ) {
        deployment.fail("distributed dataflow did not start within 60s");
    }

    let add = Command::new(&dora)
        .arg("node")
        .arg("add")
        .arg("--from-yaml")
        .arg(&late_node_yml)
        .arg("--dataflow")
        .arg("multi-daemon-late-join")
        .arg("--coordinator-port")
        .arg(coordinator_port.to_string())
        .output()
        .expect("failed to run dora node add");
    if !add.status.success() {
        deployment.fail(format!(
            "dora node add failed: {}",
            String::from_utf8_lossy(&add.stderr)
        ));
    }

    if !wait_until(
        || recorded_pid(&pid_late).is_some(),
        Duration::from_secs(45),
    ) {
        deployment.fail(
            "the node added at runtime never finished `Node()` init: its \
             subscribe was parked with nothing left to answer it \
             (dora-rs/dora#2938)",
        );
    }
}
