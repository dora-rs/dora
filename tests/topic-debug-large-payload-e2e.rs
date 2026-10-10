//! End-to-end test for large-payload topic inspection (dora-rs/dora#3535):
//! `dora topic hz` on a multi-MB output receives its frames, and the
//! daemon↔coordinator connection survives them.
//!
//! As JSON, a topic debug frame renders its payload as a decimal number array,
//! and past ~250 KB of payload that exceeds the 1 MiB message limit of the
//! daemon socket, so the coordinator reset the connection and hz received
//! nothing. Negotiated binary frames, sent in chunks under that limit, carry
//! them instead.
//!
//! The test runs a real coordinator, daemon and a node emitting 3 MB outputs
//! at 30 Hz, keeps `dora topic hz` subscribed, then stops the dataflow. The
//! assertion that fails without binary frames is that hz received any frames.
//!
//! What this test does *not* guard is control priority — control messages
//! going out ahead of queued debug frames. On localhost the link is fast
//! enough that `dora stop` completes in well under a second even without it,
//! so the stop bound below is a smoke check, not a regression test for it.
//! Priority is covered by unit tests on both ends:
//! `ws_writer_sends_control_before_queued_topic_debug_frames` and
//! `ws_writer_sends_control_between_the_chunks_of_a_debug_frame` in the
//! daemon, `control_priority_tests` in the coordinator.
//!
//! Heavyweight (spawns processes). Run with
//! `cargo test -p dora-examples --test topic-debug-large-payload-e2e`.

use std::net::{SocketAddr, TcpStream};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Once;
use std::time::{Duration, Instant};

use dora_cli::WsSession;
use dora_message::{cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply};
use uuid::Uuid;

static BUILD: Once = Once::new();

/// Size of each output. Around a 1280×720 RGB image, and about 13 MB once
/// rendered as a JSON number array.
const OUTPUT_BYTES: usize = 3_000_000;

/// How long `dora topic hz` streams before `dora stop` is sent.
const STREAM_BEFORE_STOP: Duration = Duration::from_secs(8);

/// Upper bound for `dora stop`: a smoke check that streaming large frames does
/// not wedge the control plane, not a latency guarantee (see the module docs).
/// The node honors `Stop` at once, so this takes a second or two.
const STOP_BUDGET: Duration = Duration::from_secs(15);

fn ensure_built() {
    BUILD.call_once(|| {
        let status = Command::new("cargo")
            .args(["build", "-p", "dora-cli", "-p", "large-output-source-node"])
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
#[derive(Default)]
struct Cleanup {
    children: Vec<Child>,
}

impl Drop for Cleanup {
    fn drop(&mut self) {
        // Reverse spawn order: hz, then daemon, then coordinator.
        for child in self.children.iter_mut().rev() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

/// `dora stop` completes promptly while `dora topic hz` streams a multi-MB
/// output, and hz still receives frames.
#[test]
fn topic_hz_receives_a_large_output_and_stop_still_completes() {
    ensure_built();

    let dora = bin("dora");
    let node = bin("large-output-source-node");

    let tmp = tempfile::tempdir().expect("create tempdir");
    let coord_log = tmp.path().join("coordinator.log");
    let daemon_log = tmp.path().join("daemon.log");
    let dataflow_yml = tmp.path().join("dataflow.yml");

    std::fs::write(
        &dataflow_yml,
        format!(
            "nodes:\n  \
             - id: source\n    \
             path: {node}\n    \
             inputs:\n      \
             tick: dora/timer/millis/33\n    \
             outputs:\n      \
             - frame\n    \
             env:\n      \
             DORA_TEST_OUTPUT_BYTES: \"{OUTPUT_BYTES}\"\n\
             debug:\n  \
             enable_debug_inspection: true\n",
            node = node.display(),
        ),
    )
    .expect("write dataflow yml");

    let port = free_port();
    let daemon_listen_port = free_port();
    let mut cleanup = Cleanup::default();

    let out = std::fs::File::create(&coord_log).expect("create coordinator log");
    let err = out.try_clone().expect("clone coordinator log");
    cleanup.children.push(
        Command::new(&dora)
            .arg("coordinator")
            .arg("--port")
            .arg(port.to_string())
            .stdout(Stdio::from(out))
            .stderr(Stdio::from(err))
            .spawn()
            .expect("failed to spawn coordinator"),
    );
    wait_until(
        || port_open(port),
        Duration::from_secs(20),
        "coordinator to accept connections",
    );

    let out = std::fs::File::create(&daemon_log).expect("create daemon log");
    let err = out.try_clone().expect("clone daemon log");
    cleanup.children.push(
        Command::new(&dora)
            .arg("daemon")
            .arg("--coordinator-port")
            .arg(port.to_string())
            .arg("--local-listen-port")
            .arg(daemon_listen_port.to_string())
            .stdout(Stdio::from(out))
            .stderr(Stdio::from(err))
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

    // Outlives the stop below, so the subscription is live the whole time.
    // The window covers the full run so the final report counts every frame.
    let hz_duration = STREAM_BEFORE_STOP + STOP_BUDGET + Duration::from_secs(5);
    let hz = Command::new(&dora)
        .arg("topic")
        .arg("hz")
        .arg("--dataflow")
        .arg(dataflow_id.to_string())
        .arg("source/frame")
        .arg("--duration")
        .arg(hz_duration.as_secs().to_string())
        .arg("--window")
        .arg(hz_duration.as_secs().to_string())
        .arg("--coordinator-port")
        .arg(port.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn dora topic hz");
    cleanup.children.push(hz);

    std::thread::sleep(STREAM_BEFORE_STOP);

    let stop_started = Instant::now();
    let mut stop = Command::new(&dora)
        .arg("stop")
        .arg(dataflow_id.to_string())
        .arg("--coordinator-port")
        .arg(port.to_string())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("failed to spawn dora stop");
    let stop_status = loop {
        if let Some(status) = stop.try_wait().expect("failed to poll dora stop") {
            break status;
        }
        if stop_started.elapsed() > STOP_BUDGET {
            let _ = stop.kill();
            let _ = stop.wait();
            dump_logs(&coord_log, &daemon_log);
            panic!(
                "dora stop did not complete within {STOP_BUDGET:?} while `dora topic hz` was \
                 streaming a {OUTPUT_BYTES}-byte output (dora-rs/dora#3535)"
            );
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    let stop_elapsed = stop_started.elapsed();
    let stop_output = stop.wait_with_output().expect("collect dora stop output");
    if !stop_status.success() {
        dump_logs(&coord_log, &daemon_log);
        panic!(
            "dora stop failed: stdout={} stderr={}",
            String::from_utf8_lossy(&stop_output.stdout),
            String::from_utf8_lossy(&stop_output.stderr),
        );
    }
    eprintln!("dora stop completed in {stop_elapsed:?}");

    wait_until(
        || list_active(port).map(|v| v.is_empty()).unwrap_or(false),
        Duration::from_secs(30),
        "dataflow to be fully stopped",
    );

    // hz runs out its `--duration` and prints a table; the `source/frame` row
    // ends in the sample count.
    let hz = cleanup.children.pop().expect("hz child");
    let hz_output = hz.wait_with_output().expect("collect dora topic hz output");
    let hz_stdout = String::from_utf8_lossy(&hz_output.stdout);
    let samples = hz_stdout
        .lines()
        .find(|line| line.starts_with("source/frame\t"))
        .and_then(|line| line.rsplit('\t').next())
        .and_then(|count| count.parse::<usize>().ok());
    match samples {
        Some(samples) if samples > 0 => eprintln!("dora topic hz received {samples} frames"),
        _ => {
            dump_logs(&coord_log, &daemon_log);
            panic!(
                "dora topic hz received no frames of a {OUTPUT_BYTES}-byte output: \
                 status={} stdout={hz_stdout} stderr={}",
                hz_output.status,
                String::from_utf8_lossy(&hz_output.stderr),
            );
        }
    }
}
