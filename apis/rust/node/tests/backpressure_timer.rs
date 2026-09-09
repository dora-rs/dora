//! Real-runtime regression for a `queue_policy: backpressure` input lost
//! behind a timer flood (dora-rs/dora#3428).
//!
//! Driven by `contract_backpressure_commit_survives_timer_pressure` in
//! `tests/example-smoke.rs` with the CLI built from the checkout. By hand:
//!
//! ```text
//! cargo build -p dora-cli
//! cargo test -p dora-node-api --test backpressure_timer slow_consumer -- --ignored --nocapture
//! ```
//!
//! `DORA_BACKPRESSURE_TEST_CLI` overrides the CLI path.

use dora_node_api::{DoraNode, Event, IntoArrow};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
    thread,
    time::{Duration, Instant},
};

// The timings below form one scenario; change them together.
//
//   SENDER_DELAY < RECEIVER_IDLE < RECEIVER_DEADLINE < RUN_LENGTH
//   SENDER_DELAY + SENDER_LINGER < RUN_LENGTH
//
// The bug needs the commit to arrive while the receiver is not draining its
// event stream and the timer has already filled the receiver's shared ingress
// channel (`max(sum(queue_size), 64)` slots). With a 10 ms timer the idle
// window is ~300 ticks, several times that channel; the commit lands around
// tick 150.

/// The sender waits this long before its single commit.
const SENDER_DELAY: Duration = Duration::from_millis(1500);
/// How long the sender stays alive after the commit, so the dataflow is still
/// up while the receiver polls.
const SENDER_LINGER: Duration = Duration::from_secs(4);
/// How long the receiver ignores its event stream after starting.
const RECEIVER_IDLE: Duration = Duration::from_secs(3);
/// How long the receiver polls for the commit, measured from its start.
const RECEIVER_DEADLINE: Duration = Duration::from_secs(5);
/// Timer period that floods the receiver's ingress while it idles.
const TIMER_PERIOD_MS: u64 = 10;
/// Cap on the whole run, handed to `dora run --stop-after`.
const RUN_LENGTH: &str = "8s";

#[test]
#[ignore = "requires the dora CLI built from this checkout (see module docs)"]
fn slow_consumer_keeps_backpressure_commit() -> eyre::Result<()> {
    let cli = cli_path()?;
    let executable = std::env::current_exe()?;
    // `with_timer = false` is the control: it proves the harness delivers the
    // commit at all, so a loss in the flooded run is the flood's doing.
    for with_timer in [false, true] {
        let root = std::env::temp_dir().join(format!("dora-pressure-{}", uuid::Uuid::now_v7()));
        fs::create_dir_all(&root)?;
        let result = root.join("received.txt");
        let mut inputs = serde_json::json!({
            "commit": {"source": "sender/commit", "queue_size": 4, "queue_policy": "backpressure"}
        });
        if with_timer {
            inputs["tick"] = serde_json::json!(format!("dora/timer/millis/{TIMER_PERIOD_MS}"));
        }
        let graph = serde_json::json!({"nodes": [
            {"id": "receiver", "path": executable, "args": "--ignored --exact receiver_node --nocapture",
             "env": {"DORA_PRESSURE_RESULT": result}, "inputs": inputs},
            {"id": "sender", "path": executable, "args": "--ignored --exact sender_node --nocapture", "outputs": ["commit"]}
        ]});
        let graph_path = root.join("graph.yaml");
        fs::write(&graph_path, serde_yaml::to_string(&graph)?)?;
        let output = Command::new(&cli)
            .args(["run", "--stop-after", RUN_LENGTH])
            .arg(&graph_path)
            .current_dir(&root)
            .output()?;
        fs::write(root.join("stdout.log"), &output.stdout)?;
        fs::write(root.join("stderr.log"), &output.stderr)?;
        let received =
            fs::read_to_string(&result).unwrap_or_else(|err| format!("<no result file: {err}>"));
        if !output.status.success() || received != "received" {
            // Keep the scratch directory: it is the evidence.
            eyre::bail!(
                "with_timer={with_timer}: dora run exited with {:?}, receiver reported \
                 {received:?}; evidence kept at {}",
                output.status,
                root.display()
            );
        }
        let _ = fs::remove_dir_all(&root);
    }
    Ok(())
}

/// The CLI that runs the dataflow: `DORA_BACKPRESSURE_TEST_CLI`, else the
/// workspace's own debug build. Never a bare `dora` from `PATH`, which would
/// silently validate whatever install happens to come first (the same hazard
/// `dora_bin` in `tests/example-smoke.rs` guards against).
fn cli_path() -> eyre::Result<PathBuf> {
    if let Some(cli) = std::env::var_os("DORA_BACKPRESSURE_TEST_CLI") {
        return Ok(cli.into());
    }
    let target_root = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../target"));
    let cli = target_root
        .join("debug")
        .join(format!("dora{}", std::env::consts::EXE_SUFFIX));
    eyre::ensure!(
        cli.exists(),
        "no dora CLI at {}; run `cargo build -p dora-cli` or set DORA_BACKPRESSURE_TEST_CLI",
        cli.display()
    );
    Ok(cli)
}

/// The two node helpers below are meant to be launched by the daemon. Run
/// outside a dataflow — say, by a blanket `--include-ignored` sweep — they
/// have no node config and must not fail the suite.
fn launched_as_node(name: &str) -> bool {
    if std::env::var_os("DORA_NODE_CONFIG").is_some() {
        return true;
    }
    eprintln!("{name}: not launched as a dataflow node; nothing to do");
    false
}

#[test]
#[ignore = "helper process launched as a dataflow node"]
fn sender_node() -> eyre::Result<()> {
    if !launched_as_node("sender_node") {
        return Ok(());
    }
    let (mut node, _events) = DoraNode::init_from_env()?;
    thread::sleep(SENDER_DELAY);
    node.send_output(
        "commit".into(),
        Default::default(),
        vec![1_u64].into_arrow(),
    )?;
    thread::sleep(SENDER_LINGER);
    Ok(())
}

#[test]
#[ignore = "helper process launched as a dataflow node"]
fn receiver_node() -> eyre::Result<()> {
    if !launched_as_node("receiver_node") {
        return Ok(());
    }
    let (_node, mut events) = DoraNode::init_from_env()?;
    let started = Instant::now();
    thread::sleep(RECEIVER_IDLE);
    let mut received = false;
    while let Some(event) = events.recv() {
        if matches!(event, Event::Input { ref id, .. } if id.as_str() == "commit") {
            received = true;
            break;
        }
        if started.elapsed() > RECEIVER_DEADLINE {
            break;
        }
    }
    fs::write(
        std::env::var("DORA_PRESSURE_RESULT")?,
        if received { "received" } else { "lost" },
    )?;
    Ok(())
}
