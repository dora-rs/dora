//! End-to-end validation of the finish-straggler watchdog (dora-rs/dora#2270).
//!
//! Deterministic stand-in for the flaky #2152 macOS hang: a `source` that
//! emits once and exits, plus a `wedge` that subscribes, drops its event
//! stream, and sleeps forever. Once `source` exits the dataflow is
//! "otherwise finished" except `wedge`, which never exits on its own.
//!
//! With the watchdog enabled (`DORA_FINISH_DRAIN_GRACE_SECS`), the daemon
//! escalates `wedge` and the dataflow terminates within the grace + kill
//! ladder instead of hanging until `stop_after`. This is the behaviour the
//! nightly only exercises when the #2152 flake happens to fire; here it is
//! forced every run.

use std::path::Path;
use std::time::{Duration, Instant};

use dora_daemon::{Daemon, LogDestination};
use dora_message::SessionId;

#[tokio::test(flavor = "multi_thread")]
async fn finish_straggler_watchdog_escalates_wedged_node() {
    for pkg in ["emit-then-exit-source-node", "hang-after-init-node"] {
        let status = std::process::Command::new("cargo")
            .args(["build", "-p", pkg])
            .status()
            .expect("failed to run cargo build");
        assert!(status.success(), "failed to build {pkg}");
    }

    // Enable the opt-in watchdog for this daemon. This is the daemon's own
    // process env (run_dataflow runs in-process); set here because the test
    // owns this dedicated test binary, so no other test races the var.
    // SAFETY: single-threaded set before any daemon thread reads it.
    unsafe { std::env::set_var("DORA_FINISH_DRAIN_GRACE_SECS", "2") };

    let marker = Path::new("/tmp/dora-finish-straggler-marker.log");
    let _ = std::fs::remove_file(marker);

    let dataflow_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/dataflows/finish-straggler-escalation.yml");

    let started = Instant::now();
    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        // Generous safety net, far above the ~2 s grace + kill ladder, so a
        // working watchdog finishes well before this and a broken one is
        // caught by the elapsed-time assertion below rather than hanging CI.
        Some(Duration::from_secs(40)),
        false,
        None,
        None,
    )
    .await;
    let elapsed = started.elapsed();

    let dr = result.expect("dataflow should complete");
    eprintln!(
        "dataflow completed in {elapsed:?} with {} node results",
        dr.node_results.len()
    );
    for (id, r) in &dr.node_results {
        eprintln!("  {id}: {r:?}");
    }

    // The marker confirms `wedge` actually reached its post-init hang, so a
    // non-escalation completion can't be "the node never ran".
    let contents = std::fs::read_to_string(marker)
        .expect("wedge marker should exist — hang-after-init writes it at startup");
    let _ = std::fs::remove_file(marker);
    assert!(
        contents.contains("incarnation"),
        "wedge never reached the post-init hang — unrelated regression"
    );

    // Core contract: the watchdog bounded the hang. A working watchdog
    // escalates `wedge` ~2 s after the dataflow is otherwise finished (plus
    // the kill ladder); a disabled/broken one would hang until the 40 s
    // stop_after net.
    assert!(
        elapsed < Duration::from_secs(30),
        "dataflow took {elapsed:?} — the watchdog did not bound the hang \
         (a broken watchdog runs to the 40 s stop_after)"
    );

    // `wedge` was force-terminated by the escalation ladder (a signal exit),
    // not a clean voluntary exit.
    let wedge = dr
        .node_results
        .get(&"wedge".to_string().into())
        .expect("wedge should be in node_results");
    let err = wedge
        .as_ref()
        .expect_err("wedge should have errored — escalation kill is a failure exit");
    use dora_message::common::NodeExitStatus;
    assert!(
        matches!(
            err.exit_status,
            NodeExitStatus::Signal(15) | NodeExitStatus::Signal(9)
        ),
        "expected SIGTERM(15)/SIGKILL(9) from finish-straggler escalation, got {:?}",
        err.exit_status
    );
}
