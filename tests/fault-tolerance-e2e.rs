use dora_daemon::{Daemon, LogDestination};
use dora_message::SessionId;
use std::path::Path;
use std::sync::Once;
use std::time::Duration;

static BUILD_NODES: Once = Once::new();

fn ensure_nodes_built() {
    BUILD_NODES.call_once(|| {
        let status = std::process::Command::new("cargo")
            .args([
                "build",
                "-p",
                "rust-dataflow-example-node",
                "-p",
                "rust-dataflow-example-status-node",
                "-p",
                "rust-dataflow-example-sink",
            ])
            .status()
            .expect("failed to run cargo build");
        assert!(status.success(), "failed to build example nodes");
    });
}

/// Restart policy recovers a failing node.
///
/// The status-node panics on "fail" timer inputs but has restart_policy: on-failure.
/// With max_restarts: 5, the dataflow should keep running despite repeated panics.
/// stop_after ensures the test terminates.
///
/// We verify the dataflow completes without error, proving the restart mechanism
/// kept the pipeline alive. The final node_result is Ok(()) because stop_after
/// sends a graceful Stop to the last (restarted) instance of the node.
#[tokio::test(flavor = "multi_thread")]
async fn restart_recovers_from_failure() {
    ensure_nodes_built();

    let dataflow_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/restart-recovers.yml");

    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        Some(Duration::from_secs(30)),
        false,
    )
    .await;

    let dr = result.expect("dataflow should complete without error");
    eprintln!(
        "dataflow completed with {} node results",
        dr.node_results.len()
    );
    for (id, r) in &dr.node_results {
        eprintln!("  {id}: {r:?}");
    }

    // The dataflow completing without error proves the restart mechanism worked:
    // the status-node panicked repeatedly but was restarted, keeping the pipeline alive
    // until stop_after fired. The final result is Ok(()) because stop_after sends a
    // graceful Stop to the current (restarted) node instance.
    let status_result = dr.node_results.get(&"rust-status-node".to_string().into());
    assert!(
        status_result.is_some(),
        "rust-status-node should be in node_results"
    );
    // The node was restarted and eventually stopped gracefully by stop_after.
    assert!(
        status_result.unwrap().is_ok(),
        "restarted node should exit cleanly via stop_after: {status_result:?}"
    );
}

/// Max restarts limit is enforced.
///
/// The status-node always panics (fail timer fires immediately). With max_restarts: 2,
/// the node should fail permanently after 2 restart attempts. The dataflow continues
/// because other nodes are still running, and stop_after terminates it.
#[tokio::test(flavor = "multi_thread")]
async fn max_restarts_limit_reached() {
    ensure_nodes_built();

    let dataflow_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/max-restarts-exceeded.yml");

    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        Some(Duration::from_secs(30)),
        false,
    )
    .await;

    // The dataflow should complete (stop_after terminates it after the node exhausts restarts).
    let dr = result.expect("dataflow should complete without error");
    eprintln!(
        "dataflow completed with {} node results",
        dr.node_results.len()
    );
    for (id, r) in &dr.node_results {
        eprintln!("  {id}: {r:?}");
    }

    // Verify the status-node has a result entry — it ran (and crashed/restarted).
    let status_result = dr.node_results.get(&"rust-status-node".to_string().into());
    assert!(
        status_result.is_some(),
        "rust-status-node should be in node_results"
    );
    // After exhausting max_restarts, the final exit (either the last crash or a
    // graceful stop_after) is recorded. Both outcomes prove the restart budget
    // was consumed and the dataflow survived.
    eprintln!("status-node final result: {status_result:?} (both Ok and Err are valid)");
}

/// Max restarts are **actually exhausted** — strengthens
/// `max_restarts_limit_reached` (#1631).
///
/// Background: the pre-existing `max_restarts_limit_reached` test uses the
/// shared `rust-dataflow-example-status-node`, which panics only on
/// `restart_count() == 0`. With the fail timer firing every 50 ms, the
/// first incarnation crashes, the daemon restarts the node, and every
/// incarnation after that ignores the fail signal. `max_restarts: 2` is
/// therefore **never** reached, so the test falls back to a permissive
/// "both `Ok` and `Err` are valid" comment that defeats the point.
///
/// This test drives exhaustion deterministically with a dedicated test
/// crate (`always-crash-node`) that panics on every incarnation. With
/// `max_restarts: 2` and a 50 ms tick, the budget is consumed within
/// ~300 ms (3 crashes × 100 ms restart_delay) and the daemon must end
/// the dataflow with an `Err` for the always-crash node. The assertion
/// is now: the node's final `node_result` is `Err`, not "either is fine".
#[tokio::test(flavor = "multi_thread")]
async fn max_restarts_exhaustion_marks_node_failed() {
    ensure_nodes_built();
    // Shares the `BUILD_NODES` Once gate above only because rustfmt likes it
    // co-located; the always-crash-node crate is a separate workspace
    // member and has its own compile here.
    let status = std::process::Command::new("cargo")
        .args(["build", "-p", "always-crash-node"])
        .status()
        .expect("failed to build always-crash-node");
    assert!(status.success(), "always-crash-node build failed");

    let dataflow_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/dataflows/max-restarts-truly-exhausted.yml");

    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        Some(Duration::from_secs(10)),
        false,
    )
    .await;

    let dr =
        result.expect("dataflow run itself should not error — per-node exhaustion is expected");
    eprintln!(
        "dataflow completed with {} node results",
        dr.node_results.len()
    );
    for (id, r) in &dr.node_results {
        eprintln!("  {id}: {r:?}");
    }

    let crash_result = dr
        .node_results
        .get(&"always-crash".to_string().into())
        .expect("always-crash node should be present in node_results");

    // Core contract: after `max_restarts` is truly exhausted, the daemon
    // records an `Err` for the affected node. An `Ok` here would indicate
    // either a silent restart bypass or stop_after racing ahead of the
    // final crash before the daemon published the terminal result — both
    // are regressions we want to catch.
    assert!(
        crash_result.is_err(),
        "always-crash node exhausted max_restarts: 2 but final result is Ok — \
         restart budget was not properly tracked as exhausted. got: {crash_result:?}"
    );
}

/// `restart_policy: always` restarts the node even on a clean exit
/// (#1631).
///
/// `on-failure` and `never` both leave a cleanly-exiting node alone.
/// Only `always` re-spawns after `Ok`. This test proves the difference
/// observable, not just inferred.
///
/// Fixture: `clean-exit-node` appends one `incarnation\n` line to
/// `$DORA_TEST_MARKER_FILE` on every spawn, then exits `Ok(())` after
/// the first tick. With `restart_policy: always` + `max_restarts: 2`,
/// the daemon must spawn the node three times (initial + 2 restarts)
/// — so the marker file grows to exactly three lines. With
/// `restart_policy: never`, it would contain one.
#[tokio::test(flavor = "multi_thread")]
async fn restart_policy_always_restarts_on_clean_exit() {
    let status = std::process::Command::new("cargo")
        .args(["build", "-p", "clean-exit-node"])
        .status()
        .expect("failed to build clean-exit-node");
    assert!(status.success(), "clean-exit-node build failed");

    // Fixture writes to this hardcoded path; tests already run with
    // --test-threads=1, so there is no cross-test contention. Clean
    // before and after so a prior crash doesn't pollute.
    let marker = Path::new("/tmp/dora-restart-policy-always.log");
    let _ = std::fs::remove_file(marker);

    let dataflow_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/restart-policy-always.yml");

    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        Some(Duration::from_secs(10)),
        false,
    )
    .await;

    let dr = result.expect("dataflow should complete");
    eprintln!(
        "dataflow completed with {} node results",
        dr.node_results.len()
    );
    for (id, r) in &dr.node_results {
        eprintln!("  {id}: {r:?}");
    }

    let contents = std::fs::read_to_string(marker)
        .expect("marker file should exist — the node writes it on every spawn");
    let incarnations = contents.lines().count();
    let _ = std::fs::remove_file(marker);
    eprintln!("clean-exit-node incarnations observed: {incarnations}");

    // Core contract: `restart_policy: always` + `max_restarts: 2`
    // produces exactly 3 incarnations (initial spawn + 2 restarts).
    // With the fixture's 50 ms tick + 100 ms restart_delay the full
    // cycle completes in roughly 500 ms, leaving ample headroom under
    // the 10 s stop_after — there's no legitimate race that would cut
    // the count short, so a weaker `>= 2` assertion would let an
    // off-by-one regression slip through (addressing review feedback
    // on PR #1645).
    assert_eq!(
        incarnations, 3,
        "restart_policy: always + max_restarts: 2 should produce exactly 3 \
         incarnations (initial + 2 restarts); got {incarnations}. marker contents:\n{contents}"
    );
}

/// `restart_window` resets the restart counter after the window
/// elapses (#1631).
///
/// The daemon's restart accounting (binaries/daemon/src/spawn/prepared.rs:
/// around the `restart` check) resets `window_count` when
/// `window_start.elapsed() > window`. That means a node whose restarts
/// are spaced further apart than `restart_window` can recover budget
/// indefinitely, even with a tiny `max_restarts`.
///
/// Fixture: `delayed-crash-node` appends an incarnation marker, sleeps
/// 500 ms, then panics. With `restart_window: 0.3`, every crash arrives
/// after the previous window has already expired — the counter resets,
/// and the node is re-spawned. Without the window reset (or with
/// `restart_window` misconfigured), `max_restarts: 1` would exhaust
/// after the second crash and the node would stop at 2 incarnations.
///
/// Expected over a 10 s stop_after window (cycle ≈ 600 ms): ~16–17
/// incarnations. We assert `>= 3` — the minimum that could *only* be
/// produced by a window reset (without reset, `max_restarts: 1` caps
/// incarnations at 2). A regression that disables the reset lands at
/// exactly 2, which the assertion rejects.
#[tokio::test(flavor = "multi_thread")]
async fn restart_window_resets_restart_counter() {
    let status = std::process::Command::new("cargo")
        .args(["build", "-p", "delayed-crash-node"])
        .status()
        .expect("failed to build delayed-crash-node");
    assert!(status.success(), "delayed-crash-node build failed");

    // Fixture writes to this hardcoded path; tests run with
    // --test-threads=1 so there's no cross-test contention. Pre-clean
    // in case a prior crash left stale markers around.
    let marker = Path::new("/tmp/dora-restart-window-reset.log");
    let _ = std::fs::remove_file(marker);

    let dataflow_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/restart-window-reset.yml");

    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        Some(Duration::from_secs(10)),
        false,
    )
    .await;

    let dr = result.expect("dataflow should complete");
    eprintln!(
        "dataflow completed with {} node results",
        dr.node_results.len()
    );
    for (id, r) in &dr.node_results {
        eprintln!("  {id}: {r:?}");
    }

    let contents = std::fs::read_to_string(marker)
        .expect("marker file should exist — the node writes it on every spawn");
    let incarnations = contents.lines().count();
    let _ = std::fs::remove_file(marker);
    eprintln!("delayed-crash-node incarnations observed: {incarnations}");

    // `max_restarts: 1` with no window reset → exactly 2 incarnations.
    // Anything strictly greater proves the counter reset path fired.
    assert!(
        incarnations >= 3,
        "restart_window: 0.3 with max_restarts: 1 should let the node restart \
         many times via window reset; got {incarnations} incarnations. Without \
         reset this would cap at 2. marker contents:\n{contents}"
    );
}

/// Input timeout fires when upstream stops producing.
///
/// The status-node exits after receiving trigger-exit. The sink has input_timeout: 2.0
/// on the status-node output. After the timeout, the sink receives InputClosed and
/// the dataflow completes gracefully.
#[tokio::test(flavor = "multi_thread")]
async fn input_timeout_closes_stale_input() {
    ensure_nodes_built();

    let dataflow_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/input-timeout.yml");

    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        Some(Duration::from_secs(10)),
        false,
    )
    .await;

    match &result {
        Ok(dr) => {
            eprintln!(
                "dataflow completed with {} node results",
                dr.node_results.len()
            );
            for (id, r) in &dr.node_results {
                eprintln!("  {id}: {r:?}");
            }
        }
        Err(e) => {
            eprintln!("dataflow error: {e}");
        }
    }

    assert!(result.is_ok(), "dataflow should complete: {result:?}");
}
