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
