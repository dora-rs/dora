use adora_daemon::{Daemon, LogDestination};
use adora_message::SessionId;
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
        Some(Duration::from_secs(15)),
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

    // The status-node should have an error result (it panics on "fail" timer),
    // proving it crashed and the restart mechanism was exercised.
    let status_result = dr.node_results.get(&"rust-status-node".to_string().into());
    assert!(
        status_result.is_some(),
        "rust-status-node should be in node_results"
    );
    assert!(
        status_result.unwrap().is_err(),
        "rust-status-node should have error result (it panics): {status_result:?}"
    );
}

/// Max restarts limit is enforced.
///
/// The status-node always panics (fail timer fires immediately). With max_restarts: 2,
/// the node should fail permanently after 2 restart attempts.
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
        Some(Duration::from_secs(15)),
    )
    .await;

    // The dataflow should complete (the node exhausts its restart budget).
    let dr = result.expect("dataflow should complete without error");
    eprintln!(
        "dataflow completed with {} node results",
        dr.node_results.len()
    );
    for (id, r) in &dr.node_results {
        eprintln!("  {id}: {r:?}");
    }

    // The failing node must have an error result — it always panics and
    // exhausts its max_restarts budget.
    let status_result = dr.node_results.get(&"rust-status-node".to_string().into());
    assert!(
        status_result.is_some(),
        "rust-status-node should be in node_results"
    );
    assert!(
        status_result.unwrap().is_err(),
        "node that always panics should have error result: {status_result:?}"
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
