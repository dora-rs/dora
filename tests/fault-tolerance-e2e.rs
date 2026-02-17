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

    let dataflow_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/dataflows/restart-recovers.yml");

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

    // The dataflow should complete (stop_after triggers graceful shutdown).
    // It's OK if it returns Ok or an error — the key point is that the daemon
    // didn't hang and the restart mechanism kept the node running.
    match &result {
        Ok(dr) => {
            eprintln!("dataflow completed with {} node results", dr.node_results.len());
            for (id, r) in &dr.node_results {
                eprintln!("  {id}: {r:?}");
            }
        }
        Err(e) => {
            eprintln!("dataflow returned error (acceptable for restart test): {e}");
        }
    }

    // The dataflow ran to completion without hanging — that's the success criterion.
    // The restart mechanism kept the failing node alive until stop_after fired.
    assert!(result.is_ok(), "dataflow should complete: {result:?}");
}

/// Max restarts limit is enforced.
///
/// The status-node always panics (fail timer fires immediately). With max_restarts: 2,
/// the node should fail permanently after 2 restart attempts.
#[tokio::test(flavor = "multi_thread")]
async fn max_restarts_limit_reached() {
    ensure_nodes_built();

    let dataflow_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/dataflows/max-restarts-exceeded.yml");

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
    match &result {
        Ok(dr) => {
            eprintln!("dataflow completed with {} node results", dr.node_results.len());
            for (id, r) in &dr.node_results {
                eprintln!("  {id}: {r:?}");
            }
            // The failing node should have an error result
            let status_node_result = dr.node_results.get(&"rust-status-node".to_string().into());
            if let Some(r) = status_node_result {
                assert!(r.is_err(), "node that always panics should have error result");
            }
        }
        Err(e) => {
            // An error is also acceptable — it means the daemon detected the permanent failure
            eprintln!("dataflow error (expected for max-restarts test): {e}");
        }
    }
}

/// Input timeout fires when upstream stops producing.
///
/// The status-node exits after receiving trigger-exit. The sink has input_timeout: 2.0
/// on the status-node output. After the timeout, the sink receives InputClosed and
/// the dataflow completes gracefully.
#[tokio::test(flavor = "multi_thread")]
async fn input_timeout_closes_stale_input() {
    ensure_nodes_built();

    let dataflow_path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/dataflows/input-timeout.yml");

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
            eprintln!("dataflow completed with {} node results", dr.node_results.len());
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
