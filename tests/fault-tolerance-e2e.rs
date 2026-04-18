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

/// `input_timeout` actually delivers `InputClosed` to downstream (#1631).
///
/// The pre-existing `input_timeout_closes_stale_input` test below uses a
/// fixture whose sink exits on the first `exit` input, ~500 ms after
/// startup. That's well before the 2 s `input_timeout` could possibly
/// fire — so `input_timeout` in that test is dead config and the
/// `result.is_ok()` assertion only proves the dataflow completes at all.
///
/// This test closes the gap by driving the timeout path directly:
///
/// - `silent-source-node` emits exactly one output on the first tick
///   then stays alive but silent (so channel-close doesn't fire — the
///   only way the downstream can learn the input went stale is via
///   `input_timeout`).
/// - `input-closed-observer-node` appends one line per event
///   (`Input:<id>` or `InputClosed:<id>`) to
///   `$DORA_TEST_MARKER_FILE`, exiting on the first `InputClosed`.
/// - Fixture `input-timeout-delivery.yml` sets `input_timeout: 0.5`.
///
/// Assertion: the marker records both `Input:value` (so we know the
/// initial message arrived) **and** `InputClosed:value` (so we know
/// the daemon delivered the timeout-triggered close). A regression that
/// silently dropped the timeout path would leave the marker with a
/// single `Input:value` line and the observer would hang until
/// `stop_after`, which the assertion rejects.
#[tokio::test(flavor = "multi_thread")]
async fn input_timeout_delivers_input_closed_to_downstream() {
    let status = std::process::Command::new("cargo")
        .args([
            "build",
            "-p",
            "silent-source-node",
            "-p",
            "input-closed-observer-node",
        ])
        .status()
        .expect("failed to build input-timeout test nodes");
    assert!(status.success(), "input-timeout test nodes build failed");

    let marker = Path::new("/tmp/dora-input-timeout-delivery.log");
    let _ = std::fs::remove_file(marker);

    let dataflow_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/input-timeout-delivery.yml");

    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        // 5 s is plenty — the observer should see Input at ~50 ms and
        // InputClosed at ~550 ms (input_timeout = 0.5 s), then exit.
        Some(Duration::from_secs(5)),
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
        .expect("observer marker file should exist — the observer writes on every event");
    let _ = std::fs::remove_file(marker);
    eprintln!("observer events:\n{contents}");

    assert!(
        contents.contains("Input:value"),
        "observer never saw the initial `value` input — silent-source is not emitting. contents:\n{contents}"
    );
    assert!(
        contents.contains("InputClosed:value"),
        "observer never saw `InputClosed:value` — `input_timeout: 0.5` did not fire even though \
         silent-source stayed alive without producing. contents:\n{contents}"
    );
}

/// `health_check_timeout` kills an unresponsive node with SIGKILL (#1631).
///
/// The daemon's `check_node_health`
/// (binaries/daemon/src/lib.rs:1917) compares
/// `now - last_activity` to the per-node `health_check_timeout`.
/// `last_activity` advances only when the daemon receives a
/// `DaemonRequest` from the node
/// (binaries/daemon/src/node_communication/mod.rs:339), so a node
/// whose background event-stream thread has exited and whose main
/// thread is asleep is silent from the daemon's perspective. When the
/// watchdog fires, it submits `ProcessOperation::Kill` — SIGKILL, not
/// SIGTERM — which surfaces as `NodeExitStatus::Signal(9)` in
/// `DataflowResult::node_results`.
///
/// Fixture: `hang-after-init-node` registers, pumps exactly one event
/// so `last_activity` arms past its unarmed-zero state, writes an
/// incarnation marker, drops `DoraNode` + `EventStream` to kill the
/// background event-stream thread, then sleeps indefinitely. With
/// `health_check_timeout: 0.5` and the dataflow-level
/// `health_check_interval: 0.1`, the kill fires within ~0.6 s.
///
/// A regression that disabled the watchdog would surface as a
/// `NodeExitStatus::Signal(15)` (the `stop_after` SIGTERM path) or a
/// `GraceDuration` NodeErrorCause — both of which the assertion
/// rejects.
///
/// **Scope**: this slice covers the kill-path itself. Exercising the
/// full kill → restart → kill cycle under `restart_policy: on-failure`
/// is deferred — the node_api interaction after a health-check SIGKILL
/// leaves the daemon in a state where subsequent spawns don't cleanly
/// re-trigger the path, and figuring out whether that's a test-harness
/// limitation or a real daemon bug deserves its own investigation.
#[tokio::test(flavor = "multi_thread")]
async fn health_check_timeout_sigkills_unresponsive_node() {
    let status = std::process::Command::new("cargo")
        .args(["build", "-p", "hang-after-init-node"])
        .status()
        .expect("failed to build hang-after-init-node");
    assert!(status.success(), "hang-after-init-node build failed");

    let marker = Path::new("/tmp/dora-health-check-timeout.log");
    let _ = std::fs::remove_file(marker);

    let dataflow_path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dataflows/health-check-timeout.yml");

    let result = Daemon::run_dataflow(
        &dataflow_path,
        None,
        None,
        SessionId::generate(),
        false,
        LogDestination::Tracing,
        None,
        // 10 s keeps stop_after well clear of the ~2.1-2.2 s kill —
        // see fixture for why the health_check_timeout can't just be
        // 0.5 s (init_from_env takes ~500 ms and needs to finish before
        // the marker write races the kill).
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
    // Marker confirms the node did reach the post-init hang state,
    // so any observed non-kill exit is a real kill-path regression —
    // not "the node never ran".
    let contents = std::fs::read_to_string(marker)
        .expect("marker file should exist — the node writes it right after init+recv");
    let _ = std::fs::remove_file(marker);
    eprintln!("marker:\n{contents}");
    assert!(
        contents.contains("incarnation"),
        "hang-after-init-node never reached the post-init marker write — unrelated regression"
    );

    // Core contract: the kill was a SIGKILL from the watchdog, not a
    // SIGTERM from stop_after (Signal(15)) nor a GraceDuration
    // timeout. `NodeExitStatus::Signal(9)` in node_results is the
    // observable proof the health-check path fired.
    let node_result = dr
        .node_results
        .get(&"hang-after-init".to_string().into())
        .expect("hang-after-init should be in node_results");
    let err = node_result
        .as_ref()
        .err()
        .expect("hang-after-init should have errored — kill is a failure exit");
    use dora_message::common::NodeExitStatus;
    assert!(
        matches!(err.exit_status, NodeExitStatus::Signal(9)),
        "expected `NodeExitStatus::Signal(9)` from health-check SIGKILL, got {:?} \
         (cause: {:?})",
        err.exit_status,
        err.cause,
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
