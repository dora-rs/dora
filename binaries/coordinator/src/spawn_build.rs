//! Collecting per-daemon build and spawn results into a dataflow-level
//! outcome, and the heartbeat-driven watchdogs that fail a build or spawn
//! whose daemons never report back.

use crate::{
    MAX_ARCHIVED_DATAFLOWS, MAX_DATAFLOW_RESULTS, MAX_FINISHED_BUILDS, cancel_pending_restart,
    close_topic_subscribers_on_finish,
    handlers::{dataflow_result, send_log_message},
    spawn_result_timeout,
    state::{
        ArchivedDataflow, CachedResult, DaemonConnections, PendingRestart, RunningBuild,
        RunningDataflow,
    },
};
use dora_coordinator_store::CoordinatorStore;
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
use dora_core::uhlc::HLC;
use dora_message::{
    BuildId, DataflowId,
    common::{DaemonId, NodeError, NodeErrorCause, NodeExitStatus},
    coordinator_to_cli::{ControlRequestReply, DataflowResult, LogLevel, LogMessage},
    coordinator_to_daemon::{DaemonCoordinatorEvent, Timestamped},
    daemon_to_coordinator::DataflowDaemonResult,
    id::NodeId,
};
use eyre::{Result, eyre};
use indexmap::IndexMap;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    time::Duration,
};
use uuid::Uuid;

/// Handle `Event::DataflowSpawnResult`, a daemon's async report of how its
/// share of a dataflow spawn went.
///
/// The per-daemon bookkeeping lives in [`handle_spawn_result_ok`] /
/// [`handle_spawn_result_err`]. What's here is the cross-daemon fallout of a
/// *partial* failure: daemons report independently, so one can fail long after
/// another has already started its nodes. Whichever report makes the dataflow
/// terminally failed also has to stop the daemons that did start nodes and
/// tear the dataflow down. Otherwise those nodes keep running unmanaged and
/// `dora list` reports `Running` against a `Failed` store record
/// ([#3134](https://github.com/dora-rs/dora/issues/3134)).
///
/// Either ordering ends up rolled back: a failure arriving last stops the
/// daemons that already succeeded, and a success arriving after the teardown
/// stops itself.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn handle_dataflow_spawn_result(
    dataflow_id: DataflowId,
    daemon_id: DaemonId,
    result: eyre::Result<()>,
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    archived_dataflows: &mut IndexMap<DataflowId, ArchivedDataflow>,
    dataflow_results: &mut IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>>,
    daemon_connections: &mut DaemonConnections,
    pending_restarts: &mut HashMap<DataflowId, PendingRestart>,
    clock: &HLC,
    store: &dyn CoordinatorStore,
) {
    // Set when this event is the terminal spawn failure: the daemons to roll
    // back and the message to report. Computed while the dataflow is borrowed,
    // acted on once that borrow ends.
    let failed_spawn = match running_dataflows.get_mut(&dataflow_id) {
        Some(dataflow) => {
            dataflow.pending_spawn_results.remove(&daemon_id);
            match result {
                Ok(()) => {
                    handle_spawn_result_ok(dataflow, dataflow_id, &daemon_id, store);
                    None
                }
                Err(err) => {
                    // Only the transition to terminal owns the rollback; a late
                    // Err on an already-failed dataflow was torn down by
                    // whichever path failed it first.
                    let is_terminal_transition = dataflow.spawn_result.is_pending();
                    let err_msg = format!("spawn failed on daemon `{daemon_id}`: {err}");
                    handle_spawn_result_err(dataflow, dataflow_id, &daemon_id, err, store);
                    is_terminal_transition.then(|| {
                        // Every daemon no longer waiting on a spawn result: the
                        // ones that reported success, whose nodes are live and
                        // would otherwise be orphaned, plus the failing daemon,
                        // which may have started some nodes before erroring out.
                        let started: BTreeSet<DaemonId> = dataflow
                            .daemons
                            .difference(&dataflow.pending_spawn_results)
                            .cloned()
                            .collect();
                        (started, err_msg)
                    })
                }
            }
        }
        None => {
            // The dataflow is gone, most likely torn down by another daemon's
            // spawn failure. A daemon reporting *success* now has live nodes
            // that nothing else will ever stop, so stop them here.
            if result.is_ok() && archived_dataflows.contains_key(&dataflow_id) {
                tracing::warn!(
                    dataflow = %dataflow_id,
                    daemon = %daemon_id,
                    "spawn succeeded after the dataflow was terminally failed; \
                     stopping the orphaned nodes",
                );
                fire_and_forget_rollback(
                    dataflow_id,
                    &BTreeSet::from([daemon_id]),
                    daemon_connections,
                    clock,
                )
                .await;
            } else {
                tracing::warn!(
                    "received DataflowSpawnResult, but no matching dataflow in `running_dataflows` map"
                );
            }
            None
        }
    };

    if let Some((started_daemons, err_msg)) = failed_spawn {
        teardown_failed_spawn(
            dataflow_id,
            &err_msg,
            &started_daemons,
            running_dataflows,
            archived_dataflows,
            dataflow_results,
            daemon_connections,
            pending_restarts,
            clock,
        )
        .await;
    }
}

/// Maximum number of log messages buffered per dataflow / build while no log
/// subscriber is attached. Once full, newer messages are dropped (the older
/// ones are kept so a subscriber that attaches later still sees the start of
/// the run).
pub(crate) const MAX_BUFFERED_LOG_MESSAGES: usize = 10_000;

/// Buffer `message`, or drop it once `buffer` already holds
/// [`MAX_BUFFERED_LOG_MESSAGES`].
///
/// Returns `true` exactly once — on the message that fills the buffer — so the
/// caller logs a single "buffer full" warning. Further messages are dropped and
/// return `false`, which is what keeps the warning from re-firing on every
/// subsequent message once the buffer is full.
pub(crate) fn buffer_log_message(buffer: &mut Vec<LogMessage>, message: LogMessage) -> bool {
    if buffer.len() < MAX_BUFFERED_LOG_MESSAGES {
        let now_full = buffer.len() == MAX_BUFFERED_LOG_MESSAGES - 1;
        buffer.push(message);
        now_full
    } else {
        false
    }
}

/// Handle the success arm of `Event::DataflowSpawnResult`.
///
/// Extracted from the inline event loop so the late-arrival guard can be
/// exercised directly by unit tests. Two correctness invariants:
///
/// 1. **Late-Ok guard**: if `spawn_result` is no longer `Pending` (i.e. the
///    timeout watchdog already cached an `Err`), do NOT persist `Running`.
///    Without this guard, a delayed daemon's `Ok` arriving after the
///    watchdog fires would resurrect a terminally-failed dataflow as
///    `Running` in the store — in-memory `spawn_result` would still be
///    `Cached(Err)` (because `set_result` is a no-op on `Cached`), so the
///    store and the in-memory state would silently diverge. Rescue of
///    [#1593](https://github.com/dora-rs/dora/pull/1593) Finding 1 follow-up.
///
/// 2. **All-daemons-succeeded check**: only mark the dataflow Running once
///    every assigned daemon has reported a successful spawn. Preserves the
///    pre-existing handler semantics.
pub(crate) fn handle_spawn_result_ok(
    dataflow: &mut RunningDataflow,
    dataflow_id: DataflowId,
    daemon_id: &DaemonId,
    store: &dyn CoordinatorStore,
) {
    // Guard: if the spawn was already terminally failed (by the watchdog
    // or by another daemon reporting an error), a late Ok must not
    // promote the store status back to Running.
    if !dataflow.spawn_result.is_pending() {
        tracing::warn!(
            dataflow = %dataflow_id,
            daemon = %daemon_id,
            "ignoring late successful spawn_result on a dataflow already \
             terminally failed (e.g. by the spawn-timeout watchdog)",
        );
        return;
    }

    if dataflow.pending_spawn_results.is_empty() {
        tracing::info!("successfully spawned dataflow `{dataflow_id}`");
        dataflow
            .spawn_result
            .set_result(Ok(ControlRequestReply::DataflowSpawned {
                uuid: dataflow_id,
            }));
        if let Err(e) = dataflow
            .make_record(StoreDataflowStatus::Running)
            .and_then(|r| store.put_dataflow(&r))
        {
            tracing::warn!("failed to persist dataflow running: {e}");
        }
    }
}

/// Handle the failure arm of `Event::DataflowSpawnResult`.
///
/// Symmetric with [`handle_spawn_result_ok`]. The late-arrival guard
/// prevents a late daemon `Err` from overwriting the watchdog's (or any
/// other terminal-failure path's) more informative store record with a
/// generic `"spawn failed: <daemon-error>"` message — same data-integrity
/// concern as the Ok guard, just with cosmetic-only consequences instead
/// of resurrection. Also avoids bumping `store_generation` for no
/// observable state change.
pub(crate) fn handle_spawn_result_err(
    dataflow: &mut RunningDataflow,
    dataflow_id: DataflowId,
    daemon_id: &DaemonId,
    err: eyre::Report,
    store: &dyn CoordinatorStore,
) {
    if !dataflow.spawn_result.is_pending() {
        tracing::warn!(
            dataflow = %dataflow_id,
            daemon = %daemon_id,
            "ignoring late failed spawn_result on a dataflow already \
             terminally failed: {err:?}",
        );
        return;
    }

    tracing::warn!("error while spawning dataflow `{dataflow_id}`");
    if let Err(e) = dataflow
        .make_record(StoreDataflowStatus::Failed {
            error: format!("spawn failed: {err}"),
            // Daemon-side spawn error: the daemon authoritatively
            // reported the spawn failed. Terminal so a later daemon
            // status report can't promote this back to Running.
            terminal: true,
        })
        .and_then(|r| store.put_dataflow(&r))
    {
        tracing::warn!("failed to persist dataflow spawn failure: {e}");
    }
    dataflow.spawn_result.set_result(Err(err));
}

/// Fire-and-forget compensating rollback used by [`check_spawn_timeouts`].
///
/// Enqueues `StopDataflow{force: true}` to each succeeded daemon via
/// `connection.send()` (no reply wait). Unlike
/// `run::rollback_spawned_daemons`, this does NOT use `send_and_receive` —
/// see the comment in `check_spawn_timeouts` for why: (a) reply-waiting
/// blocks the heartbeat handler for up to `TCP_READ_TIMEOUT` per wedged
/// daemon, risking false-positive disconnections elsewhere in the cluster;
/// (b) `send_and_receive` is not cancellation-safe (it inserts into
/// `pending_replies` *before* registering its own cleanup), so any
/// external timeout that cancels it leaks state.
///
/// Returns `(daemon_id, error)` pairs for dispatch failures (serialization
/// failure or absent daemon connection). The mpsc `send` itself is
/// near-instantaneous so total wall-clock is bounded regardless of N.
pub(crate) async fn fire_and_forget_rollback(
    dataflow_id: Uuid,
    spawned_daemons: &BTreeSet<DaemonId>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
) -> Vec<(DaemonId, String)> {
    if spawned_daemons.is_empty() {
        return Vec::new();
    }

    let stop_message = match serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::StopDataflow {
            dataflow_id,
            grace_duration: None,
            force: true,
        },
        timestamp: clock.new_timestamp(),
    }) {
        Ok(m) => m,
        Err(e) => {
            // Serialization failure is unlikely but fatal for rollback.
            return spawned_daemons
                .iter()
                .map(|id| (id.clone(), format!("failed to serialize stop message: {e}")))
                .collect();
        }
    };

    let mut errors = Vec::new();
    for daemon_id in spawned_daemons {
        let Some(conn) = daemon_connections.get_mut(daemon_id) else {
            errors.push((daemon_id.clone(), "no daemon connection".to_string()));
            continue;
        };
        if let Err(e) = conn.send(&stop_message).await {
            errors.push((daemon_id.clone(), format!("send failed: {e}")));
        }
    }
    errors
}

/// Resolve a completed build's waiters and cache its result. `build.errors`
/// decides success vs. failure. Shared by the `DataflowBuildResult` handler and
/// [`cleanup_disconnected_daemons_from_running_builds`] so the `finished_builds`
/// cap bookkeeping lives in a single place.
pub(crate) fn finalize_build(
    build_id: BuildId,
    mut build: RunningBuild,
    finished_builds: &mut IndexMap<BuildId, CachedResult>,
) {
    let result = if build.errors.is_empty() {
        Ok(())
    } else {
        Err(format!("build failed: {}", build.errors.join("\n\n")))
    };
    build
        .build_result
        .set_result(Ok(ControlRequestReply::DataflowBuildFinished {
            build_id,
            result,
        }));
    finished_builds.insert(build_id, build.build_result);
    while finished_builds.len() > MAX_FINISHED_BUILDS {
        finished_builds.shift_remove_index(0);
    }
}

/// Scan `running_dataflows` for spawns that have been pending past
/// [`spawn_result_timeout`] and resolve them as terminally failed.
///
/// For each stuck dataflow:
/// 1. Roll back any daemons that already reported a successful spawn
///    (fire-and-forget; failures are logged).
/// 2. Set `spawn_result` to an error so `wait_for_spawn` waiters are
///    unblocked with an actionable message instead of hanging on the
///    client-side RPC deadline.
/// 3. Persist the dataflow as `Failed` so a restarted coordinator does
///    not re-resurrect it.
/// 4. **Tear down in-memory state**: archive the dataflow, drain
///    `stop_reply_senders`, close `topic_subscribers`, send a final
///    "dataflow failed" log to `log_subscribers`, then remove from
///    `running_dataflows`. Mirrors the `DataflowFinishedOnDaemon` teardown
///    so the dataflow is no longer visible to `Check` / `List` /
///    `DaemonStatusReport` reconciliation / `Clean` / etc. as if it
///    were still active. Without this step, those handlers would
///    contradict the watchdog's "terminally failed" verdict
///    (PR #1854 round-5 Findings 1 and 2).
///
/// Idempotent: `CachedResult::set_result` is a no-op on `Cached`, and
/// removed dataflows simply don't reappear in the next pass, so
/// re-running this on subsequent heartbeats is safe.
///
/// FIFO-evict *finished* entries from `dataflow_results` until it is within
/// [`MAX_DATAFLOW_RESULTS`].
///
/// `dataflow_results` is not pure history: a partially-finished multi-daemon
/// dataflow accumulates one entry per daemon while it is still running, and
/// the final success/failure status is computed from the full set once the
/// last daemon finishes. Evicting an entry whose dataflow is still in
/// `running_dataflows` would drop an earlier daemon's failure and let the
/// dataflow be reported as `Succeeded` (dora-rs/dora#2027 review). So only
/// entries for dataflows no longer running are evictable; if every over-cap
/// entry is still active the map is left above the cap rather than corrupting
/// live state (the bound targets finished-history growth, not concurrency).
pub(crate) fn cap_dataflow_results(
    dataflow_results: &mut IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>>,
    running_dataflows: &HashMap<DataflowId, RunningDataflow>,
) {
    while dataflow_results.len() > MAX_DATAFLOW_RESULTS {
        let Some(idx) = dataflow_results
            .keys()
            .position(|uuid| !running_dataflows.contains_key(uuid))
        else {
            break;
        };
        dataflow_results.shift_remove_index(idx);
    }
}

/// Rescue of [#1593](https://github.com/dora-rs/dora/pull/1593)
/// (issue [#1592](https://github.com/dora-rs/dora/issues/1592)).
#[allow(clippy::too_many_arguments)]
pub(crate) async fn check_spawn_timeouts(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    archived_dataflows: &mut IndexMap<DataflowId, ArchivedDataflow>,
    dataflow_results: &mut IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>>,
    daemon_connections: &mut DaemonConnections,
    pending_restarts: &mut HashMap<DataflowId, PendingRestart>,
    clock: &HLC,
    store: &dyn CoordinatorStore,
) {
    let timeout_threshold = spawn_result_timeout();
    // First pass: identify stuck spawns and snapshot the daemon sets we
    // need for rollback. We collect into an owned Vec so the immutable
    // borrow of `running_dataflows` drops before we mutate it below.
    let stuck: Vec<(DataflowId, BTreeSet<DaemonId>, usize)> = running_dataflows
        .iter()
        .filter_map(|(uuid, df)| {
            if df.spawn_result.is_pending() && df.spawn_started_at.elapsed() > timeout_threshold {
                // Daemons assigned to this dataflow that already reported
                // successful spawn (i.e. were removed from
                // `pending_spawn_results`). These are the ones we need to
                // roll back to avoid leaving partial state running.
                let succeeded: BTreeSet<DaemonId> = df
                    .daemons
                    .difference(&df.pending_spawn_results)
                    .cloned()
                    .collect();
                Some((*uuid, succeeded, df.pending_spawn_results.len()))
            } else {
                None
            }
        })
        .collect();

    for (uuid, succeeded_daemons, pending_count) in stuck {
        tracing::warn!(
            dataflow = %uuid,
            timeout_secs = timeout_threshold.as_secs(),
            pending = pending_count,
            succeeded = succeeded_daemons.len(),
            "spawn timeout: releasing waiters and rolling back",
        );

        let err_msg = format!(
            "spawn timed out after {}s; {} daemon(s) never reported \
             spawn_result; rolled back {} previously-started daemon(s)",
            timeout_threshold.as_secs(),
            pending_count,
            succeeded_daemons.len(),
        );

        // Fire the spawn_result error and persist Failed before the teardown
        // below removes the entry from `running_dataflows`.
        if let Some(df) = running_dataflows.get_mut(&uuid) {
            df.spawn_result.set_result(Err(eyre!(err_msg.clone())));
            if let Err(e) = df
                .make_record(StoreDataflowStatus::Failed {
                    error: err_msg.clone(),
                    // Watchdog verdict is terminal: even across coordinator
                    // restarts, a wedged daemon that eventually reports the
                    // dataflow as running must NOT resurrect this record to
                    // Running via the reconcile path (round-8 Finding 1).
                    terminal: true,
                })
                .and_then(|r| store.put_dataflow(&r))
            {
                tracing::warn!(
                    dataflow = %uuid,
                    "failed to persist spawn timeout: {e}",
                );
            }
        }

        teardown_failed_spawn(
            uuid,
            &err_msg,
            &succeeded_daemons,
            running_dataflows,
            archived_dataflows,
            dataflow_results,
            daemon_connections,
            pending_restarts,
            clock,
        )
        .await;
    }
}

/// Roll back the daemons that already started nodes for a dataflow whose spawn
/// just failed terminally, and tear down its in-memory state.
///
/// The caller fires `spawn_result` and persists the `Failed` store record
/// first (the wording differs per failure path), and this handles everything
/// that follows, so the dataflow is terminal from every other handler's point
/// of view (Check, List, reconcile, Clean, ...).
///
/// Shared by the spawn-timeout watchdog ([`check_spawn_timeouts`]) and the
/// asynchronous partial-failure path in the `Event::DataflowSpawnResult`
/// handler, where one daemon reports a spawn error after another has already
/// reported success ([#3134](https://github.com/dora-rs/dora/issues/3134)).
///
/// The rollback is fire-and-forget: it enqueues `StopDataflow{force: true}`
/// on each daemon in `rollback_daemons` WITHOUT awaiting a reply, which
/// avoids two problems at once:
///
/// 1. Cascade-failure risk: a reply-awaiting rollback (the original
///    `run::rollback_spawned_daemons` path) blocks `TCP_READ_TIMEOUT = 30s`
///    per wedged daemon. With N wedged daemons, the heartbeat handler would
///    block ~N*30s, during which heartbeats to *other* healthy daemons aren't
///    dispatched and they trip the 30s disconnect threshold.
/// 2. Cancellation safety: an earlier version wrapped
///    `rollback_spawned_daemons` in `tokio::time::timeout`, but that future
///    cancels mid-`send_and_receive`, which inserts a pending reply *before*
///    registering its own cleanup — the cancellation would leak
///    `pending_replies` entries. `connection.send()` is just an mpsc enqueue;
///    no pending state, no cleanup needed, fully cancellation-safe.
///
/// Trade-off: we don't get per-daemon ack of "stop succeeded". That is
/// acceptable here — the user is already getting a clear error, and unstopped
/// daemons will be reclaimed by daemon-disconnect or operator `dora stop`.
#[allow(clippy::too_many_arguments)]
pub(crate) async fn teardown_failed_spawn(
    uuid: DataflowId,
    err_msg: &str,
    rollback_daemons: &BTreeSet<DaemonId>,
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    archived_dataflows: &mut IndexMap<DataflowId, ArchivedDataflow>,
    dataflow_results: &mut IndexMap<DataflowId, BTreeMap<DaemonId, DataflowDaemonResult>>,
    daemon_connections: &mut DaemonConnections,
    pending_restarts: &mut HashMap<DataflowId, PendingRestart>,
    clock: &HLC,
) {
    let rollback_errors =
        fire_and_forget_rollback(uuid, rollback_daemons, daemon_connections, clock).await;
    if !rollback_errors.is_empty() {
        let rollback_summary = rollback_errors
            .iter()
            .map(|(id, e)| format!("  {id}: {e}"))
            .collect::<Vec<_>>()
            .join("\n");
        tracing::warn!(
            dataflow = %uuid,
            "rollback partial after failed spawn, {} dispatch(es) failed:\n{rollback_summary}",
            rollback_errors.len(),
        );
    }

    let Some(mut df) = running_dataflows.remove(&uuid) else {
        // Concurrent removal — nothing more to do. (Not currently
        // reachable from any other code path; defensive.)
        return;
    };

    // `PendingRestart` is keyed by this (old) UUID: `initiate_restart` only
    // requires the UUID to be present in `running_dataflows`, which it is
    // while the spawn is still in flight — i.e. a restart was requested for
    // this dataflow before the spawn gave up. Without this, the entry would
    // never drain (only `DataflowFinishedOnDaemon` and the daemon-disconnect
    // path do), permanently wedging both the parked restart caller and any
    // future `Stop` for this UUID.
    cancel_pending_restart(
        pending_restarts,
        uuid,
        format!("dataflow `{uuid}`'s spawn failed while a restart was pending: {err_msg}"),
    );

    // Final log message to anyone subscribed.
    send_log_message(
        &mut df.log_subscribers,
        &LogMessage {
            build_id: None,
            dataflow_id: Some(uuid),
            node_id: None,
            daemon_id: None,
            level: LogLevel::Error.into(),
            target: Some("coordinator".into()),
            module_path: None,
            file: None,
            line: None,
            message: err_msg.to_owned(),
            timestamp: clock.new_timestamp().get_time().to_system_time().into(),
            fields: None,
        },
    )
    .await;

    // Close topic subscribers so attached clients see a clean end-of-
    // stream rather than hanging.
    close_topic_subscribers_on_finish(&mut df);

    // Synthesize a `dataflow_results` entry so:
    //   - `dora list` shows the dataflow as Failed (instead of
    //     disappearing entirely — round-6 Finding 1)
    //   - `dora stop <uuid>` returns DataflowStopped via the early-
    //     return at the Stop handler (instead of "no known running
    //     dataflow" — round-6 Finding 3)
    //   - Late `DataflowFinishedOnDaemon` events can merge into the
    //     same entry rather than being silently discarded (round-6
    //     Finding 2; merge logic lives in that handler's Vacant arm).
    //
    // For each daemon that was assigned to this dataflow, emit a
    // per-daemon `DataflowDaemonResult` with one `Err(NodeError {
    // cause: FailedToSpawn(..) })` entry per node assigned to that
    // daemon. This makes
    // `results.values().all(DataflowDaemonResult::is_ok) == false`,
    // which classifies the dataflow as `Failed` in
    // `DataflowList` (lib.rs ~1019).
    //
    // **Crucially, iterate `df.node_to_daemon` for the daemon set,
    // not `df.daemons`**: the daemon-disconnect cleanup path at
    // `lib.rs:1893-1899` removes disconnected daemons from
    // `df.daemons` but leaves `df.node_to_daemon` (the original
    // assignment) intact. If we iterated `df.daemons` here and the
    // disconnect-mid-spawn case had emptied it, the result map
    // would be empty and List's classification check
    // `results.values().all(is_ok)` would be vacuously true,
    // misclassifying the dataflow as `Finished` (round-7
    // Finding 2). The original assignment is the right source of
    // truth for "what daemons should have been running this".
    let synth_results = synthesize_failed_dataflow_results(&df, uuid, err_msg, clock);
    // Insert before draining stop senders so the DataflowResult
    // they receive carries the synthesized node-level errors.
    dataflow_results
        .entry(uuid)
        .or_default()
        .extend(synth_results);

    // Drain `stop_reply_senders`. Any in-flight `dora stop` calls were
    // waiting for the dataflow to stop; that's effectively what just
    // happened (the spawn failed terminally and the dataflow will not
    // proceed). Use `dataflow_result` (the helper used by the normal
    // DataflowFinishedOnDaemon path) over the synthesized entry so
    // the reply carries the per-node errors that `dora list` /
    // `dora check` will also surface.
    let stop_reply = ControlRequestReply::DataflowStopped {
        uuid,
        result: dataflow_results
            .get(&uuid)
            .map(|r| dataflow_result(r, uuid, clock))
            .unwrap_or_else(|| DataflowResult::ok_empty(uuid, clock.new_timestamp())),
    };
    for sender in df.stop_reply_senders.drain(..) {
        let _ = sender.send(Ok(stop_reply.clone()));
    }

    // Archive so `dora list` still surfaces the dataflow's name +
    // descriptor for users investigating after the fact. Capped to
    // prevent unbounded growth — uses the same MAX_ARCHIVED_DATAFLOWS
    // limit as the DataflowFinishedOnDaemon teardown.
    archived_dataflows
        .entry(uuid)
        .or_insert_with(|| ArchivedDataflow::from(&df));
    while archived_dataflows.len() > MAX_ARCHIVED_DATAFLOWS {
        archived_dataflows.shift_remove_index(0);
    }

    // Cap LAST: the synthesized entry was just read for the stop reply and
    // archival above, so evicting it now (if it is over-cap finished
    // history) can't misreport this dataflow as `ok_empty`. `uuid` is no
    // longer in `running_dataflows` here.
    cap_dataflow_results(dataflow_results, running_dataflows);
    // `df` drops here, releasing all remaining resources.
}

/// Scan `running_builds` for builds that have been pending past
/// [`build_result_timeout`] and resolve them as terminally failed.
///
/// Mirrors [`check_spawn_timeouts`] but for builds, and is simpler:
/// there is no per-daemon "succeeded" state to roll back — a timed-out
/// build is just released. Daemons' local build artifacts (if any)
/// stay on disk and can be reused by a future `dora build`; the user
/// gets an actionable error immediately instead of a hung
/// `wait_for_build`.
///
/// For each stuck build the watchdog:
/// 1. Removes the entry from `running_builds`.
/// 2. Sets `build_result` to `Err` with a clear timeout message so
///    already-registered `wait_for_build` waiters are released.
/// 3. Sends a final log line to subscribers (`dora build --attach`).
/// 4. Inserts the cached result into `finished_builds` (FIFO-capped)
///    so a `wait_for_build` registered AFTER the watchdog fired also
///    receives the cached error rather than "unknown build id".
///
/// Idempotent: `CachedResult::set_result` is a no-op on already-Cached
/// values, and a build removed from `running_builds` simply doesn't
/// reappear in the next heartbeat tick's filter. Late
/// `DataflowBuildResult` replies after this fires fall into the
/// `running_builds.get_mut → None` arm in the event handler (warn
/// only, no resurrection).
///
/// Rescue of [#1465](https://github.com/dora-rs/dora/issues/1465).
pub(crate) async fn check_build_timeouts(
    running_builds: &mut HashMap<BuildId, RunningBuild>,
    finished_builds: &mut IndexMap<BuildId, CachedResult>,
    clock: &HLC,
    timeout_threshold: Duration,
) {
    let stuck: Vec<(BuildId, usize)> = running_builds
        .iter()
        .filter_map(|(id, build)| {
            if build.build_result.is_pending()
                && build.build_started_at.elapsed() > timeout_threshold
            {
                Some((*id, build.pending_build_results.len()))
            } else {
                None
            }
        })
        .collect();

    for (build_id, pending_count) in stuck {
        tracing::warn!(
            build_id = %build_id,
            timeout_secs = timeout_threshold.as_secs(),
            pending = pending_count,
            "build timeout: releasing wait_for_build waiters",
        );
        let Some(mut build) = running_builds.remove(&build_id) else {
            // Concurrent removal — nothing to do. (Single-task event
            // loop makes this unreachable today; defensive.)
            continue;
        };
        let err_msg = format!(
            "build timed out after {}s; {} daemon(s) never reported build_result",
            timeout_threshold.as_secs(),
            pending_count,
        );
        build.build_result.set_result(Err(eyre!(err_msg.clone())));

        // Final log to subscribers so attached `dora build --attach`
        // sessions see a clean end-of-stream rather than hanging.
        send_log_message(
            &mut build.log_subscribers,
            &LogMessage {
                build_id: Some(build_id),
                dataflow_id: None,
                node_id: None,
                daemon_id: None,
                level: LogLevel::Error.into(),
                target: Some("coordinator".into()),
                module_path: None,
                file: None,
                line: None,
                message: err_msg.clone(),
                timestamp: clock.new_timestamp().get_time().to_system_time().into(),
                fields: None,
            },
        )
        .await;

        // Insert into finished_builds so a `wait_for_build` registered
        // AFTER the watchdog fired also receives the cached error
        // (handler at the `WaitForBuild` arm above).
        finished_builds.insert(build_id, build.build_result);
        while finished_builds.len() > MAX_FINISHED_BUILDS {
            finished_builds.shift_remove_index(0);
        }
    }
}

/// Build the per-daemon `DataflowDaemonResult` map that classifies a
/// terminally-failed dataflow as `Failed` (every assigned node reported as
/// `Err(FailedToSpawn(err_msg))`).
///
/// Iterates `node_to_daemon` (the original assignment), NOT `daemons`: the
/// disconnect-cleanup path prunes `daemons` but leaves `node_to_daemon` intact,
/// and an empty map would be vacuously classified `Finished` by `DataflowList`
/// (round-7 Finding 2). Falls back to a sentinel entry when `node_to_daemon` is
/// empty so the map is never empty (round-8 Finding 2).
///
/// Shared by the spawn-timeout watchdog and the daemon-disconnect teardown
/// (#2028).
pub(crate) fn synthesize_failed_dataflow_results(
    df: &RunningDataflow,
    uuid: DataflowId,
    err_msg: &str,
    clock: &HLC,
) -> BTreeMap<DaemonId, DataflowDaemonResult> {
    let synth_timestamp = clock.new_timestamp();
    let assigned_daemons: BTreeSet<DaemonId> = df.node_to_daemon.values().cloned().collect();
    let synth_results: BTreeMap<DaemonId, DataflowDaemonResult> = assigned_daemons
        .iter()
        .map(|daemon_id| {
            let nodes_for_daemon: BTreeMap<NodeId, Result<(), NodeError>> = df
                .node_to_daemon
                .iter()
                .filter(|(_, did)| *did == daemon_id)
                .map(|(node_id, _)| {
                    (
                        node_id.clone(),
                        Err(NodeError {
                            timestamp: synth_timestamp,
                            cause: NodeErrorCause::FailedToSpawn(err_msg.to_string()),
                            exit_status: NodeExitStatus::Unknown,
                        }),
                    )
                })
                .collect();
            (
                daemon_id.clone(),
                DataflowDaemonResult {
                    timestamp: synth_timestamp,
                    node_results: nodes_for_daemon,
                },
            )
        })
        .collect();
    if synth_results.is_empty() {
        tracing::warn!(
            dataflow = %uuid,
            "teardown: node_to_daemon was empty; injecting sentinel result \
             so list classification is Failed",
        );
        let mut sentinel = BTreeMap::new();
        let mut node_results = BTreeMap::new();
        // Use `"watchdog"` (no angle brackets): `NodeId::from(invalid_chars)`
        // PANICS via `validate_node_id` (rejects chars outside `[a-zA-Z0-9_.-]`).
        node_results.insert(
            NodeId::from("watchdog".to_string()),
            Err(NodeError {
                timestamp: synth_timestamp,
                cause: NodeErrorCause::FailedToSpawn(err_msg.to_string()),
                exit_status: NodeExitStatus::Unknown,
            }),
        );
        sentinel.insert(
            DaemonId::new(Some("watchdog".to_string())),
            DataflowDaemonResult {
                timestamp: synth_timestamp,
                node_results,
            },
        );
        sentinel
    } else {
        synth_results
    }
}
