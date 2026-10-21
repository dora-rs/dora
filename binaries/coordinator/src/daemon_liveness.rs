//! Daemon heartbeats and disconnects: deciding when a daemon is gone, what
//! that does to the dataflows and builds it was part of, and reclaiming a
//! still-running dataflow when the daemon comes back.

use crate::{
    broadcast_all_nodes_ready, close_topic_subscribers_on_finish, finalize_build,
    handlers::{send_heartbeat_message, send_log_message},
    state::{CachedResult, DaemonConnections, PendingRestart, RunningBuild, RunningDataflow},
};
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
use dora_core::{descriptor::DescriptorExt, uhlc::HLC};
use dora_message::{
    BuildId, DataflowId,
    common::DaemonId,
    coordinator_to_cli::{ControlRequestReply, DataflowResult, LogLevel, LogMessage},
    coordinator_to_daemon::{DaemonCoordinatorEvent, Timestamped},
};
use eyre::{Result, WrapErr, eyre};
use indexmap::IndexMap;
use std::{
    collections::{BTreeSet, HashMap},
    sync::Arc,
    time::Duration,
};

/// Consecutive heartbeat-send timeouts after which a daemon is disconnected.
///
/// A heartbeat send is a write to the daemon's *bounded* command channel — the
/// same channel every coordinator→daemon command uses. A single send timeout is
/// transient backpressure (a large spawn, a burst of control commands) and must
/// not disconnect a daemon that is otherwise alive and still sending its own
/// inbound heartbeats; disconnecting on one timeout tears down all of its
/// dataflows over momentary congestion.
///
/// This backstop bounds only the **no-traffic** wedge: a daemon whose command
/// channel stays full while *no* command is routed to it is disconnected after
/// this many consecutive heartbeat ticks (~30 s at the 3 s interval, matching
/// the 30 s inbound-heartbeat liveness horizon). It does **not** bound the case
/// where a command *is* routed to a wedged daemon first: `send_and_receive`
/// awaits the same channel with no timeout, so that send blocks the
/// single-threaded coordinator event loop before any further heartbeat tick can
/// advance this counter. Bounding that enqueue is a separate change (it alters
/// command-dispatch failure semantics); see the PR discussion.
pub(crate) const MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS: u32 = 10;

/// Classification of one heartbeat-send attempt.
pub(crate) enum HeartbeatSendOutcome {
    /// The heartbeat was enqueued for delivery.
    Delivered,
    /// The send returned an error: the command channel's receiver has been
    /// dropped (the daemon's WS writer task is gone) or — cosmetically — the
    /// heartbeat could not be serialized. Either way the send cannot succeed,
    /// so disconnect immediately.
    SendFailed,
    /// The 500 ms deadline elapsed while the bounded command channel was full
    /// (backpressure), which does not by itself mean the daemon is dead.
    TimedOut,
}

/// Fold a heartbeat-send outcome and the running count of consecutive timeouts
/// into the next count and whether to disconnect the daemon.
///
/// - `Delivered` clears the timeout streak and keeps the daemon.
/// - `SendFailed` disconnects immediately and clears the streak.
/// - `TimedOut` increments the streak and disconnects only once it reaches
///   [`MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS`], so transient backpressure is
///   tolerated while a persistently wedged channel is still torn down.
pub(crate) fn heartbeat_disconnect_decision(
    outcome: HeartbeatSendOutcome,
    consecutive_timeouts: u32,
) -> (u32, bool) {
    match outcome {
        HeartbeatSendOutcome::Delivered => (0, false),
        HeartbeatSendOutcome::SendFailed => (0, true),
        HeartbeatSendOutcome::TimedOut => {
            let streak = consecutive_timeouts.saturating_add(1);
            (streak, streak >= MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS)
        }
    }
}

/// Send one heartbeat to `connection` with a 500 ms deadline and return, tagged
/// with `machine_id`, whether the watchdog should disconnect the daemon.
///
/// A send *timeout* means the bounded command channel is momentarily full, not
/// that the daemon is dead — so a single timeout no longer disconnects a live
/// daemon (which would fail all of its dataflows over transient congestion).
/// Only a failed send disconnects immediately; a persistent timeout streak
/// escalates via [`heartbeat_disconnect_decision`]. The counter lives on
/// `connection`, and each future here holds a distinct `&mut` (one per daemon),
/// so updating it from inside the concurrent `join_all` is sound.
pub(crate) async fn send_heartbeat_with_timeout(
    machine_id: DaemonId,
    connection: &mut crate::state::DaemonConnection,
    timestamp: dora_core::uhlc::Timestamp,
) -> (DaemonId, bool) {
    let outcome = match tokio::time::timeout(
        Duration::from_millis(500),
        send_heartbeat_message(connection, timestamp),
    )
    .await
    {
        Ok(Ok(())) => HeartbeatSendOutcome::Delivered,
        Ok(Err(err)) => {
            tracing::warn!("heartbeat send to daemon at `{machine_id}` failed: {err:?}");
            HeartbeatSendOutcome::SendFailed
        }
        Err(_elapsed) => HeartbeatSendOutcome::TimedOut,
    };
    let (streak, disconnect) =
        heartbeat_disconnect_decision(outcome, connection.consecutive_heartbeat_send_timeouts);
    connection.consecutive_heartbeat_send_timeouts = streak;
    // `streak > 0` only for a `TimedOut` outcome.
    if streak > 0 {
        if disconnect {
            tracing::error!(
                "heartbeat send to daemon at `{machine_id}` timed out {streak} times in a row \
                 (command channel persistently full); disconnecting"
            );
        } else {
            tracing::warn!(
                "heartbeat send to daemon at `{machine_id}` timed out (command channel full, \
                 streak {streak}/{MAX_CONSECUTIVE_HEARTBEAT_SEND_TIMEOUTS}); not disconnecting a \
                 live daemon (last inbound heartbeat {:?} ago)",
                connection.last_heartbeat.elapsed()
            );
        }
    }
    (machine_id, disconnect)
}

/// Remove disconnected daemon ids from all in-memory dataflow membership sets.
///
/// This intentionally does not resolve `spawn_result`: the spawn timeout
/// watchdog remains the single path that releases spawn waiters for
/// disconnect-mid-spawn cases.
/// Action the daemon-disconnect cleanup asks the (async) caller to perform for
/// a dataflow that was already past spawn when a daemon it depended on vanished.
/// See #2028.
pub(crate) enum DisconnectAction {
    /// The last daemon we were awaiting `ReadyOnDaemon` from disconnected, but
    /// survivors remain — release the start barrier so they don't block forever.
    ReleaseReadyBarrier(DataflowId),
    /// Every daemon running the dataflow disconnected — begin a reclaim window
    /// rather than failing terminally. The daemon may have only transiently lost
    /// the coordinator (a heartbeat blip, a coordinator restart) while its node
    /// processes keep running (dora-rs/dora#2029); marking the dataflow
    /// `Recovering` lets the reconnecting daemon's `DaemonStatusReport` reconcile
    /// it back to `Running`. If no daemon reclaims it within the recovery window,
    /// the existing recovery-timeout path fails it terminally (dora-rs/dora#2028).
    ReclaimOrphaned(DataflowId),
}

/// Cancel a pending restart for `uuid`, if one exists: removes it from
/// `pending_restarts` and replies to the parked restart caller with `Err`
/// explaining why. No-op if there is no pending restart for `uuid`.
pub(crate) fn cancel_pending_restart(
    pending_restarts: &mut HashMap<DataflowId, PendingRestart>,
    uuid: DataflowId,
    reason: impl std::fmt::Display,
) {
    if let Some(restart) = pending_restarts.remove(&uuid) {
        let reason = reason.to_string();
        tracing::warn!(dataflow = %uuid, "cancelling pending restart: {reason}");
        let _ = restart.reply_sender.send(Err(eyre!("{reason}")));
    }
}

pub(crate) fn cleanup_disconnected_daemons_from_running_dataflows(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    disconnected: &BTreeSet<DaemonId>,
    pending_restarts: &mut HashMap<DataflowId, PendingRestart>,
) -> Vec<DisconnectAction> {
    let mut actions = Vec::new();
    let mut affected_uuids = Vec::new();
    for df in running_dataflows.values_mut() {
        let pending_was_nonempty = !df.pending_daemons.is_empty();
        let mut affected = false;
        for daemon_id in disconnected {
            affected |= df.daemons.remove(daemon_id);
            affected |= df.pending_daemons.remove(daemon_id);
            affected |= df.pending_spawn_results.remove(daemon_id);
        }
        if !affected {
            continue;
        }
        affected_uuids.push(df.uuid);
        // Only act on dataflows that already spawned successfully. While
        // `spawn_result` is still pending, the spawn-timeout watchdog
        // (`check_spawn_timeouts`) owns the disconnect-mid-spawn case, so we
        // must not race it here.
        let spawned_ok = df.spawn_result.is_cached_ok();
        if df.daemons.is_empty() {
            tracing::warn!(
                dataflow = %df.uuid,
                "all daemons disconnected - entering reclaim window (waiting for daemon reconnect)"
            );
            if spawned_ok {
                actions.push(DisconnectAction::ReclaimOrphaned(df.uuid));
            }
        } else if spawned_ok && pending_was_nonempty && df.pending_daemons.is_empty() {
            // The last daemon we were waiting on for `ReadyOnDaemon` vanished
            // via disconnect; `AllNodesReady` would otherwise never fire.
            actions.push(DisconnectAction::ReleaseReadyBarrier(df.uuid));
        }
    }
    // Drain pending restarts for affected dataflows: the disconnected
    // daemon(s) will never send DataflowFinishedOnDaemon, so any caller
    // waiting on a deferred restart would hang indefinitely (#2082 H1).
    for uuid in &affected_uuids {
        cancel_pending_restart(
            pending_restarts,
            *uuid,
            format!("daemon disconnected while restart was pending for dataflow `{uuid}`"),
        );
    }
    actions
}

/// Re-establish the in-memory [`RunningDataflow`] for a dataflow a daemon has
/// just re-reported as running after a reconnect (dora-rs/dora#2029 P1).
///
/// `begin_orphaned_dataflow_reclaim` (and a coordinator restart) leave the
/// dataflow only in the persisted store; the live control plane — `dora list`,
/// `stop`, `logs`, `node`, `param` — all read `running_dataflows`, so without
/// this the surviving nodes would be invisible and unmanageable even though the
/// store says `Running`. If the entry is still present (a multi-daemon dataflow
/// whose other daemons are live), just relink this daemon's share; otherwise
/// reconstruct it from the persisted record + the daemon's report.
/// Returns `true` if the ready barrier already released for this dataflow,
/// meaning the caller owes this daemon a replay (dora-rs/dora#2998).
#[must_use]
pub(crate) fn reestablish_running_dataflow(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    record: &dora_coordinator_store::DataflowRecord,
    daemon_id: &DaemonId,
    reported_nodes: &[dora_core::config::NodeId],
) -> bool {
    if let Some(df) = running_dataflows.get_mut(&record.uuid) {
        df.daemons.insert(daemon_id.clone());
        df.pending_daemons.remove(daemon_id);
        df.pending_spawn_results.remove(daemon_id);
        for node in reported_nodes {
            df.node_to_daemon.insert(node.clone(), daemon_id.clone());
        }
        // The barrier may have released while this daemon was gone. The
        // broadcast only reaches `daemons` as it stands at that moment, and
        // a disconnected daemon has been removed from it, so nothing else
        // will ever tell this one (dora-rs/dora#2998).
        return df.ready_barrier_released;
    }

    let descriptor: dora_message::descriptor::Descriptor =
        match serde_json::from_str(&record.descriptor_json) {
            Ok(d) => d,
            Err(e) => {
                tracing::warn!(
                    "cannot re-establish running dataflow {}: failed to parse descriptor: {e}",
                    record.uuid
                );
                return false;
            }
        };
    let nodes = match descriptor.resolve_aliases_and_set_defaults() {
        Ok(n) => n,
        Err(e) => {
            tracing::warn!(
                "cannot re-establish running dataflow {}: failed to resolve nodes: {e}",
                record.uuid
            );
            return false;
        }
    };
    running_dataflows.insert(
        record.uuid,
        RunningDataflow::recovered(record, descriptor, nodes, daemon_id.clone(), reported_nodes),
    );
    tracing::info!(
        "re-established running dataflow {} in live coordinator state after daemon {daemon_id} reconnect",
        record.uuid
    );
    // The rebuilt entry carries the persisted release, so this path owes the
    // replay exactly as the relink path does. Returning a flat `false` here is
    // what left orphan-reclaim and coordinator-restart reconnects hanging.
    record.ready_barrier_released
}

pub(crate) fn status_report_should_stop_orphan(status: &StoreDataflowStatus) -> bool {
    matches!(
        status,
        StoreDataflowStatus::Failed { terminal: true, .. } | StoreDataflowStatus::Succeeded
    )
}

/// Tell a single daemon to stop a dataflow the coordinator has terminally given
/// up on but the daemon still reports as running (dora-rs/dora#2029 P3).
///
/// Closes the orphan window where the daemon's reconnect window outlives the
/// coordinator's recovery timeout: the daemon comes back with surviving nodes
/// *after* the coordinator already failed the dataflow terminally, so the
/// reconcile won't re-adopt it (the terminal verdict is deliberately preserved).
/// Rather than leave those now-unmanageable nodes running, ask the daemon to
/// stop them.
///
/// Fire-and-forget (`send`, not `send_and_receive`): the reconcile loop must not
/// block on the daemon's stop round-trip, and no reply is needed (the daemon
/// processes a `daemon_event` without replying).
pub(crate) async fn stop_orphaned_dataflow_on_daemon(
    dataflow_id: DataflowId,
    daemon_id: &DaemonId,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
) {
    let Some(connection) = daemon_connections.get_mut(daemon_id) else {
        return;
    };
    let message = match serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::StopDataflow {
            dataflow_id,
            grace_duration: None,
            force: false,
        },
        timestamp: clock.new_timestamp(),
    }) {
        Ok(m) => m,
        Err(e) => {
            tracing::warn!("failed to serialize orphan-stop for dataflow {dataflow_id}: {e}");
            return;
        }
    };
    match connection.send(&message).await {
        Ok(()) => tracing::warn!(
            "told daemon {daemon_id} to stop orphaned dataflow {dataflow_id}: it reconnected with \
             the dataflow still running after the coordinator already failed it terminally (#2029)"
        ),
        Err(e) => tracing::warn!(
            "failed to tell daemon {daemon_id} to stop orphaned dataflow {dataflow_id}: {e}"
        ),
    }
}

/// Begin a reclaim window for a dataflow whose every daemon disconnected *after*
/// it had already spawned successfully. This is the running-dataflow counterpart
/// of [`check_spawn_timeouts`] (which only fires while `spawn_result` is pending),
/// but unlike that watchdog it does **not** fail the dataflow terminally: a
/// daemon often only transiently loses the coordinator (a heartbeat blip or a
/// coordinator restart) while its node processes keep running
/// (dora-rs/dora#2029).
///
/// So instead of the terminal teardown tail (persist `Failed { terminal: true }`,
/// synthesize a `Failed` result, archive), this:
/// - persists `Recovering`, so a reconnecting daemon's `DaemonStatusReport` can
///   reconcile it back to `Running` (the reconcile path skips archived/terminal
///   records — hence we must NOT archive here and must NOT mark it terminal);
/// - releases parked `dora stop` waiters so they don't hang (deadlock #2 of
///   dora-rs/dora#2028) — a `stop` racing a disconnect still returns;
/// - removes the dataflow from `running_dataflows`; the live entry is rebuilt by
///   `reestablish_running_dataflow` when the daemon reconnects and re-reports it
///   (so `dora list` / `stop` / `logs` work again — dora-rs/dora#2029 P1).
///
/// If no daemon reclaims it, the recovery-timeout path (the `Recovering -> Failed`
/// sweep) fails it terminally after `RECOVERY_TIMEOUT_SECS`.
///
/// Idempotent: `running_dataflows.remove` returning `None` makes a repeated call
/// a no-op.
pub(crate) async fn begin_orphaned_dataflow_reclaim(
    uuid: DataflowId,
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    clock: &HLC,
    store: &dyn dora_coordinator_store::CoordinatorStore,
) {
    let Some(mut df) = running_dataflows.remove(&uuid) else {
        return;
    };
    let msg = "all daemons running this dataflow disconnected; \
               waiting for daemon reconnect (Recovering)"
        .to_string();

    if let Err(e) = df
        .make_record(StoreDataflowStatus::Recovering)
        .and_then(|r| store.put_dataflow(&r))
    {
        tracing::warn!(dataflow = %uuid, "failed to persist reclaim (Recovering) state: {e}");
    }

    send_log_message(
        &mut df.log_subscribers,
        &LogMessage {
            build_id: None,
            dataflow_id: Some(uuid),
            node_id: None,
            daemon_id: None,
            level: LogLevel::Warn.into(),
            target: Some("coordinator".into()),
            module_path: None,
            file: None,
            line: None,
            message: msg,
            timestamp: clock.new_timestamp().get_time().to_system_time().into(),
            fields: None,
        },
    )
    .await;

    close_topic_subscribers_on_finish(&mut df);

    // Release any in-flight `dora stop` waiters so they don't hang across the
    // reclaim window (deadlock #2 of #2028). There is no synthesized failure
    // result here (the dataflow is recovering, not failed), so report an empty
    // OK result.
    let stop_reply = ControlRequestReply::DataflowStopped {
        uuid,
        result: DataflowResult::ok_empty(uuid, clock.new_timestamp()),
    };
    for sender in df.stop_reply_senders.drain(..) {
        let _ = sender.send(Ok(stop_reply.clone()));
    }
}

/// Execute the [`DisconnectAction`]s produced by
/// [`cleanup_disconnected_daemons_from_running_dataflows`]: release the start
/// barrier for dataflows whose last pending daemon vanished, and begin the
/// reclaim window for dataflows that lost every daemon. Runs at the (async)
/// caller after the synchronous set-pruning. See #2028 / #2029.
pub(crate) async fn apply_disconnect_actions(
    actions: Vec<DisconnectAction>,
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: &Arc<HLC>,
) -> eyre::Result<()> {
    for action in actions {
        match action {
            DisconnectAction::ReleaseReadyBarrier(uuid) => {
                if let Some(df) = running_dataflows.get_mut(&uuid) {
                    broadcast_all_nodes_ready(uuid, df, daemon_connections, store, clock).await?;
                }
            }
            DisconnectAction::ReclaimOrphaned(uuid) => {
                begin_orphaned_dataflow_reclaim(uuid, running_dataflows, clock, store.as_ref())
                    .await;
            }
        }
    }
    Ok(())
}

/// Mirror of [`cleanup_disconnected_daemons_from_running_dataflows`] for
/// `running_builds`: handle daemons that disconnect part-way through a
/// `dora build`.
///
/// A daemon still listed in a build's `pending_build_results` disconnected
/// before reporting its `build_result`, so that daemon's part of the build
/// never completed. For each such build we:
///
/// 1. remove the disconnected daemon from `pending_build_results` and record
///    the disconnect in `build.errors`, so the build resolves as failed rather
///    than silently succeeding on the strength of the *other* daemons' results;
/// 2. if that empties `pending_build_results`, finalize the build immediately
///    via [`finalize_build`] (resolve `build_result` and move the entry into
///    `finished_builds`), mirroring the `DataflowBuildResult` finalize branch.
///
/// Without step 2, a build whose *last* pending daemon disconnects would linger
/// in `running_builds` until [`check_build_timeouts`] fires the 20-minute
/// deadline, because the empty-set finalize check lives only in the
/// `DataflowBuildResult` handler and no further report will ever arrive (#1465).
pub(crate) fn cleanup_disconnected_daemons_from_running_builds(
    running_builds: &mut HashMap<BuildId, RunningBuild>,
    finished_builds: &mut IndexMap<BuildId, CachedResult>,
    disconnected: &BTreeSet<DaemonId>,
) {
    let mut emptied = Vec::new();
    for (build_id, build) in running_builds.iter_mut() {
        let mut pruned_pending = false;
        for daemon_id in disconnected {
            if build.pending_build_results.remove(daemon_id) {
                pruned_pending = true;
                build.errors.push(format!(
                    "daemon `{daemon_id}` disconnected before reporting its build result"
                ));
            }
        }
        if pruned_pending && build.pending_build_results.is_empty() {
            emptied.push(*build_id);
        }
    }

    for build_id in emptied {
        let Some(build) = running_builds.remove(&build_id) else {
            continue;
        };
        tracing::warn!(
            build_id = %build_id,
            "finalizing build as failed: a daemon disconnected before reporting its build result",
        );
        // `build.errors` is non-empty (we just recorded a disconnect), so
        // `finalize_build` resolves this as a failed build.
        finalize_build(build_id, build, finished_builds);
    }
}

pub(crate) async fn notify_daemons_about_disconnected_peers(
    disconnected: &BTreeSet<DaemonId>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
) -> Result<()> {
    for disconnected_id in disconnected {
        let msg = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::PeerDaemonDisconnected {
                daemon_id: disconnected_id.clone(),
            },
            timestamp: clock.new_timestamp(),
        })
        .wrap_err("failed to serialize PeerDaemonDisconnected")?;
        for (_id, conn) in daemon_connections.iter_mut() {
            if let Err(err) = conn.send(&msg).await {
                tracing::warn!("failed to notify daemon of peer disconnect: {err}");
            }
        }
    }
    Ok(())
}
