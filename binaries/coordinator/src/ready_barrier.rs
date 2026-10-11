//! The all-nodes-ready barrier: releasing a dataflow once every daemon has
//! its nodes subscribed, and replaying that release to a daemon that
//! reconnects afterwards.

use crate::state::{DaemonConnections, RunningDataflow};
use crate::{replay_persisted_params_for_daemon, schedule_param_replay_for_ready_dataflow};
use dora_coordinator_store::DataflowStatus as StoreDataflowStatus;
use dora_core::uhlc::HLC;
use dora_message::{
    DataflowId,
    common::DaemonId,
    coordinator_to_daemon::{DaemonCoordinatorEvent, Timestamped},
};
use eyre::WrapErr;
use std::sync::Arc;

/// The `AllNodesReady` frame for a dataflow, as sent to a daemon.
///
/// Shared by the initial broadcast and the reconnect replay so the two
/// cannot drift — in particular so a replayed barrier carries the same
/// `exited_before_subscribe` the broadcast did (dora-rs/dora#2998).
pub(crate) fn all_nodes_ready_message(
    uuid: DataflowId,
    dataflow: &RunningDataflow,
    clock: &Arc<HLC>,
) -> eyre::Result<Vec<u8>> {
    serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::AllNodesReady {
            dataflow_id: uuid,
            exited_before_subscribe: dataflow.exited_before_subscribe.clone(),
        },
        timestamp: clock.new_timestamp(),
    })
    .wrap_err("failed to serialize AllNodesReady message")
}

/// Build the `AllNodesReady` frame for the *initial* release and record on the
/// dataflow that the barrier has fired.
///
/// The mark and the message are produced together so they cannot drift: a
/// broadcast that forgot to record itself would leave a daemon reconnecting
/// later unaware the barrier is down, which is the whole of #2998.
pub(crate) fn release_barrier_message(
    uuid: DataflowId,
    dataflow: &mut RunningDataflow,
    clock: &Arc<HLC>,
) -> eyre::Result<Vec<u8>> {
    dataflow.ready_barrier_released = true;
    all_nodes_ready_message(uuid, dataflow, clock)
}

/// Persist the ready-barrier release durably.
///
/// The release flag and the failure verdict live on the record produced by
/// `make_record` (`ready_barrier_released` / `barrier_exited_before_subscribe`).
/// The in-memory entry is destroyed by orphan reclaim and by coordinator
/// restart; without a durable record of the release, a daemon that missed the
/// broadcast and reconnects afterwards is never replayed it and hangs for the
/// life of the dataflow (#2998).
///
/// This must happen on a *failed* barrier too -- a reconnecting daemon still has
/// to be told the barrier is down and why. **Only the status is conditional; the
/// write itself is not.** A successful barrier promotes to `Running`; a failed
/// barrier must never promote, so it preserves the record's current status when
/// it can be read and otherwise falls back to `Pending` (a non-promoting status
/// the reconcile path can still advance).
///
/// That fallback deliberately trades status fidelity for release durability: a
/// record that was `Stopping` or `Failed { terminal: true }` on disk is lowered
/// to `Pending`, which a later `DaemonStatusReport` can promote to `Running`.
/// Accepted because the release flag is the load-bearing invariant here -- a
/// lost release parks a daemon's nodes forever, while a wrong status is
/// re-derived from the next daemon report -- and because the successful-barrier
/// arm above already writes `Running` with no read at all.
///
/// Previously the failed path skipped the
/// entire write when the status read failed or returned `None`, which left the
/// in-memory flag `true` but no durable record -- re-opening the #2998 window on
/// exactly the store-I/O-trouble path where durability matters most (#3115).
pub(crate) fn persist_ready_barrier_release(
    uuid: DataflowId,
    dataflow: &mut RunningDataflow,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
) {
    let status = if dataflow.exited_before_subscribe.is_empty() {
        StoreDataflowStatus::Running
    } else {
        match store.get_dataflow(&uuid) {
            Ok(Some(existing)) => existing.status,
            Ok(None) => StoreDataflowStatus::Pending,
            Err(e) => {
                tracing::warn!(
                    dataflow = %uuid,
                    "cannot read status to persist failed barrier, falling back to Pending: {e}"
                );
                StoreDataflowStatus::Pending
            }
        }
    };
    if let Err(e) = dataflow
        .make_record(status)
        .and_then(|r| store.put_dataflow(&r))
    {
        tracing::warn!(dataflow = %uuid, "failed to persist ready-barrier release: {e}");
    }
}

/// Re-send a barrier release that a daemon missed because it was
/// disconnected when the broadcast fired.
///
/// The daemon latches the release and answers every later subscribe from it
/// (dora-rs/dora#2938); without this it never learns the barrier is down and
/// each of its nodes parks in `init_from_env()` for the life of the dataflow.
pub(crate) async fn replay_all_nodes_ready(
    uuid: DataflowId,
    dataflow: &RunningDataflow,
    daemon_id: &DaemonId,
    daemon_connections: &mut DaemonConnections,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: &Arc<HLC>,
) -> eyre::Result<()> {
    let message = all_nodes_ready_message(uuid, dataflow, clock)?;
    let Some(connection) = daemon_connections.get_mut(daemon_id) else {
        tracing::warn!("no daemon connection found for machine `{daemon_id}` to replay barrier");
        return Ok(());
    };
    tracing::info!(
        "replaying AllNodesReady({uuid}) to reconnected daemon `{daemon_id}` \
         (barrier released while it was disconnected)"
    );
    let connection_for_params = connection.clone();
    connection.send(&message).await.wrap_err_with(|| {
        format!("failed to replay AllNodesReady({uuid}) to machine {daemon_id}")
    })?;

    // The original broadcast pairs the barrier with a persisted-parameter
    // replay (`schedule_param_replay_for_ready_dataflow`). This daemon missed
    // both halves, so replaying only the barrier would release its nodes with
    // default state instead of the parameters the operator set.
    let node_ids_on_daemon = nodes_on_daemon(dataflow, daemon_id);
    let store = store.clone();
    let clock = clock.clone();
    let daemon_id = daemon_id.clone();
    tokio::spawn(async move {
        replay_persisted_params_for_daemon(
            uuid,
            daemon_id,
            node_ids_on_daemon,
            store,
            connection_for_params,
            clock,
        )
        .await;
    });
    Ok(())
}

/// Broadcast `AllNodesReady` to every daemon running part of `dataflow` and
/// schedule the persisted-parameter replay. Extracted from the `ReadyOnDaemon`
/// handler so the disconnect/cleanup path can also release the start barrier
/// when the last *pending* daemon goes away via disconnect rather than
/// `ReadyOnDaemon` (see issue #2028).
pub(crate) async fn broadcast_all_nodes_ready(
    uuid: DataflowId,
    dataflow: &mut RunningDataflow,
    daemon_connections: &mut DaemonConnections,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: &Arc<HLC>,
) -> eyre::Result<()> {
    tracing::debug!("sending all nodes ready message to daemons");
    let message = release_barrier_message(uuid, dataflow, clock)?;

    persist_ready_barrier_release(uuid, dataflow, store);

    // notify all machines that run parts of the dataflow.
    //
    // This is best-effort per daemon: a broken (but not yet dropped)
    // connection makes `send` fail, but that must not tear down coordination
    // for every other dataflow and daemon. `ready_barrier_released` was
    // already set and persisted above, so a daemon that misses this frame is
    // replayed on reconnect (#2998) -- a dropped send is recoverable. Log and
    // move on instead of aborting the coordinator's event loop (the same
    // best-effort posture the multi-daemon stop path already takes), and keep
    // notifying the remaining daemons rather than stopping at the first
    // failure. A fully-gone daemon takes the `None` branch above.
    for daemon_id in &dataflow.daemons {
        let Some(connection) = daemon_connections.get_mut(daemon_id) else {
            tracing::warn!("no daemon connection found for machine `{daemon_id}`");
            continue;
        };
        if let Err(err) = connection.send(&message).await {
            tracing::warn!(
                "failed to send AllNodesReady({uuid}) message to machine {daemon_id}: {err} \
                 (will be replayed on reconnect)"
            );
        }
    }

    schedule_param_replay_for_ready_dataflow(
        uuid,
        dataflow,
        daemon_connections,
        store.clone(),
        clock.clone(),
    );
    Ok(())
}

/// The nodes of `dataflow` assigned to `daemon_id`.
///
/// Shared by the initial param replay and the reconnect replay so both select
/// the same set: a daemon replayed the barrier must be replayed exactly the
/// parameters the broadcast would have sent it (#2998).
pub(crate) fn nodes_on_daemon(
    dataflow: &RunningDataflow,
    daemon_id: &DaemonId,
) -> Vec<dora_core::config::NodeId> {
    dataflow
        .node_to_daemon
        .iter()
        .filter(|(_, assigned)| *assigned == daemon_id)
        .map(|(node_id, _)| node_id.clone())
        .collect()
}
