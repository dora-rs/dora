//! Dataflow parameters: forwarding `SetParam`/`DeleteParam` to daemons and
//! replaying persisted parameters to a daemon that (re)joins a dataflow.

use crate::nodes_on_daemon;
use crate::state::{DaemonConnections, RunningDataflow};
use dora_core::uhlc::HLC;
use dora_message::{DataflowId, common::DaemonId, daemon_to_coordinator::DaemonCoordinatorReply};
use eyre::eyre;
use serde::Serialize;
use serde_json::value::RawValue;
use std::sync::Arc;

pub(crate) struct ParamReplayItem {
    pub(crate) node_id: dora_core::config::NodeId,
    pub(crate) key: String,
}

#[derive(Debug, Default)]
pub(crate) struct ParamReplaySummary {
    pub(crate) attempted: usize,
    pub(crate) failed: usize,
}

/// State catch-up for a daemon whose ack is behind the pruned part of the
/// state log, so the entries it missed can no longer be sent to it.
///
/// It gets the full persisted state instead, from a param replay: the one
/// the caller already started for it in the same status report
/// (`params_replay_started`, the ready-barrier replay), or the one that
/// releasing the barrier sends to every daemon of the dataflow, or else one
/// started here. Replaying a second time would only send the same store
/// contents again (#3684).
///
/// The ack advances to the current `state_log_sequence` right away, without
/// waiting for the replay: every entry up to it is already in the store
/// (`dora param set`/`delete` persist before appending to the log), and the
/// replay reads each param from the store when it queues it. Later changes
/// are forwarded and acked one by one (`ack_applied_state_log_entry`), which
/// only works once the ack is past the pruned gap. Left behind it, the
/// daemon would also hold back log pruning for every daemon.
///
/// A failed replay leaves the daemon's params behind the store until it
/// reconnects, as before: the daemon sends one status report per
/// connection, so nothing retried a failed replay earlier either, and the
/// reconnect replays every param regardless of the ack.
pub(crate) fn catch_up_daemon_behind_pruned_log(
    dataflow_id: DataflowId,
    dataflow: &mut RunningDataflow,
    daemon_id: &DaemonId,
    params_replay_started: bool,
    daemon_connections: &DaemonConnections,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: &Arc<HLC>,
) {
    if params_replay_started {
        tracing::info!(
            "state catch-up: log pruned for dataflow {dataflow_id}; daemon {daemon_id} \
             gets every persisted param from the ready-barrier replay"
        );
    } else if !dataflow.ready_barrier_released {
        tracing::info!(
            "state catch-up: log pruned for dataflow {dataflow_id}; daemon {daemon_id} \
             gets every persisted param when the ready barrier releases"
        );
    } else if spawn_param_replay_to_daemon(
        dataflow_id,
        dataflow,
        daemon_id,
        daemon_connections,
        store,
        clock,
    ) {
        tracing::info!(
            "state catch-up: log pruned for dataflow {dataflow_id}, \
             falling back to full param replay for daemon {daemon_id}"
        );
    } else {
        tracing::warn!(
            "failed to run fallback replay for dataflow {dataflow_id}: \
             daemon {daemon_id} is not connected"
        );
        return;
    }
    dataflow.mark_daemon_caught_up(daemon_id);
}

/// List the persisted params of every node on a daemon, flattened into
/// replay items. Only the keys are kept: the replay re-reads each value under
/// the connection's param write lock, since it may have changed by then.
///
/// Also returns how many nodes' params could not be loaded: the caller must
/// count those as failed, or a store read error would look like "nothing to
/// replay" and let the daemon be marked caught up.
pub(crate) fn collect_param_replay_items(
    dataflow_id: DataflowId,
    node_ids_on_daemon: &[dora_core::config::NodeId],
    store: &dyn dora_coordinator_store::CoordinatorStore,
) -> (Vec<ParamReplayItem>, usize) {
    let mut items = Vec::new();
    let mut load_failures = 0;
    for node_id in node_ids_on_daemon {
        let params = match store.list_node_params(&dataflow_id, node_id) {
            Ok(params) => params,
            Err(err) => {
                tracing::warn!(
                    "failed to load persisted params for {dataflow_id}/{node_id}: {err}"
                );
                load_failures += 1;
                continue;
            }
        };
        for (key, _) in params {
            items.push(ParamReplayItem {
                node_id: node_id.clone(),
                key,
            });
        }
    }
    (items, load_failures)
}

pub(crate) fn build_set_param_message_from_raw_json(
    dataflow_id: DataflowId,
    node_id: &dora_core::config::NodeId,
    key: &str,
    value_json: &[u8],
    timestamp: dora_core::uhlc::Timestamp,
) -> eyre::Result<Vec<u8>> {
    #[derive(Serialize)]
    struct SetParamPayloadRaw<'a> {
        dataflow_id: DataflowId,
        node_id: &'a dora_core::config::NodeId,
        key: &'a str,
        value: &'a RawValue,
    }
    #[derive(Serialize)]
    enum DaemonCoordinatorEventRaw<'a> {
        SetParam(SetParamPayloadRaw<'a>),
    }
    #[derive(Serialize)]
    struct TimestampedDaemonEventRaw<'a> {
        inner: DaemonCoordinatorEventRaw<'a>,
        timestamp: dora_core::uhlc::Timestamp,
    }

    // Parse persisted bytes as raw JSON once, then rely on serde for
    // envelope serialization instead of manual string JSON assembly.
    let value = serde_json::from_slice::<Box<RawValue>>(value_json)
        .map_err(|e| eyre!("invalid persisted param JSON: {e}"))?;

    serde_json::to_vec(&TimestampedDaemonEventRaw {
        inner: DaemonCoordinatorEventRaw::SetParam(SetParamPayloadRaw {
            dataflow_id,
            node_id,
            key,
            value: value.as_ref(),
        }),
        timestamp,
    })
    .map_err(Into::into)
}

pub(crate) fn ensure_set_param_forward_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::SetParamResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::SetParamResult(Err(err)) => Err(eyre!(
            "daemon failed to apply SetParam for node `{node_id}`: {err}"
        )),
        other => Err(eyre!(
            "unexpected daemon reply for SetParam on node `{node_id}`: {other:?}"
        )),
    }
}

pub(crate) fn ensure_delete_param_forward_applied(
    reply_raw: &[u8],
    node_id: &dora_core::config::NodeId,
) -> eyre::Result<()> {
    match serde_json::from_slice(reply_raw)? {
        DaemonCoordinatorReply::DeleteParamResult(Ok(())) => Ok(()),
        DaemonCoordinatorReply::DeleteParamResult(Err(err)) => Err(eyre!(
            "daemon failed to apply DeleteParam for node `{node_id}`: {err}"
        )),
        other => Err(eyre!(
            "unexpected daemon reply for DeleteParam on node `{node_id}`: {other:?}"
        )),
    }
}

pub(crate) fn schedule_param_replay_for_ready_dataflow(
    dataflow_id: DataflowId,
    dataflow: &RunningDataflow,
    daemon_connections: &DaemonConnections,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: Arc<HLC>,
) {
    // Replay persisted runtime parameters once nodes are ready.
    // This restores desired node state after restart/recovery.
    for daemon_id in &dataflow.daemons {
        if !spawn_param_replay_to_daemon(
            dataflow_id,
            dataflow,
            daemon_id,
            daemon_connections,
            &store,
            &clock,
        ) {
            tracing::warn!(
                "cannot replay params for dataflow {dataflow_id}: no connection for daemon {daemon_id}"
            );
        }
    }
}

/// Start replaying every persisted param of `daemon_id`'s nodes to it, in a
/// spawned task. Returns `false`, without starting one, when the daemon is
/// not connected.
pub(crate) fn spawn_param_replay_to_daemon(
    dataflow_id: DataflowId,
    dataflow: &RunningDataflow,
    daemon_id: &DaemonId,
    daemon_connections: &DaemonConnections,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: &Arc<HLC>,
) -> bool {
    let Some(connection) = daemon_connections.get(daemon_id).cloned() else {
        return false;
    };
    tokio::spawn(replay_persisted_params_for_daemon(
        dataflow_id,
        daemon_id.clone(),
        nodes_on_daemon(dataflow, daemon_id),
        store.clone(),
        connection,
        clock.clone(),
    ));
    true
}

/// Send every persisted param of the daemon's nodes to it, one `SetParam` at
/// a time.
///
/// This runs in a spawned task, concurrently with `dora param set`/`delete`
/// on the event loop. Each item is therefore read from the store and queued
/// to the daemon while holding the connection's `param_write_lock`, which
/// those handlers hold from persisting a change to queuing its forward. The
/// daemon applies commands in queue order, so it ends up with the newest
/// value either way: the replay reads a change that was persisted before it,
/// and a change persisted after it is forwarded after it. A param deleted
/// since the keys were listed is skipped (#3683).
///
/// The lock is released before waiting for the daemon's reply, so a slow
/// daemon delays neither the event loop nor writes to other daemons.
pub(crate) async fn replay_persisted_params_for_daemon(
    dataflow_id: DataflowId,
    daemon_id: DaemonId,
    node_ids_on_daemon: Vec<dora_core::config::NodeId>,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    connection: crate::state::DaemonConnection,
    clock: Arc<HLC>,
) -> ParamReplaySummary {
    let (replay_items, load_failures) =
        collect_param_replay_items(dataflow_id, &node_ids_on_daemon, store.as_ref());
    let mut summary = ParamReplaySummary {
        attempted: load_failures,
        failed: load_failures,
    };
    if replay_items.is_empty() {
        return summary;
    }

    tracing::debug!(
        "replaying {} persisted params for dataflow {} on daemon {}",
        replay_items.len(),
        dataflow_id,
        daemon_id
    );

    for item in replay_items {
        let write_guard = connection.param_write_lock.lock().await;
        let value_json = match store.get_node_param(&dataflow_id, &item.node_id, &item.key) {
            Ok(Some(value_json)) => value_json,
            Ok(None) => {
                tracing::debug!(
                    "not replaying param {dataflow_id}/{}/{}: deleted since replay started",
                    item.node_id,
                    item.key
                );
                continue;
            }
            Err(err) => {
                tracing::warn!(
                    "failed to load persisted param {dataflow_id}/{}/{}: {err}",
                    item.node_id,
                    item.key
                );
                summary.attempted += 1;
                summary.failed += 1;
                continue;
            }
        };
        summary.attempted += 1;
        let message = match build_set_param_message_from_raw_json(
            dataflow_id,
            &item.node_id,
            &item.key,
            &value_json,
            clock.new_timestamp(),
        ) {
            Ok(msg) => msg,
            Err(err) => {
                tracing::warn!(
                    "skipping corrupt persisted param {dataflow_id}/{}/{}: {err}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
                continue;
            }
        };

        let sent = connection.send_request(&message).await;
        drop(write_guard);
        let reply = match sent {
            Ok(pending) => pending.wait().await,
            Err(err) => Err(err),
        };
        let reply_raw = match reply {
            Ok(reply) => reply,
            Err(err) => {
                tracing::warn!(
                    "failed to replay param {dataflow_id}/{}/{} to daemon {daemon_id}: {err}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
                continue;
            }
        };

        match serde_json::from_slice(&reply_raw) {
            Ok(DaemonCoordinatorReply::SetParamResult(Ok(()))) => {}
            Ok(DaemonCoordinatorReply::SetParamResult(Err(err))) => {
                tracing::warn!(
                    "daemon rejected replayed param {dataflow_id}/{}/{}: {err}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
            }
            Ok(other) => {
                tracing::warn!(
                    "unexpected daemon reply while replaying param {dataflow_id}/{}/{}: {other:?}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
            }
            Err(err) => {
                tracing::warn!(
                    "failed to deserialize daemon reply while replaying param {dataflow_id}/{}/{}: {err}",
                    item.node_id,
                    item.key
                );
                summary.failed += 1;
            }
        }
    }

    summary
}
