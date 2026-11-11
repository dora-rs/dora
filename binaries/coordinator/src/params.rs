//! Dataflow parameters: forwarding `SetParam`/`DeleteParam` to daemons,
//! replaying persisted parameters to a daemon that (re)joins a dataflow, and
//! deleting on a reconnected daemon the parameters it missed the delete of.

use crate::nodes_on_daemon;
use crate::state::{DaemonConnections, RunningDataflow};
use dora_core::{config::NodeId, uhlc::HLC};
use dora_message::{
    DataflowId,
    common::DaemonId,
    coordinator_to_daemon::{DaemonCoordinatorEvent, Timestamped},
    daemon_to_coordinator::DaemonCoordinatorReply,
};
use eyre::eyre;
use serde::Serialize;
use serde_json::value::RawValue;
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};

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
///
/// Only the params still in the store are sent. One the daemon's nodes hold
/// but that was deleted while it could not be told is removed by
/// [`delete_unpersisted_params_on_daemon`], from the keys the daemon reports.
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
        let outcome = queue_param_command(
            &connection,
            &item.node_id,
            || match store.get_node_param(&dataflow_id, &item.node_id, &item.key) {
                Ok(Some(value_json)) => build_set_param_message_from_raw_json(
                    dataflow_id,
                    &item.node_id,
                    &item.key,
                    &value_json,
                    clock.new_timestamp(),
                )
                .map(Some)
                .map_err(|err| eyre!("corrupt persisted param: {err}")),
                Ok(None) => {
                    tracing::debug!(
                        "not replaying param {dataflow_id}/{}/{}: deleted since replay started",
                        item.node_id,
                        item.key
                    );
                    Ok(None)
                }
                Err(err) => Err(eyre!("failed to load persisted param: {err}")),
            },
            ensure_set_param_forward_applied,
        )
        .await;
        summary.record(outcome, || {
            format!(
                "failed to replay param {dataflow_id}/{}/{} to daemon {daemon_id}",
                item.node_id, item.key
            )
        });
    }

    summary
}

/// The reported keys of the nodes `daemon_id` runs in `dataflow`, as replay
/// items. A key reported for a node that runs on another daemon, or that the
/// dataflow does not have, is not this daemon's to reconcile and is left out.
pub(crate) fn delivered_param_items_on_daemon(
    dataflow: &RunningDataflow,
    daemon_id: &DaemonId,
    delivered: BTreeMap<NodeId, BTreeSet<String>>,
) -> Vec<ParamReplayItem> {
    let mut items = Vec::new();
    for (node_id, keys) in delivered {
        if dataflow.node_to_daemon.get(&node_id) != Some(daemon_id) {
            tracing::debug!(
                "ignoring reported param keys of node `{node_id}` in dataflow {}: \
                 it does not run on daemon {daemon_id}",
                dataflow.uuid
            );
            continue;
        }
        items.extend(keys.into_iter().map(|key| ParamReplayItem {
            node_id: node_id.clone(),
            key,
        }));
    }
    items
}

/// Delete, on the daemon, every reported param that is not in the store.
///
/// The daemon reports the keys it has delivered to its nodes on every
/// connection (`DaemonEvent::DeliveredParamKeys`). A key among them that the
/// store no longer has was deleted while the daemon could not be told: the
/// forward failed and the state log entry that would have caught it up was
/// pruned, or dropped with the dataflow when every daemon disconnected. The
/// full param replay cannot express that, since it sends what the store has.
///
/// Runs alongside the replay and `dora param set`/`delete`, with the same
/// ordering rule as [`replay_persisted_params_for_daemon`]: each key is
/// re-read from the store and its `DeleteParam` queued while holding the
/// connection's `param_write_lock`. A key set again in the meantime is in the
/// store by then and is left alone; the replay or the set's own forward sends
/// its value.
pub(crate) async fn delete_unpersisted_params_on_daemon(
    dataflow_id: DataflowId,
    daemon_id: DaemonId,
    items: Vec<ParamReplayItem>,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    connection: crate::state::DaemonConnection,
    clock: Arc<HLC>,
) -> ParamReplaySummary {
    let mut summary = ParamReplaySummary::default();
    for item in items {
        let outcome = queue_param_command(
            &connection,
            &item.node_id,
            || match store.get_node_param(&dataflow_id, &item.node_id, &item.key) {
                // Still set: the node keeps it.
                Ok(Some(_)) => Ok(None),
                Ok(None) => serde_json::to_vec(&Timestamped {
                    inner: DaemonCoordinatorEvent::DeleteParam {
                        dataflow_id,
                        node_id: item.node_id.clone(),
                        key: item.key.clone(),
                    },
                    timestamp: clock.new_timestamp(),
                })
                .map(Some)
                .map_err(|err| eyre!("failed to serialize DeleteParam: {err}")),
                Err(err) => Err(eyre!("failed to load persisted param: {err}")),
            },
            ensure_delete_param_forward_applied,
        )
        .await;
        if matches!(outcome, ParamCommandOutcome::Applied) {
            tracing::info!(
                "deleted param {dataflow_id}/{}/{} on daemon {daemon_id}: \
                 it was deleted while the daemon was disconnected",
                item.node_id,
                item.key
            );
        }
        summary.record(outcome, || {
            format!(
                "failed to delete param {dataflow_id}/{}/{} on daemon {daemon_id}",
                item.node_id, item.key
            )
        });
    }
    summary
}

/// Start [`delete_unpersisted_params_on_daemon`] for the keys `daemon_id`
/// reports for its nodes in `dataflow`, in a spawned task. Returns `false`,
/// without starting one, when the daemon is not connected.
pub(crate) fn spawn_delete_of_unpersisted_params(
    dataflow_id: DataflowId,
    dataflow: &RunningDataflow,
    daemon_id: &DaemonId,
    delivered: BTreeMap<NodeId, BTreeSet<String>>,
    daemon_connections: &DaemonConnections,
    store: &Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: &Arc<HLC>,
) -> bool {
    let Some(connection) = daemon_connections.get(daemon_id).cloned() else {
        return false;
    };
    let items = delivered_param_items_on_daemon(dataflow, daemon_id, delivered);
    if items.is_empty() {
        return true;
    }
    let daemon_id = daemon_id.clone();
    let store = store.clone();
    let clock = clock.clone();
    tokio::spawn(async move {
        let reported = items.len();
        let summary = delete_unpersisted_params_on_daemon(
            dataflow_id,
            daemon_id.clone(),
            items,
            store,
            connection,
            clock,
        )
        .await;
        if summary.failed > 0 {
            tracing::warn!(
                "{} of {reported} reported param key(s) of dataflow {dataflow_id} could not be \
                 reconciled on daemon {daemon_id}; its nodes may keep deleted params until it \
                 reconnects",
                summary.failed
            );
        }
    });
    true
}

/// What happened to one param command of a replay or reconcile.
enum ParamCommandOutcome {
    /// Nothing to send for it once the store was re-read.
    Skipped,
    /// Queued, and the daemon applied it.
    Applied,
    /// Not sent, not delivered, or not applied.
    Failed(eyre::Report),
}

impl ParamReplaySummary {
    fn record(&mut self, outcome: ParamCommandOutcome, context: impl FnOnce() -> String) {
        match outcome {
            ParamCommandOutcome::Skipped => {}
            ParamCommandOutcome::Applied => self.attempted += 1,
            ParamCommandOutcome::Failed(err) => {
                tracing::warn!("{}: {err:#}", context());
                self.attempted += 1;
                self.failed += 1;
            }
        }
    }
}

/// Queue one param command to the daemon in `param_write_lock` order, then
/// wait for its reply with the lock released.
///
/// `prepare` runs under the lock, so the store contents it reads cannot
/// change before the command it builds from them is queued. It returns
/// `None` when there is nothing to send. `confirm` checks the daemon's reply.
async fn queue_param_command(
    connection: &crate::state::DaemonConnection,
    node_id: &NodeId,
    prepare: impl FnOnce() -> eyre::Result<Option<Vec<u8>>>,
    confirm: fn(&[u8], &NodeId) -> eyre::Result<()>,
) -> ParamCommandOutcome {
    let write_guard = connection.param_write_lock.lock().await;
    let message = match prepare() {
        Ok(Some(message)) => message,
        Ok(None) => return ParamCommandOutcome::Skipped,
        Err(err) => return ParamCommandOutcome::Failed(err),
    };
    let sent = connection.send_request(&message).await;
    drop(write_guard);
    let reply = match sent {
        Ok(pending) => pending.wait().await,
        Err(err) => Err(err),
    };
    match reply.and_then(|reply_raw| confirm(&reply_raw, node_id)) {
        Ok(()) => ParamCommandOutcome::Applied,
        Err(err) => ParamCommandOutcome::Failed(err),
    }
}
