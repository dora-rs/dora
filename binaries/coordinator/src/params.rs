//! Dataflow parameters: forwarding `SetParam`/`DeleteParam` to daemons and
//! replaying persisted parameters to a daemon that (re)joins a dataflow.

use crate::state::{DaemonConnections, RunningDataflow};
use crate::{Event, FALLBACK_REPLAY_BACKOFF, nodes_on_daemon};
use dora_core::uhlc::HLC;
use dora_message::{DataflowId, common::DaemonId, daemon_to_coordinator::DaemonCoordinatorReply};
use eyre::eyre;
use serde::Serialize;
use serde_json::value::RawValue;
use std::{sync::Arc, time::Instant};
use tokio::sync::mpsc;
use uuid::Uuid;

pub(crate) struct ParamReplayItem {
    pub(crate) node_id: dora_core::config::NodeId,
    pub(crate) key: String,
    pub(crate) value_json: Vec<u8>,
}

#[derive(Debug, Default)]
pub(crate) struct ParamReplaySummary {
    pub(crate) attempted: usize,
    pub(crate) failed: usize,
}

/// Start a full param replay for a daemon whose state-log ack was pruned.
///
/// The replay sends every persisted param one round-trip at a time, each
/// bounded by `TCP_READ_TIMEOUT`, so it runs in a spawned task rather than on
/// the event loop (#3684). It reports back through `events` with
/// [`Event::ParamFallbackReplayFinished`], handled by
/// [`finish_pruned_state_catchup_fallback`].
#[allow(clippy::too_many_arguments)]
pub(crate) fn start_pruned_state_catchup_fallback(
    dataflow_id: DataflowId,
    dataflow: &mut RunningDataflow,
    daemon_id: &DaemonId,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    daemon_connections: &mut DaemonConnections,
    clock: Arc<HLC>,
    events: mpsc::Sender<Event>,
    now: Instant,
) {
    let Some(connection) = daemon_connections.get_mut(daemon_id).cloned() else {
        tracing::warn!(
            "failed to run fallback replay for dataflow {dataflow_id}: \
             daemon {daemon_id} is not connected"
        );
        return;
    };
    let connection_id = connection.connection_id;

    if dataflow.fallback_replay_in_flight.get(daemon_id) == Some(&connection_id) {
        tracing::debug!(
            "skipping fallback replay for dataflow {dataflow_id} on daemon {daemon_id}: \
             a replay is already in flight"
        );
        return;
    }
    if let Some(last_replay_attempt) = dataflow.last_replay_attempt.get(daemon_id)
        && now.duration_since(*last_replay_attempt) < FALLBACK_REPLAY_BACKOFF
    {
        tracing::debug!(
            "skipping fallback replay for dataflow {dataflow_id} on daemon {daemon_id}: \
                 backoff active"
        );
        return;
    }

    let node_ids_on_daemon = nodes_on_daemon(dataflow, daemon_id);
    dataflow.last_replay_attempt.insert(daemon_id.clone(), now);
    dataflow
        .fallback_replay_in_flight
        .insert(daemon_id.clone(), connection_id);
    // Captured before the replay reads the store: everything up to here is
    // in the store, so the replay covers it. Entries appended while it runs
    // were forwarded to the daemon by their own handlers.
    let ack_sequence = dataflow.state_log_sequence;
    let daemon_id = daemon_id.clone();

    tokio::spawn(async move {
        let replay_summary = replay_persisted_params_for_daemon(
            dataflow_id,
            daemon_id.clone(),
            node_ids_on_daemon,
            store,
            connection,
            clock,
        )
        .await;
        if replay_summary.failed != 0 {
            tracing::warn!(
                "fallback replay incomplete for dataflow {dataflow_id} on daemon \
                 {daemon_id}: attempted={}, failed={}",
                replay_summary.attempted,
                replay_summary.failed,
            );
        }
        let finished = Event::ParamFallbackReplayFinished {
            dataflow_id,
            daemon_id,
            connection_id,
            ack_sequence,
            succeeded: replay_summary.failed == 0,
        };
        // Fails only when the coordinator is shutting down.
        let _ = events.send(finished).await;
    });
}

/// Record the outcome of a replay started by
/// [`start_pruned_state_catchup_fallback`].
///
/// `current_connection_id` is the daemon's connection now. A replay sent on
/// an older connection says nothing about the daemon behind the new one, so
/// it neither advances the ack nor clears that connection's in-flight mark.
pub(crate) fn finish_pruned_state_catchup_fallback(
    dataflow: &mut RunningDataflow,
    daemon_id: &DaemonId,
    connection_id: Uuid,
    current_connection_id: Option<Uuid>,
    ack_sequence: u64,
    succeeded: bool,
) {
    if dataflow.fallback_replay_in_flight.get(daemon_id) == Some(&connection_id) {
        dataflow.fallback_replay_in_flight.remove(daemon_id);
    }
    if current_connection_id != Some(connection_id) {
        tracing::debug!(
            "ignoring fallback replay result for daemon {daemon_id}: \
             it was sent on a connection that has since been replaced"
        );
        return;
    }
    if !succeeded {
        // Advancing the ack on partial failure could silently diverge
        // runtime and store state; the next status report retries.
        return;
    }
    // Replay is authoritative for pruned history. Individual SetParam
    // events don't trigger StateCatchUpAck, so set the ack here to avoid
    // repeated fallback replays on every status-report cycle. Never move it
    // backwards past an ack that arrived meanwhile.
    let current = dataflow
        .daemon_ack_sequence
        .get(daemon_id)
        .copied()
        .unwrap_or(0);
    dataflow
        .daemon_ack_sequence
        .insert(daemon_id.clone(), current.max(ack_sequence));
}

/// Load the persisted params of every node on a daemon, flattened into
/// replay items. Also returns how many nodes' params could not be loaded:
/// the caller must count those as failed, or a store read error would look
/// like "nothing to replay" and let the daemon be marked caught up.
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
        for (key, bytes) in params {
            items.push(ParamReplayItem {
                node_id: node_id.clone(),
                key,
                value_json: bytes,
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
    daemon_connections: &mut DaemonConnections,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    clock: Arc<HLC>,
) {
    // Replay persisted runtime parameters once nodes are ready.
    // This restores desired node state after restart/recovery.
    let daemon_ids: Vec<_> = dataflow.daemons.iter().cloned().collect();
    for daemon_id in daemon_ids {
        let Some(connection) = daemon_connections.get_mut(&daemon_id).cloned() else {
            tracing::warn!(
                "cannot replay params for dataflow {dataflow_id}: no connection for daemon {daemon_id}"
            );
            continue;
        };
        let node_ids_on_daemon = nodes_on_daemon(dataflow, &daemon_id);
        let store = store.clone();
        let clock = clock.clone();
        tokio::spawn(async move {
            replay_persisted_params_for_daemon(
                dataflow_id,
                daemon_id,
                node_ids_on_daemon,
                store,
                connection,
                clock,
            )
            .await;
        });
    }
}

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
        summary.attempted += 1;
        let message = match build_set_param_message_from_raw_json(
            dataflow_id,
            &item.node_id,
            &item.key,
            &item.value_json,
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

        let reply_raw = match connection.send_and_receive(&message).await {
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
