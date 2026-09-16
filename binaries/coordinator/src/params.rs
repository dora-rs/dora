//! Dataflow parameters: forwarding `SetParam`/`DeleteParam` to daemons and
//! replaying persisted parameters to a daemon that (re)joins a dataflow.

use crate::state::{DaemonConnections, RunningDataflow};
use crate::{FALLBACK_REPLAY_BACKOFF, nodes_on_daemon};
use dora_core::uhlc::HLC;
use dora_message::{DataflowId, common::DaemonId, daemon_to_coordinator::DaemonCoordinatorReply};
use eyre::eyre;
use serde::Serialize;
use serde_json::value::RawValue;
use std::{sync::Arc, time::Instant};

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

pub(crate) async fn handle_pruned_state_catchup_fallback(
    dataflow_id: DataflowId,
    dataflow: &mut RunningDataflow,
    daemon_id: &DaemonId,
    store: Arc<dyn dora_coordinator_store::CoordinatorStore>,
    daemon_connections: &mut DaemonConnections,
    clock: Arc<HLC>,
    now: Instant,
) {
    if let Some(last_replay_attempt) = dataflow.last_replay_attempt.get(daemon_id)
        && now.duration_since(*last_replay_attempt) < FALLBACK_REPLAY_BACKOFF
    {
        tracing::debug!(
            "skipping fallback replay for dataflow {dataflow_id} on daemon {daemon_id}: \
                 backoff active"
        );
        return;
    }

    let Some(connection) = daemon_connections.get_mut(daemon_id).cloned() else {
        tracing::warn!(
            "failed to run fallback replay for dataflow {dataflow_id}: \
             daemon {daemon_id} is not connected"
        );
        return;
    };

    let node_ids_on_daemon: Vec<_> = dataflow
        .node_to_daemon
        .iter()
        .filter(|(_, did)| *did == daemon_id)
        .map(|(node_id, _)| node_id.clone())
        .collect();
    dataflow.last_replay_attempt.insert(daemon_id.clone(), now);

    let replay_summary = replay_persisted_params_for_daemon(
        dataflow_id,
        daemon_id.clone(),
        node_ids_on_daemon,
        store,
        connection,
        clock,
    )
    .await;

    let last_ack = dataflow
        .daemon_ack_sequence
        .get(daemon_id)
        .copied()
        .unwrap_or(0);
    if replay_summary.failed == 0 {
        // Mark daemon as caught up only when full replay succeeds:
        // replay is authoritative for pruned history, and advancing
        // ack on partial failure can silently diverge runtime/store state.
        // Individual SetParam events don't trigger StateCatchUpAck, so
        // we set ack here for successful full replay to avoid repeated
        // fallback replays on every status-report cycle.
        dataflow
            .daemon_ack_sequence
            .insert(daemon_id.clone(), dataflow.state_log_sequence);
    } else {
        tracing::warn!(
            "fallback replay incomplete for dataflow {dataflow_id} on daemon \
             {daemon_id}: attempted={}, failed={}; leaving ack at {}",
            replay_summary.attempted,
            replay_summary.failed,
            last_ack
        );
    }
}

pub(crate) fn collect_param_replay_items(
    dataflow_id: DataflowId,
    node_ids_on_daemon: &[dora_core::config::NodeId],
    store: &dyn dora_coordinator_store::CoordinatorStore,
) -> Vec<ParamReplayItem> {
    let mut items = Vec::new();
    for node_id in node_ids_on_daemon {
        let params = match store.list_node_params(&dataflow_id, node_id) {
            Ok(params) => params,
            Err(err) => {
                tracing::warn!(
                    "failed to load persisted params for {dataflow_id}/{node_id}: {err}"
                );
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
    items
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
    let replay_items = collect_param_replay_items(dataflow_id, &node_ids_on_daemon, store.as_ref());
    let mut summary = ParamReplaySummary::default();
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
