//! Topic debug streams for `dora topic echo`/`hz`: resolving which daemon owns
//! a topic and starting, stopping, rolling back and restoring the daemon-side
//! streams behind a CLI subscription.

use crate::state::{DaemonConnections, RunningDataflow};
use crate::topic_subscriber;
use dora_core::uhlc::HLC;
use dora_message::{
    DataflowId,
    common::DaemonId,
    coordinator_to_daemon::{DaemonCoordinatorEvent, Timestamped},
    daemon_to_coordinator::DaemonCoordinatorReply,
    descriptor::{CoreNodeKind, RuntimeNode},
    id::DataId,
};
use eyre::{ContextCompat, WrapErr, eyre};
use futures::future::join_all;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use uuid::Uuid;

pub(crate) fn topic_outputs_by_daemon(
    running_dataflows: &HashMap<DataflowId, RunningDataflow>,
    dataflow_id: DataflowId,
    topics: &[(dora_message::id::NodeId, dora_message::id::DataId)],
) -> eyre::Result<BTreeMap<DaemonId, Vec<(dora_message::id::NodeId, dora_message::id::DataId)>>> {
    let dataflow = running_dataflows
        .get(&dataflow_id)
        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;

    // `RunningDataflow.nodes` is the result of `resolve_aliases_and_set_defaults`
    // on this exact descriptor (`spawn_dataflow`), kept in sync by `AddNode` and
    // `ReplaceNode`. Re-resolving would clone and re-walk every node on every
    // subscribe, and add a failure mode this path otherwise doesn't have.
    let resolved_nodes = &dataflow.nodes;

    let mut outputs_by_daemon: BTreeMap<
        DaemonId,
        Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    > = BTreeMap::new();
    for (node_id, data_id) in topics {
        let Some(node) = resolved_nodes.get(node_id) else {
            eyre::bail!(
                "no output `{node_id}/{data_id}` in dataflow `{dataflow_id}`\n\n  \
                 hint: available nodes: {}",
                resolved_nodes
                    .keys()
                    .map(|n| n.to_string())
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        };
        let resolved_data_id = resolved_topic_output_id(node_id, dataflow_id, &node.kind, data_id)?;
        let daemon_id = dataflow
            .node_to_daemon
            .get(node_id)
            .wrap_err_with(|| format!("no daemon mapping found for node `{node_id}`"))?;
        outputs_by_daemon
            .entry(daemon_id.clone())
            .or_default()
            .push((node_id.clone(), resolved_data_id));
    }

    Ok(outputs_by_daemon)
}

/// Translate a subscription request's public output id into the id the daemon
/// keys its debug watchers by.
///
/// A single-operator runtime node's `image` is `op/image` after resolution, and
/// the descriptor's own `inputs:` mappings use the bare form -- so the bare form
/// is the only one a user can have seen. A node with two or more operators has
/// no bare form at all (nothing in a descriptor can reference one), so an
/// unqualified id there is rejected rather than guessed at; the error lists the
/// qualified names so the fix is visible.
pub(crate) fn resolved_topic_output_id(
    node_id: &dora_message::id::NodeId,
    dataflow_id: DataflowId,
    kind: &CoreNodeKind,
    data_id: &DataId,
) -> eyre::Result<DataId> {
    let outputs = resolved_node_outputs(kind);
    if outputs.contains(data_id) {
        return Ok(data_id.clone());
    }

    if let CoreNodeKind::Runtime(node) = kind
        && let [operator] = node.operators.as_slice()
        && operator.config.outputs.contains(data_id)
    {
        return format!("{}/{}", operator.id, data_id)
            .parse::<DataId>()
            .map_err(|e| eyre::eyre!("failed to resolve topic output id: {e}"));
    }

    let available = if outputs.is_empty() {
        "none".to_string()
    } else {
        outputs
            .iter()
            .map(|o| o.to_string())
            .collect::<Vec<_>>()
            .join(", ")
    };
    eyre::bail!(
        "no output `{node_id}/{data_id}` in dataflow `{dataflow_id}`\n\n  \
         hint: outputs of `{node_id}`: {available}"
    )
}

pub(crate) fn resolved_node_outputs(kind: &CoreNodeKind) -> BTreeSet<DataId> {
    match kind {
        CoreNodeKind::Custom(node) => node.run_config.outputs.clone(),
        CoreNodeKind::Runtime(node) => runtime_node_outputs(node),
    }
}

pub(crate) fn runtime_node_outputs(node: &RuntimeNode) -> BTreeSet<DataId> {
    node.operators
        .iter()
        .flat_map(|operator| {
            operator
                .config
                .outputs
                .iter()
                .map(|output_id| DataId::from(format!("{}/{output_id}", operator.id)))
        })
        .collect()
}

pub(crate) fn topic_debug_enabled(
    running_dataflows: &HashMap<DataflowId, RunningDataflow>,
    dataflow_id: DataflowId,
) -> eyre::Result<bool> {
    let dataflow = running_dataflows
        .get(&dataflow_id)
        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?;
    Ok(dataflow.descriptor.debug.enable_debug_inspection)
}

pub(crate) async fn start_topic_debug_stream(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    dataflow_id: DataflowId,
    topics: Vec<(dora_message::id::NodeId, dora_message::id::DataId)>,
    sender: tokio::sync::mpsc::Sender<crate::topic_subscriber::TopicFrame>,
    clock: &HLC,
) -> eyre::Result<Uuid> {
    let outputs_by_daemon = topic_outputs_by_daemon(running_dataflows, dataflow_id, &topics)?;
    if !topic_debug_enabled(running_dataflows, dataflow_id)? {
        eyre::bail!("topic inspection requires `debug.enable_debug_inspection: true`");
    }
    let subscription_id = Uuid::new_v4();
    // Build the subscriber and every per-daemon start request *before*
    // registering the subscriber in `topic_subscribers`. Both the connection
    // lookup and the message serialization below can bail with `?`; if they did
    // so after the insert, the just-registered `subscription_id` would be
    // orphaned — the CLI only sees the returned error and never learns the id,
    // so it can never `TopicUnsubscribe` it, and the entry would linger for the
    // life of the dataflow (only opportunistically reaped by `send_topic_frames`
    // if a frame ever happens to route to it). Registering only once everything
    // fallible has succeeded keeps the pre-dispatch failure path leak-free; the
    // post-dispatch path already rolls back via `rollback_topic_debug_stream`.
    // No frame can reach the subscriber before it is inserted, because the
    // requests are merely built here and not dispatched until `join_all` below.
    let subscriber = topic_subscriber::TopicSubscriber::new(outputs_by_daemon.clone(), sender);

    let mut start_requests = Vec::new();
    for (daemon_id, outputs) in outputs_by_daemon {
        let connection = daemon_connections
            .get_mut(&daemon_id)
            .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?
            .clone();
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::start_topic_debug_stream(
                dataflow_id,
                outputs,
                subscription_id,
            ),
            timestamp: clock.new_timestamp(),
        })?;
        start_requests.push(async move {
            let result = async {
                let reply_raw = connection
                    .send_and_receive(&message)
                    .await
                    .wrap_err("failed to send start-topic-debug-stream message")?;
                let reply: DaemonCoordinatorReply = serde_json::from_slice(&reply_raw)
                    .wrap_err("failed to deserialize start-topic-debug-stream reply")?;
                match reply {
                    DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(())) => Ok(()),
                    DaemonCoordinatorReply::StartTopicDebugStreamResult(Err(err)) => {
                        Err(eyre!(err))
                    }
                    other => Err(eyre!(
                        "unexpected start-topic-debug-stream reply: {other:?}"
                    )),
                }
            }
            .await;
            (daemon_id, result)
        });
    }

    // Everything fallible above has succeeded — register the subscriber now, so
    // the post-dispatch failure path (rolled back below) is the only one that
    // has to clean it up.
    running_dataflows
        .get_mut(&dataflow_id)
        .wrap_err_with(|| format!("no running dataflow with ID `{dataflow_id}`"))?
        .topic_subscribers
        .insert(subscription_id, subscriber);

    let mut started_daemons = Vec::new();
    let mut first_error = None;
    for (daemon_id, result) in join_all(start_requests).await {
        match result {
            Ok(()) => started_daemons.push(daemon_id),
            Err(err) => {
                if first_error.is_none() {
                    first_error = Some(err);
                }
            }
        }
    }

    if let Some(err) = first_error {
        rollback_topic_debug_stream(
            running_dataflows,
            daemon_connections,
            dataflow_id,
            subscription_id,
            &started_daemons,
            clock,
        )
        .await?;
        return Err(err);
    }

    Ok(subscription_id)
}

pub(crate) async fn rollback_topic_debug_stream(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    dataflow_id: DataflowId,
    subscription_id: Uuid,
    started_daemons: &[DaemonId],
    clock: &HLC,
) -> eyre::Result<()> {
    let Some(_) = running_dataflows
        .get_mut(&dataflow_id)
        .and_then(|dataflow| dataflow.topic_subscribers.remove(&subscription_id))
    else {
        return Ok(());
    };

    for daemon_id in started_daemons {
        let Some(connection) = daemon_connections.get_mut(daemon_id).cloned() else {
            continue;
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::StopTopicDebugStream {
                dataflow_id,
                subscription_id,
            },
            timestamp: clock.new_timestamp(),
        })?;
        let reply_raw = connection
            .send_and_receive(&message)
            .await
            .wrap_err("failed to roll back start-topic-debug-stream message")?;
        let reply: DaemonCoordinatorReply = serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize rollback stop-topic-debug-stream reply")?;
        match reply {
            DaemonCoordinatorReply::StopTopicDebugStreamResult(Ok(())) => {}
            DaemonCoordinatorReply::StopTopicDebugStreamResult(Err(err)) => {
                tracing::warn!(%daemon_id, %subscription_id, "failed to roll back topic debug stream: {err}");
            }
            other => {
                tracing::warn!(%daemon_id, %subscription_id, "unexpected rollback reply: {other:?}");
            }
        }
    }
    Ok(())
}

pub(crate) async fn stop_topic_debug_stream(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    subscription_id: Uuid,
    clock: &HLC,
) -> eyre::Result<()> {
    let Some((dataflow_id, subscriber)) = running_dataflows.iter_mut().find_map(|(id, df)| {
        df.topic_subscribers
            .remove(&subscription_id)
            .map(|subscriber| (*id, subscriber))
    }) else {
        return Ok(());
    };
    teardown_topic_debug_stream(
        daemon_connections,
        dataflow_id,
        subscription_id,
        subscriber.outputs_by_daemon().keys().cloned(),
        clock,
    )
    .await
}

/// Forward a daemon's topic debug frame to its CLI subscribers.
///
/// A subscriber that got closed on the way (its CLI went away, or it stayed
/// too slow for too long) is removed from the dataflow by
/// [`send_topic_frames`](crate::handlers::send_topic_frames); stop its daemon
/// streams here too. Otherwise the daemons keep serializing and shipping every
/// matching output to the coordinator until the dataflow ends, and a later
/// `TopicUnsubscribe` for the id no longer finds anything to tear down.
pub(crate) async fn forward_topic_frames(
    running_dataflows: &mut HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    dataflow_id: DataflowId,
    subscription_ids: Vec<Uuid>,
    payload: Vec<u8>,
    clock: &HLC,
) {
    let Some(dataflow) = running_dataflows.get_mut(&dataflow_id) else {
        return;
    };
    let evicted = crate::handlers::send_topic_frames(
        &mut dataflow.topic_subscribers,
        subscription_ids,
        payload,
    )
    .await;
    for (subscription_id, subscriber) in evicted {
        let stop_requests = match topic_stop_requests(
            daemon_connections,
            dataflow_id,
            subscription_id,
            subscriber.outputs_by_daemon().keys().cloned(),
            clock,
        ) {
            Ok(requests) => requests,
            Err(err) => {
                tracing::warn!(
                    %subscription_id,
                    "failed to stop topic debug stream of a closed subscriber: {err:?}"
                );
                continue;
            }
        };
        // Don't wait for the replies here: this runs inside the serial
        // coordinator event loop, and the daemon streaming these frames may
        // have its WS task blocked on a full event channel. That task can
        // then neither forward the stop nor read the reply, so awaiting it
        // would stall the whole coordinator until `TCP_READ_TIMEOUT`.
        tokio::spawn(async move {
            for (daemon_id, result) in join_all(stop_requests).await {
                if let Err(err) = result {
                    tracing::warn!(
                        %daemon_id,
                        %subscription_id,
                        "failed to stop topic debug stream of a closed subscriber: {err}"
                    );
                }
            }
        });
    }
}

/// Send `StopTopicDebugStream` for `subscription_id` to every daemon in
/// `daemon_ids` and wait for their replies. The subscriber must already be
/// removed from the dataflow's `topic_subscribers`.
async fn teardown_topic_debug_stream(
    daemon_connections: &mut DaemonConnections,
    dataflow_id: DataflowId,
    subscription_id: Uuid,
    daemon_ids: impl IntoIterator<Item = DaemonId>,
    clock: &HLC,
) -> eyre::Result<()> {
    let stop_requests = topic_stop_requests(
        daemon_connections,
        dataflow_id,
        subscription_id,
        daemon_ids,
        clock,
    )?;

    let mut first_error = None;
    for (daemon_id, result) in join_all(stop_requests).await {
        if let Err(err) = result {
            tracing::warn!(
                %daemon_id,
                %subscription_id,
                "failed to stop topic debug stream on daemon: {err}"
            );
            if first_error.is_none() {
                first_error = Some(err);
            }
        }
    }

    if let Some(err) = first_error {
        return Err(err);
    }

    Ok(())
}

/// Build one `StopTopicDebugStream` request per daemon in `daemon_ids`. Each
/// future owns its connection handle, so it can be awaited in place or
/// spawned.
fn topic_stop_requests(
    daemon_connections: &mut DaemonConnections,
    dataflow_id: DataflowId,
    subscription_id: Uuid,
    daemon_ids: impl IntoIterator<Item = DaemonId>,
    clock: &HLC,
) -> eyre::Result<Vec<impl Future<Output = (DaemonId, eyre::Result<()>)> + Send + 'static>> {
    let mut stop_requests = Vec::new();
    for daemon_id in daemon_ids {
        let Some(connection) = daemon_connections.get_mut(&daemon_id).cloned() else {
            tracing::warn!(
                %daemon_id,
                %subscription_id,
                "skipping topic debug stream teardown for missing daemon connection"
            );
            continue;
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::StopTopicDebugStream {
                dataflow_id,
                subscription_id,
            },
            timestamp: clock.new_timestamp(),
        })?;
        stop_requests.push(async move {
            let result = async {
                let reply_raw = connection
                    .send_and_receive(&message)
                    .await
                    .wrap_err("failed to send stop-topic-debug-stream message")?;
                let reply: DaemonCoordinatorReply = serde_json::from_slice(&reply_raw)
                    .wrap_err("failed to deserialize stop-topic-debug-stream reply")?;
                match reply {
                    DaemonCoordinatorReply::StopTopicDebugStreamResult(Ok(())) => Ok(()),
                    DaemonCoordinatorReply::StopTopicDebugStreamResult(Err(err)) => Err(eyre!(err)),
                    other => Err(eyre!("unexpected stop-topic-debug-stream reply: {other:?}")),
                }
            }
            .await;
            (daemon_id, result)
        });
    }
    Ok(stop_requests)
}

/// Close every CLI topic-subscriber channel on a finished dataflow so the
/// corresponding `dora topic echo/hz/info` client sees EOF on its receiver
/// instead of hanging forever.
///
/// Called from the `DataflowFinishedOnDaemon` arm of the event loop when the
/// last daemon has finished. Pulling this out of the inline match arm gives
/// the behavior a named call site: removing it from the event loop shows up
/// in a code review as "no more callers of close_topic_subscribers_on_finish".
pub(crate) fn close_topic_subscribers_on_finish(dataflow: &mut RunningDataflow) {
    for subscriber in dataflow.topic_subscribers.values_mut() {
        subscriber.close();
    }
}

pub(crate) async fn restore_topic_debug_streams_for_daemon(
    running_dataflows: &HashMap<DataflowId, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    daemon_id: &DaemonId,
    reported_dataflows: &BTreeSet<DataflowId>,
    clock: &HLC,
) {
    let Some(connection) = daemon_connections.get_mut(daemon_id).cloned() else {
        return;
    };

    for (dataflow_id, dataflow) in running_dataflows {
        if !reported_dataflows.contains(dataflow_id) {
            continue;
        }
        for (subscription_id, subscriber) in &dataflow.topic_subscribers {
            let Some(outputs) = subscriber.outputs_by_daemon().get(daemon_id).cloned() else {
                continue;
            };
            let message = match serde_json::to_vec(&Timestamped {
                inner: DaemonCoordinatorEvent::start_topic_debug_stream(
                    *dataflow_id,
                    outputs,
                    *subscription_id,
                ),
                timestamp: clock.new_timestamp(),
            }) {
                Ok(message) => message,
                Err(err) => {
                    tracing::warn!(
                        %daemon_id,
                        %dataflow_id,
                        %subscription_id,
                        "failed to serialize topic debug stream restore message: {err}"
                    );
                    continue;
                }
            };

            match connection.send_and_receive(&message).await {
                Ok(reply_raw) => {
                    match serde_json::from_slice::<DaemonCoordinatorReply>(&reply_raw) {
                        Ok(DaemonCoordinatorReply::StartTopicDebugStreamResult(Ok(()))) => {}
                        Ok(DaemonCoordinatorReply::StartTopicDebugStreamResult(Err(err))) => {
                            tracing::warn!(
                                %daemon_id,
                                %dataflow_id,
                                %subscription_id,
                                "daemon rejected restored topic debug stream: {err}"
                            );
                        }
                        Ok(other) => {
                            tracing::warn!(
                                %daemon_id,
                                %dataflow_id,
                                %subscription_id,
                                "unexpected restore-topic-debug-stream reply: {other:?}"
                            );
                        }
                        Err(err) => {
                            tracing::warn!(
                                %daemon_id,
                                %dataflow_id,
                                %subscription_id,
                                "failed to deserialize restore-topic-debug-stream reply: {err}"
                            );
                        }
                    }
                }
                Err(err) => {
                    tracing::warn!(
                        %daemon_id,
                        %dataflow_id,
                        %subscription_id,
                        "failed to restore topic debug stream after daemon reconnect: {err}"
                    );
                }
            }
        }
    }
}
