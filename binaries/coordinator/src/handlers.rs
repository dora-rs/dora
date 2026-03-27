use crate::{
    log_subscriber::LogSubscriber,
    run::{SpawnedDataflow, resolve_daemon, spawn_dataflow},
    state::{
        self, ArchivedDataflow, CachedResult, DaemonConnection, DaemonConnections, RunningBuild,
        RunningDataflow,
    },
};
use adora_core::{
    config::{NodeId, OperatorId},
    descriptor::DescriptorExt,
    uhlc::{self, HLC},
};
use adora_message::{
    BuildId, SessionId,
    common::{DaemonId, GitSource},
    coordinator_to_cli::{DataflowResult, LogMessage},
    coordinator_to_daemon::{BuildDataflowNodes, DaemonCoordinatorEvent, Timestamped},
    daemon_to_coordinator::{DaemonCoordinatorReply, DataflowDaemonResult},
    descriptor::Descriptor,
};
use eyre::{ContextCompat, WrapErr, bail, eyre};
use futures::future::join_all;
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    path::PathBuf,
    time::Duration,
};
use uuid::Uuid;

// Resolve the dataflow name.
pub(crate) fn resolve_name(
    name: String,
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    archived_dataflows: &indexmap::IndexMap<Uuid, ArchivedDataflow>,
) -> eyre::Result<Uuid> {
    let uuids: Vec<_> = running_dataflows
        .iter()
        .filter(|(_, v)| v.name.as_deref() == Some(name.as_str()))
        .map(|(k, _)| k)
        .copied()
        .collect();
    let archived_uuids: Vec<_> = archived_dataflows
        .iter()
        .filter(|(_, v)| v.name.as_deref() == Some(name.as_str()))
        .map(|(k, _)| k)
        .copied()
        .collect();

    if uuids.is_empty() {
        if archived_uuids.is_empty() {
            bail!("no dataflow with name `{name}`");
        } else if let [uuid] = archived_uuids.as_slice() {
            Ok(*uuid)
        } else {
            bail!(
                "multiple archived dataflows found with name `{name}`, Please provide the UUID instead."
            );
        }
    } else if let [uuid] = uuids.as_slice() {
        Ok(*uuid)
    } else {
        bail!("multiple dataflows found with name `{name}`");
    }
}

pub(crate) async fn send_log_message(
    log_subscribers: &mut Vec<LogSubscriber>,
    message: &LogMessage,
) {
    for subscriber in log_subscribers.iter_mut() {
        let send_result =
            tokio::time::timeout(Duration::from_millis(100), subscriber.send_message(message));

        if send_result.await.is_err() {
            subscriber.close();
        }
    }
    log_subscribers.retain(|s| !s.is_closed());
}

pub(crate) fn dataflow_result(
    results: &BTreeMap<DaemonId, DataflowDaemonResult>,
    dataflow_uuid: Uuid,
    clock: &uhlc::HLC,
) -> DataflowResult {
    let mut node_results = BTreeMap::new();
    for result in results.values() {
        node_results.extend(result.node_results.clone());
        if let Err(err) = clock.update_with_timestamp(&result.timestamp) {
            tracing::warn!("failed to update HLC: {err}");
        }
    }

    DataflowResult {
        uuid: dataflow_uuid,
        timestamp: clock.new_timestamp(),
        node_results,
    }
}

pub(crate) async fn handle_destroy(
    running_dataflows: &mut HashMap<Uuid, RunningDataflow>,
    daemon_connections: &mut DaemonConnections,
    abortable_events: &futures::stream::AbortHandle,
    clock: &HLC,
    store: &dyn adora_coordinator_store::CoordinatorStore,
) -> Result<(), eyre::ErrReport> {
    abortable_events.abort();
    // Persist Stopping for all running dataflows before sending stop messages.
    for dataflow in running_dataflows.values_mut() {
        if let Err(e) = dataflow
            .make_record(adora_coordinator_store::DataflowStatus::Stopping)
            .and_then(|r| store.put_dataflow(&r))
        {
            tracing::warn!("failed to persist dataflow stopping on destroy: {e}");
        }
    }
    for dataflow_uuid in running_dataflows.keys().cloned().collect::<Vec<_>>() {
        let _ = stop_dataflow(
            running_dataflows,
            dataflow_uuid,
            daemon_connections,
            clock.new_timestamp(),
            None,
            false,
        )
        .await?;
    }

    destroy_daemons(daemon_connections, clock.new_timestamp()).await
}

pub(crate) async fn send_heartbeat_message(
    connection: &mut DaemonConnection,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Heartbeat,
        timestamp,
    })
    .context("Could not serialize heartbeat message")?;

    connection
        .send(&message)
        .await
        .wrap_err("failed to send heartbeat message to daemon")
}

pub(crate) async fn stop_dataflow<'a>(
    running_dataflows: &'a mut HashMap<Uuid, RunningDataflow>,
    dataflow_uuid: Uuid,
    daemon_connections: &mut DaemonConnections,
    timestamp: uhlc::Timestamp,
    grace_duration: Option<Duration>,
    force: bool,
) -> eyre::Result<&'a mut RunningDataflow> {
    let Some(dataflow) = running_dataflows.get_mut(&dataflow_uuid) else {
        bail!("no known running dataflow found with UUID `{dataflow_uuid}`")
    };

    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::StopDataflow {
            dataflow_id: dataflow_uuid,
            grace_duration,
            force,
        },
        timestamp,
    })?;

    for daemon_id in &dataflow.daemons {
        let daemon_connection = daemon_connections
            .get_mut(daemon_id)
            .wrap_err("no daemon connection")?;

        let reply_raw = daemon_connection
            .send_and_receive(&message)
            .await
            .wrap_err("failed to send/receive stop message")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize stop reply from daemon")?
        {
            DaemonCoordinatorReply::StopResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to stop dataflow")?,
            other => bail!("unexpected reply after sending stop: {other:?}"),
        }
    }

    tracing::info!("successfully send stop dataflow `{dataflow_uuid}` to all daemons");

    Ok(dataflow)
}

pub(crate) async fn reload_dataflow(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    operator_id: Option<OperatorId>,
    daemon_connections: &mut DaemonConnections,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let Some(dataflow) = running_dataflows.get(&dataflow_id) else {
        bail!("No running dataflow found with UUID `{dataflow_id}`")
    };
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::ReloadDataflow {
            dataflow_id,
            node_id,
            operator_id,
        },
        timestamp,
    })?;

    for machine_id in &dataflow.daemons {
        let daemon_connection = daemon_connections
            .get_mut(machine_id)
            .wrap_err("no daemon connection")?;

        let reply_raw = daemon_connection
            .send_and_receive(&message)
            .await
            .wrap_err("failed to send/receive reload message")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize reload reply from daemon")?
        {
            DaemonCoordinatorReply::ReloadResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("failed to reload dataflow")?,
            other => bail!("unexpected reply after sending reload: {other:?}"),
        }
    }
    tracing::info!("successfully reloaded dataflow `{dataflow_id}`");

    Ok(())
}

/// Send a single-node command (restart or stop) to the appropriate daemon.
async fn dispatch_node_command(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    dataflow_id: Uuid,
    node_id: &NodeId,
    event: DaemonCoordinatorEvent,
    daemon_connections: &mut DaemonConnections,
    timestamp: uhlc::Timestamp,
    action: &str,
) -> eyre::Result<DaemonCoordinatorReply> {
    let Some(dataflow) = running_dataflows.get(&dataflow_id) else {
        bail!("No running dataflow found with UUID `{dataflow_id}`")
    };
    let daemon_id = dataflow
        .node_to_daemon
        .get(node_id)
        .with_context(|| format!("node `{node_id}` not found in dataflow `{dataflow_id}`"))?;

    let message = serde_json::to_vec(&Timestamped {
        inner: event,
        timestamp,
    })?;

    let daemon_connection = daemon_connections
        .get_mut(daemon_id)
        .wrap_err("no daemon connection")?;
    let reply_raw = daemon_connection
        .send_and_receive(&message)
        .await
        .wrap_err_with(|| format!("failed to send/receive {action} node message"))?;
    serde_json::from_slice(&reply_raw)
        .wrap_err_with(|| format!("failed to deserialize {action} node reply"))
}

pub(crate) async fn restart_node(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    grace_duration: Option<Duration>,
    daemon_connections: &mut DaemonConnections,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let reply = dispatch_node_command(
        running_dataflows,
        dataflow_id,
        &node_id,
        DaemonCoordinatorEvent::RestartNode {
            dataflow_id,
            node_id: node_id.clone(),
            grace_duration,
        },
        daemon_connections,
        timestamp,
        "restart",
    )
    .await?;
    match reply {
        DaemonCoordinatorReply::RestartNodeResult(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("failed to restart node")?,
        other => bail!("unexpected reply after sending restart node: {other:?}"),
    }
    tracing::info!("successfully restarted node `{node_id}` in dataflow `{dataflow_id}`");
    Ok(())
}

pub(crate) async fn stop_node(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    grace_duration: Option<Duration>,
    daemon_connections: &mut DaemonConnections,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let reply = dispatch_node_command(
        running_dataflows,
        dataflow_id,
        &node_id,
        DaemonCoordinatorEvent::StopNode {
            dataflow_id,
            node_id: node_id.clone(),
            grace_duration,
        },
        daemon_connections,
        timestamp,
        "stop",
    )
    .await?;
    match reply {
        DaemonCoordinatorReply::StopNodeResult(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("failed to stop node")?,
        other => bail!("unexpected reply after sending stop node: {other:?}"),
    }
    tracing::info!("successfully stopped node `{node_id}` in dataflow `{dataflow_id}`");
    Ok(())
}

pub(crate) async fn retrieve_logs(
    running_dataflows: &HashMap<Uuid, RunningDataflow>,
    archived_dataflows: &indexmap::IndexMap<Uuid, ArchivedDataflow>,
    dataflow_id: Uuid,
    node_id: NodeId,
    daemon_connections: &mut DaemonConnections,
    timestamp: uhlc::Timestamp,
    tail: Option<usize>,
) -> eyre::Result<Vec<u8>> {
    let nodes = if let Some(dataflow) = archived_dataflows.get(&dataflow_id) {
        dataflow.nodes.clone()
    } else if let Some(dataflow) = running_dataflows.get(&dataflow_id) {
        dataflow.nodes.clone()
    } else {
        bail!("No dataflow found with UUID `{dataflow_id}`")
    };

    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Logs {
            dataflow_id,
            node_id: node_id.clone(),
            tail,
        },
        timestamp,
    })?;

    let machine_ids: Vec<Option<String>> = nodes
        .values()
        .filter(|node| node.id == node_id)
        .map(|node| node.deploy.as_ref().and_then(|d| d.machine.clone()))
        .collect();

    let machine_id = if let [machine_id] = &machine_ids[..] {
        machine_id
    } else if machine_ids.is_empty() {
        bail!("No machine contains {}/{}", dataflow_id, node_id)
    } else {
        bail!(
            "More than one machine contains {}/{}. However, it should only be present on one.",
            dataflow_id,
            node_id
        )
    };

    let daemon_ids: Vec<_> = match machine_id {
        None => daemon_connections.unnamed().collect(),
        Some(machine_id) => daemon_connections
            .get_matching_daemon_id(machine_id)
            .into_iter()
            .collect(),
    };
    let daemon_id = match &daemon_ids[..] {
        [id] => (*id).clone(),
        [] => eyre::bail!("no matching daemon connections for machine ID `{machine_id:?}`"),
        _ => eyre::bail!("multiple matching daemon connections for machine ID `{machine_id:?}`"),
    };
    let daemon_connection = daemon_connections
        .get_mut(&daemon_id)
        .wrap_err_with(|| format!("no daemon connection to `{daemon_id}`"))?;

    let reply_raw = daemon_connection
        .send_and_receive(&message)
        .await
        .wrap_err("failed to send/receive logs message")?;
    let reply_logs = match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize logs reply from daemon")?
    {
        DaemonCoordinatorReply::Logs(logs) => logs,
        other => bail!("unexpected reply after sending logs: {other:?}"),
    };
    tracing::info!("successfully retrieved logs for `{dataflow_id}/{node_id}`");

    reply_logs.map_err(|err| eyre!(err))
}

#[allow(clippy::too_many_arguments)]
#[tracing::instrument(skip(daemon_connections, clock))]
pub(crate) async fn build_dataflow(
    build_id: BuildId,
    session_id: SessionId,
    dataflow: Descriptor,
    git_sources: BTreeMap<NodeId, GitSource>,
    prev_git_sources: BTreeMap<NodeId, GitSource>,
    local_working_dir: Option<PathBuf>,
    clock: &HLC,
    uv: bool,
    daemon_connections: &mut DaemonConnections,
) -> eyre::Result<RunningBuild> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;

    // Resolve each node to its target daemon, group git sources by daemon.
    let mut nodes_by_daemon: BTreeMap<DaemonId, Vec<NodeId>> = BTreeMap::new();
    let mut git_sources_by_daemon: BTreeMap<DaemonId, BTreeMap<NodeId, GitSource>> =
        BTreeMap::new();
    let mut prev_git_sources_by_daemon: BTreeMap<DaemonId, BTreeMap<NodeId, GitSource>> =
        BTreeMap::new();

    for (node_id, node) in &nodes {
        let daemon_id = resolve_daemon(daemon_connections, node.deploy.as_ref())?;
        nodes_by_daemon
            .entry(daemon_id.clone())
            .or_default()
            .push(node_id.clone());
        if let Some(gs) = git_sources.get(node_id) {
            git_sources_by_daemon
                .entry(daemon_id.clone())
                .or_default()
                .insert(node_id.clone(), gs.clone());
        }
        if let Some(gs) = prev_git_sources.get(node_id) {
            prev_git_sources_by_daemon
                .entry(daemon_id)
                .or_default()
                .insert(node_id.clone(), gs.clone());
        }
    }

    let mut daemons = BTreeSet::new();
    for (daemon_id, nodes_on_daemon) in &nodes_by_daemon {
        tracing::debug!(
            "Running dataflow build `{build_id}` on daemon `{daemon_id}` (nodes: {nodes_on_daemon:?})"
        );

        let build_command = BuildDataflowNodes {
            build_id,
            session_id,
            local_working_dir: local_working_dir.clone(),
            git_sources: git_sources_by_daemon.remove(daemon_id).unwrap_or_default(),
            prev_git_sources: prev_git_sources_by_daemon
                .remove(daemon_id)
                .unwrap_or_default(),
            dataflow_descriptor: dataflow.clone(),
            nodes_on_machine: nodes_on_daemon.iter().cloned().collect(),
            uv,
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::Build(build_command),
            timestamp: clock.new_timestamp(),
        })?;

        let daemon_connection = daemon_connections
            .get_mut(daemon_id)
            .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?;

        let reply_raw = daemon_connection
            .send_and_receive(&message)
            .await
            .wrap_err("failed to send/receive build message")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize build reply from daemon")?
        {
            DaemonCoordinatorReply::TriggerBuildResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("daemon returned an error")?,
            _ => bail!("unexpected reply"),
        }

        daemons.insert(daemon_id.clone());
    }

    tracing::info!("successfully triggered dataflow build `{build_id}`",);

    Ok(RunningBuild {
        errors: Vec::new(),
        build_result: CachedResult::default(),
        buffered_log_messages: Vec::new(),
        log_subscribers: Vec::new(),
        pending_build_results: daemons,
    })
}

#[allow(clippy::too_many_arguments)]
pub(crate) async fn start_dataflow(
    build_id: Option<BuildId>,
    session_id: SessionId,
    dataflow: Descriptor,
    local_working_dir: Option<PathBuf>,
    name: Option<String>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
    uv: bool,
    write_events_to: Option<PathBuf>,
) -> eyre::Result<RunningDataflow> {
    let SpawnedDataflow {
        uuid,
        daemons,
        nodes,
        node_to_daemon,
    } = spawn_dataflow(
        build_id,
        session_id,
        dataflow.clone(),
        local_working_dir,
        daemon_connections,
        clock,
        uv,
        write_events_to,
    )
    .await?;
    Ok(RunningDataflow {
        uuid,
        name,
        descriptor: dataflow,
        pending_daemons: if daemons.len() > 1 {
            daemons.clone()
        } else {
            BTreeSet::new()
        },
        exited_before_subscribe: Default::default(),
        daemons: daemons.clone(),
        nodes,
        node_to_daemon,
        node_metrics: BTreeMap::new(),
        network_metrics: None,
        spawn_result: CachedResult::default(),
        stop_reply_senders: Vec::new(),
        buffered_log_messages: Vec::new(),
        log_subscribers: Vec::new(),
        daemon_ack_sequence: daemons.iter().map(|d| (d.clone(), 0)).collect(),
        pending_spawn_results: daemons,
        created_at: state::now_millis(),
        store_generation: 0,
        last_recovery_attempt: BTreeMap::new(),
        uv,
        state_log_sequence: 0,
        state_log: Vec::new(),
    })
}

async fn destroy_daemon(
    daemon_id: DaemonId,
    daemon_connection: DaemonConnection,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Destroy,
        timestamp,
    })?;

    let reply_raw = daemon_connection
        .send_and_receive(&message)
        .await
        .wrap_err(format!(
            "failed to send/receive destroy message to daemon `{daemon_id}`"
        ))?;
    match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize destroy reply from daemon")?
    {
        DaemonCoordinatorReply::DestroyResult { result, .. } => result
            .map_err(|e| eyre!(e))
            .wrap_err("failed to destroy dataflow")?,
        other => bail!("unexpected reply after sending `destroy`: {other:?}"),
    }

    tracing::info!("successfully destroyed daemon `{daemon_id}`");
    Ok(())
}

async fn destroy_daemons(
    daemon_connections: &mut DaemonConnections,
    timestamp: uhlc::Timestamp,
) -> eyre::Result<()> {
    let futures = daemon_connections
        .drain()
        .map(|(daemon_id, daemon_connection)| {
            destroy_daemon(daemon_id, daemon_connection, timestamp)
        })
        .collect::<Vec<_>>();
    let results: Vec<std::result::Result<(), eyre::Error>> =
        join_all(futures).await.into_iter().collect::<Vec<_>>();
    for result in results {
        result?;
    }
    Ok(())
}
