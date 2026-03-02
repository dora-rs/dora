use crate::DaemonConnections;

use adora_core::{descriptor::DescriptorExt, uhlc::HLC};
use adora_message::{
    BuildId, SessionId,
    common::DaemonId,
    coordinator_to_daemon::{DaemonCoordinatorEvent, SpawnDataflowNodes, Timestamped},
    daemon_to_coordinator::DaemonCoordinatorReply,
    descriptor::{Deploy, Descriptor, ResolvedNode},
    id::NodeId,
};
use eyre::{ContextCompat, WrapErr, bail, eyre};
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};
use uuid::{NoContext, Timestamp, Uuid};

#[allow(clippy::too_many_arguments)]
#[tracing::instrument(skip(daemon_connections, clock))]
pub(super) async fn spawn_dataflow(
    build_id: Option<BuildId>,
    session_id: SessionId,
    dataflow: Descriptor,
    local_working_dir: Option<PathBuf>,
    daemon_connections: &mut DaemonConnections,
    clock: &HLC,
    uv: bool,
    write_events_to: Option<PathBuf>,
) -> eyre::Result<SpawnedDataflow> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;
    let uuid = Uuid::new_v7(Timestamp::now(NoContext));

    // Resolve each node to its target daemon, then group by daemon.
    let mut nodes_by_daemon: BTreeMap<DaemonId, Vec<&ResolvedNode>> = BTreeMap::new();
    for node in nodes.values() {
        let daemon_id = resolve_daemon(daemon_connections, node.deploy.as_ref())?;
        nodes_by_daemon.entry(daemon_id).or_default().push(node);
    }

    let mut daemons = BTreeSet::new();
    let mut node_to_daemon = BTreeMap::new();

    for (daemon_id, nodes_on_daemon) in &nodes_by_daemon {
        let spawn_nodes = nodes_on_daemon.iter().map(|n| n.id.clone()).collect();
        tracing::debug!(
            "Spawning dataflow `{uuid}` on daemon `{daemon_id}` (nodes: {spawn_nodes:?})"
        );

        let spawn_command = SpawnDataflowNodes {
            build_id,
            session_id,
            dataflow_id: uuid,
            local_working_dir: local_working_dir.clone(),
            nodes: nodes.clone(),
            dataflow_descriptor: dataflow.clone(),
            spawn_nodes,
            uv,
            write_events_to: write_events_to.clone(),
            artifact_base_url: None,
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::Spawn(spawn_command),
            timestamp: clock.new_timestamp(),
        })?;

        let daemon_connection = daemon_connections
            .get_mut(daemon_id)
            .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?;

        let reply_raw = daemon_connection
            .send_and_receive(&message)
            .await
            .wrap_err("failed to send/receive spawn message")?;
        match serde_json::from_slice(&reply_raw)
            .wrap_err("failed to deserialize spawn reply from daemon")?
        {
            DaemonCoordinatorReply::TriggerSpawnResult(result) => result
                .map_err(|e| eyre!(e))
                .wrap_err("daemon returned an error")?,
            _ => bail!("unexpected reply"),
        }

        daemons.insert(daemon_id.clone());
        for node in nodes_on_daemon {
            node_to_daemon.insert(node.id.clone(), daemon_id.clone());
        }
    }

    tracing::info!("successfully triggered dataflow spawn `{uuid}`",);

    Ok(SpawnedDataflow {
        uuid,
        daemons,
        nodes,
        node_to_daemon,
    })
}

/// Resolve which daemon should run a node based on its deploy config.
/// Priority: machine > labels > unnamed.
pub(super) fn resolve_daemon(
    connections: &DaemonConnections,
    deploy: Option<&Deploy>,
) -> eyre::Result<DaemonId> {
    match deploy {
        Some(Deploy {
            machine: Some(machine),
            ..
        }) => connections
            .get_matching_daemon_id(machine)
            .cloned()
            .wrap_err_with(|| format!("no matching daemon for machine id `{machine}`")),
        Some(d) if !d.labels.is_empty() => connections
            .get_matching_daemon_by_labels(&d.labels)
            .cloned()
            .wrap_err_with(|| format!("no daemon matches labels {:?}", d.labels)),
        _ => connections
            .unnamed()
            .next()
            .cloned()
            .wrap_err("no unnamed daemon connections"),
    }
}

pub struct SpawnedDataflow {
    pub uuid: Uuid,
    pub daemons: BTreeSet<DaemonId>,
    pub nodes: BTreeMap<NodeId, ResolvedNode>,
    pub node_to_daemon: BTreeMap<NodeId, DaemonId>,
}
