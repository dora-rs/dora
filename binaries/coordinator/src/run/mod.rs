use crate::{
    DaemonConnections,
    tcp_utils::{tcp_receive, tcp_send},
};

use dora_core::{descriptor::DescriptorExt, uhlc::HLC};
use dora_message::{
    BuildId, SessionId,
    common::DaemonId,
    coordinator_to_daemon::{DaemonCoordinatorEvent, SpawnDataflowNodes, Timestamped},
    daemon_to_coordinator::DaemonCoordinatorReply,
    descriptor::{Descriptor, ResolvedNode},
    id::NodeId,
};
use eyre::{ContextCompat, WrapErr, bail, eyre};
use itertools::Itertools;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};
use uuid::{NoContext, Timestamp, Uuid};

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

    let nodes_by_daemon = nodes
        .values()
        .into_group_map_by(|n| n.deploy.as_ref().and_then(|d| d.machine.as_ref()));

    let mut daemons = BTreeSet::new();
    let mut node_to_daemon = BTreeMap::new();

    for (machine, nodes_on_machine) in &nodes_by_daemon {
        let spawn_nodes = nodes_on_machine.iter().map(|n| n.id.clone()).collect();
        tracing::debug!(
            "Spawning dataflow `{uuid}` on machine `{machine:?}` (nodes: {spawn_nodes:?})"
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
        };
        let message = serde_json::to_vec(&Timestamped {
            inner: DaemonCoordinatorEvent::Spawn(spawn_command),
            timestamp: clock.new_timestamp(),
        })?;

        let daemon_id =
            spawn_dataflow_on_machine(daemon_connections, machine.map(|m| m.as_str()), &message)
                .await
                .wrap_err_with(|| format!("failed to spawn dataflow on machine `{machine:?}`"))?;
        daemons.insert(daemon_id.clone());

        // Map each node on this machine to its daemon
        for node in nodes_on_machine {
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

async fn spawn_dataflow_on_machine(
    daemon_connections: &mut DaemonConnections,
    machine: Option<&str>,
    message: &[u8],
) -> Result<DaemonId, eyre::ErrReport> {
    let daemon_id = match machine {
        Some(machine) => daemon_connections
            .get_matching_daemon_id(machine)
            .wrap_err_with(|| format!("no matching daemon for machine id {machine:?}"))?
            .clone(),
        None => daemon_connections
            .unnamed()
            .next()
            .wrap_err("no unnamed daemon connections")?
            .clone(),
    };

    let daemon_connection = daemon_connections
        .get_mut(&daemon_id)
        .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?;
    tcp_send(&mut daemon_connection.stream, message)
        .await
        .wrap_err("failed to send spawn message to daemon")?;

    let reply_raw = tcp_receive(&mut daemon_connection.stream)
        .await
        .wrap_err("failed to receive spawn reply from daemon")?;
    match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize spawn reply from daemon")?
    {
        DaemonCoordinatorReply::TriggerSpawnResult(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("daemon returned an error")?,
        _ => bail!("unexpected reply"),
    }
    Ok(daemon_id)
}

pub struct SpawnedDataflow {
    pub uuid: Uuid,
    pub daemons: BTreeSet<DaemonId>,
    pub nodes: BTreeMap<NodeId, ResolvedNode>,
    pub node_to_daemon: BTreeMap<NodeId, DaemonId>,
}
