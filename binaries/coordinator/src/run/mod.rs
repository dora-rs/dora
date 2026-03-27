use crate::DaemonConnections;

use dora_core::descriptor::DescriptorExt;
use dora_message::{
    BuildId, SessionId,
    common::DaemonId,
    coordinator_to_daemon::SpawnDataflowNodes,
    descriptor::{Descriptor, ResolvedNode},
    id::NodeId,
    tarpc,
};
use eyre::{ContextCompat, WrapErr, eyre};
use itertools::Itertools;
use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
};
use uuid::{NoContext, Timestamp, Uuid};

/// Plan a dataflow spawn without sending any commands to daemons yet.
///
/// Resolves nodes, generates a UUID, and determines which daemon handles
/// each node group. The actual spawn commands are prepared but not sent.
#[tracing::instrument(skip(daemon_connections))]
pub(super) fn plan_dataflow(
    dataflow_id: Option<Uuid>,
    build_id: Option<BuildId>,
    session_id: SessionId,
    dataflow: &Descriptor,
    local_working_dir: Option<PathBuf>,
    daemon_connections: &DaemonConnections,
    uv: bool,
    write_events_to: Option<PathBuf>,
    hot_reload: bool,
) -> eyre::Result<DataflowPlan> {
    let nodes = dataflow.resolve_aliases_and_set_defaults()?;
    let uuid = dataflow_id.unwrap_or_else(|| Uuid::new_v7(Timestamp::now(NoContext)));

    let nodes_by_daemon = nodes
        .values()
        .into_group_map_by(|n| n.deploy.as_ref().and_then(|d| d.machine.as_ref()));

    let mut daemons = BTreeSet::new();
    let mut node_to_daemon = BTreeMap::new();
    let mut daemon_spawn_commands: Vec<(DaemonId, SpawnDataflowNodes)> = Vec::new();

    for (machine, nodes_on_machine) in &nodes_by_daemon {
        let spawn_nodes = nodes_on_machine.iter().map(|n| n.id.clone()).collect();
        tracing::debug!(
            "Spawning dataflow `{uuid}` on machine `{machine:?}` (nodes: {spawn_nodes:?})"
        );

        let daemon_id = resolve_daemon_for_machine(daemon_connections, machine.map(|m| m.as_str()))
            .wrap_err_with(|| format!("failed to resolve daemon for machine `{machine:?}`"))?;

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
            hot_reload,
            dataflow_path: None,
        };

        daemon_spawn_commands.push((daemon_id.clone(), spawn_command));
        daemons.insert(daemon_id.clone());

        // Map each node on this machine to its daemon
        for node in nodes_on_machine {
            node_to_daemon.insert(node.id.clone(), daemon_id.clone());
        }
    }

    Ok(DataflowPlan {
        uuid,
        daemons,
        nodes,
        node_to_daemon,
        daemon_spawn_commands,
    })
}

/// Send the prepared spawn commands to the daemons via tarpc RPC calls.
pub(super) async fn execute_dataflow_plan(
    uuid: Uuid,
    daemon_spawn_commands: Vec<(DaemonId, SpawnDataflowNodes)>,
    daemon_connections: &DaemonConnections,
) -> eyre::Result<()> {
    for (daemon_id, spawn_command) in daemon_spawn_commands {
        let client = daemon_connections
            .get(&daemon_id)
            .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?
            .client
            .clone();
        // DashMap lock is dropped — safe to do async I/O.
        client
            .spawn(tarpc::context::current(), spawn_command)
            .await
            .wrap_err("RPC transport error")?
            .map_err(|e: String| eyre!(e))
            .wrap_err("daemon returned an error")?;
    }

    tracing::info!("successfully triggered dataflow spawn `{uuid}`",);

    Ok(())
}

fn resolve_daemon_for_machine(
    daemon_connections: &DaemonConnections,
    machine: Option<&str>,
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
    Ok(daemon_id)
}

pub struct DataflowPlan {
    pub uuid: Uuid,
    pub daemons: BTreeSet<DaemonId>,
    pub nodes: BTreeMap<NodeId, ResolvedNode>,
    pub node_to_daemon: BTreeMap<NodeId, DaemonId>,
    pub daemon_spawn_commands: Vec<(DaemonId, SpawnDataflowNodes)>,
}
