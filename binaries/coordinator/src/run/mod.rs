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
    let mut started_daemons = Vec::new();

    for (daemon_id, spawn_command) in daemon_spawn_commands {
        let client = daemon_connections
            .get(&daemon_id)
            .wrap_err_with(|| format!("no daemon connection for daemon `{daemon_id}`"))?
            .client
            .clone();
        // DashMap lock is dropped — safe to do async I/O.
        let spawn_result = client
            .spawn(tarpc::context::current(), spawn_command)
            .await
            .wrap_err("RPC transport error")
            .and_then(|result| {
                result
                    .map_err(|e: String| eyre!(e))
                    .wrap_err("daemon returned an error")
            });

        match spawn_result {
            Ok(()) => started_daemons.push(daemon_id),
            Err(spawn_error) => {
                let rollback_errors =
                    rollback_spawned_daemons(uuid, &started_daemons, daemon_connections).await;
                if rollback_errors.is_empty() {
                    return Err(spawn_error.wrap_err(format!(
                        "spawn failed while starting dataflow `{uuid}`; \
                         rollback succeeded on {} daemon(s)",
                        started_daemons.len()
                    )));
                }

                let rollback_summary = rollback_errors.join("; ");
                return Err(spawn_error.wrap_err(format!(
                    "spawn failed while starting dataflow `{uuid}`; \
                     rollback attempted on {} daemon(s) but encountered errors: {rollback_summary}",
                    started_daemons.len()
                )));
            }
        }
    }

    tracing::info!("successfully triggered dataflow spawn `{uuid}`",);

    Ok(())
}

async fn rollback_spawned_daemons(
    dataflow_id: Uuid,
    started_daemons: &[DaemonId],
    daemon_connections: &DaemonConnections,
) -> Vec<String> {
    let mut errors = Vec::new();

    for daemon_id in started_daemons {
        let Some(connection) = daemon_connections.get(daemon_id) else {
            errors.push(format!(
                "missing daemon connection for `{daemon_id}` during rollback"
            ));
            continue;
        };
        let client = connection.client.clone();
        drop(connection);

        let result = client
            .stop_dataflow(tarpc::context::current(), dataflow_id, None, false)
            .await
            .map_err(|e| eyre!(e))
            .and_then(|r| r.map_err(|e: String| eyre!(e)));

        if let Err(err) = result {
            errors.push(format!(
                "failed to stop dataflow `{dataflow_id}` on daemon `{daemon_id}`: {err:?}"
            ));
        }
    }

    errors
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
