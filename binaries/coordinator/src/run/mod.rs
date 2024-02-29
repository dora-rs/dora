use crate::{
    tcp_utils::{tcp_receive, tcp_send},
    DaemonConnection,
};

use dora_core::{
    daemon_messages::{
        DaemonCoordinatorEvent, DaemonCoordinatorReply, SpawnDataflowNodes, Timestamped,
    },
    descriptor::{Descriptor, ResolvedNode},
    message::uhlc::HLC,
};
use eyre::{bail, eyre, ContextCompat, WrapErr};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    path::PathBuf,
};
use uuid::{NoContext, Timestamp, Uuid};

#[tracing::instrument(skip(daemon_connections, clock))]
pub(super) async fn spawn_dataflow(
    dataflow: Descriptor,
    working_dir: PathBuf,
    daemon_connections: &mut HashMap<String, DaemonConnection>,
    clock: &HLC,
) -> eyre::Result<SpawnedDataflow> {
    dataflow.check(&working_dir)?;

    let nodes = dataflow.resolve_aliases_and_set_defaults();
    let uuid = Uuid::new_v7(Timestamp::now(NoContext));

    let machines: BTreeSet<_> = nodes.iter().map(|n| n.deploy.machine.clone()).collect();
    let machine_listen_ports = machines
        .iter()
        .map(|m| {
            daemon_connections
                .get(m)
                .ok_or_else(|| eyre!("no daemon listen port for machine `{m}`"))
                .map(|c| (m.clone(), c.listen_socket))
        })
        .collect::<Result<BTreeMap<_, _>, _>>()?;

    let spawn_command = SpawnDataflowNodes {
        dataflow_id: uuid,
        working_dir,
        nodes: nodes.clone(),
        machine_listen_ports,
        dataflow_descriptor: dataflow,
    };
    let message = serde_json::to_vec(&Timestamped {
        inner: DaemonCoordinatorEvent::Spawn(spawn_command),
        timestamp: clock.new_timestamp(),
    })?;

    for machine in &machines {
        tracing::trace!("Spawning dataflow `{uuid}` on machine `{machine}`");
        spawn_dataflow_on_machine(daemon_connections, machine, &message)
            .await
            .wrap_err_with(|| format!("failed to spawn dataflow on machine `{machine}`"))?;
    }

    tracing::info!("successfully spawned dataflow `{uuid}`");

    Ok(SpawnedDataflow {
        uuid,
        machines,
        nodes,
    })
}

async fn spawn_dataflow_on_machine(
    daemon_connections: &mut HashMap<String, DaemonConnection>,
    machine: &str,
    message: &[u8],
) -> Result<(), eyre::ErrReport> {
    let daemon_connection = daemon_connections
        .get_mut(machine)
        .wrap_err_with(|| format!("no daemon connection for machine `{machine}`"))?;
    tcp_send(&mut daemon_connection.stream, message)
        .await
        .wrap_err("failed to send spawn message to daemon")?;
    let reply_raw = tcp_receive(&mut daemon_connection.stream)
        .await
        .wrap_err("failed to receive spawn reply from daemon")?;
    match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize spawn reply from daemon")?
    {
        DaemonCoordinatorReply::SpawnResult(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("daemon returned an error")?,
        _ => bail!("unexpected reply"),
    }
    Ok(())
}

pub struct SpawnedDataflow {
    pub uuid: Uuid,
    pub machines: BTreeSet<String>,
    pub nodes: Vec<ResolvedNode>,
}
