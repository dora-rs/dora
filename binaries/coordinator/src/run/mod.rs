use crate::tcp_utils::{tcp_receive, tcp_send};

use dora_core::{
    config::{CommunicationConfig, NodeId},
    daemon_messages::{DaemonCoordinatorEvent, DaemonCoordinatorReply, SpawnDataflowNodes},
    descriptor::{CoreNodeKind, Descriptor},
};
use eyre::{bail, eyre, ContextCompat, WrapErr};
use futures::{stream::FuturesUnordered, StreamExt};
use std::{
    collections::{BTreeMap, BTreeSet, HashMap},
    env::consts::EXE_EXTENSION,
    path::Path,
};
use tokio::net::TcpStream;
use uuid::Uuid;

mod runtime;

pub async fn spawn_dataflow(
    runtime: &Path,
    dataflow_path: &Path,
    daemon_connections: &mut HashMap<String, TcpStream>,
) -> eyre::Result<SpawnedDataflow> {
    let mut runtime = runtime.with_extension(EXE_EXTENSION);
    let descriptor = read_descriptor(dataflow_path).await.wrap_err_with(|| {
        format!(
            "failed to read dataflow descriptor at {}",
            dataflow_path.display()
        )
    })?;
    let working_dir = dataflow_path
        .canonicalize()
        .context("failed to canoncialize dataflow path")?
        .parent()
        .ok_or_else(|| eyre!("canonicalized dataflow path has no parent"))?
        .to_owned();
    let nodes = descriptor.resolve_aliases();
    let uuid = Uuid::new_v4();
    let communication_config = {
        let mut config = descriptor.communication;
        // add uuid as prefix to ensure isolation
        config.add_topic_prefix(&uuid.to_string());
        config
    };
    if nodes
        .iter()
        .any(|n| matches!(n.kind, CoreNodeKind::Runtime(_)))
    {
        match which::which(runtime.as_os_str()) {
            Ok(path) => {
                runtime = path;
            }
            Err(err) => {
                let err = eyre!(err).wrap_err(format!(
                    "There is no runtime at {}, or it is not a file",
                    runtime.display()
                ));
                bail!("{err:?}")
            }
        }
    }

    let spawn_command = SpawnDataflowNodes {
        dataflow_id: uuid,
        working_dir,
        nodes,
    };
    let message = serde_json::to_vec(&DaemonCoordinatorEvent::Spawn(spawn_command))?;

    // TODO allow partitioning a dataflow across multiple machines
    let machine_id = "";
    let machines = [machine_id.to_owned()].into();

    let daemon_connection = daemon_connections
        .get_mut(machine_id)
        .wrap_err("no daemon connection")?; // TODO: take from dataflow spec
    tcp_send(daemon_connection, &message)
        .await
        .wrap_err("failed to send spawn message to daemon")?;

    // wait for reply
    let reply_raw = tcp_receive(daemon_connection)
        .await
        .wrap_err("failed to receive spawn reply from daemon")?;
    match serde_json::from_slice(&reply_raw)
        .wrap_err("failed to deserialize spawn reply from daemon")?
    {
        DaemonCoordinatorReply::SpawnResult(result) => result
            .map_err(|e| eyre!(e))
            .wrap_err("failed to spawn dataflow")?,
        _ => bail!("unexpected reply"),
    }
    tracing::info!("successfully spawned dataflow `{uuid}`");

    Ok(SpawnedDataflow {
        communication_config,
        uuid,
        machines,
    })
}

pub struct SpawnedDataflow {
    pub uuid: Uuid,
    pub communication_config: CommunicationConfig,
    pub machines: BTreeSet<String>,
}

async fn read_descriptor(file: &Path) -> Result<Descriptor, eyre::Error> {
    let descriptor_file = tokio::fs::read(file)
        .await
        .context("failed to open given file")?;
    let descriptor: Descriptor =
        serde_yaml::from_slice(&descriptor_file).context("failed to parse given descriptor")?;
    Ok(descriptor)
}

fn command_init_common_env(
    command: &mut tokio::process::Command,
    node_id: &NodeId,
    communication: &dora_core::config::CommunicationConfig,
) -> Result<(), eyre::Error> {
    command.env(
        "DORA_NODE_ID",
        serde_yaml::to_string(&node_id).wrap_err("failed to serialize custom node ID")?,
    );
    command.env(
        "DORA_COMMUNICATION_CONFIG",
        serde_yaml::to_string(communication)
            .wrap_err("failed to serialize communication config")?,
    );
    Ok(())
}
