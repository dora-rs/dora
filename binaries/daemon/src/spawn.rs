use crate::{listener::listener_loop, shared_mem_handler, DoraEvent, Event};
use dora_core::{
    daemon_messages::{DataflowId, NodeConfig, SpawnNodeParams},
    descriptor::{resolve_path, source_is_url},
};
use dora_download::download_file;
use eyre::{eyre, WrapErr};
use shared_memory_server::{ShmemConf, ShmemServer};
use std::{env::consts::EXE_EXTENSION, path::Path, process::Stdio};

#[tracing::instrument]
pub async fn spawn_node(
    dataflow_id: DataflowId,
    params: SpawnNodeParams,
    daemon_tx: flume::Sender<Event>,
    shmem_handler_tx: flume::Sender<shared_mem_handler::Event>,
) -> eyre::Result<()> {
    let SpawnNodeParams {
        node_id,
        node,
        working_dir,
    } = params;

    tracing::trace!("Spawning node `{dataflow_id}/{node_id}`");

    let resolved_path = if source_is_url(&node.source) {
        // try to download the shared library
        let target_path = Path::new("build")
            .join(node_id.to_string())
            .with_extension(EXE_EXTENSION);
        download_file(&node.source, &target_path)
            .await
            .wrap_err("failed to download custom node")?;
        target_path.clone()
    } else {
        resolve_path(&node.source, &working_dir)
            .wrap_err_with(|| format!("failed to resolve node source `{}`", node.source))?
    };

    let daemon_control_region = ShmemConf::new()
        .size(4096)
        .create()
        .wrap_err("failed to allocate daemon_control_region")?;
    let daemon_events_region = ShmemConf::new()
        .size(4096)
        .create()
        .wrap_err("failed to allocate daemon_events_region")?;
    let node_config = NodeConfig {
        dataflow_id,
        node_id: node_id.clone(),
        run_config: node.run_config.clone(),
        daemon_control_region_id: daemon_control_region.get_os_id().to_owned(),
        daemon_events_region_id: daemon_events_region.get_os_id().to_owned(),
    };

    {
        let server = unsafe { ShmemServer::new(daemon_control_region) }
            .wrap_err("failed to create control server")?;
        let daemon_tx = daemon_tx.clone();
        let shmem_handler_tx = shmem_handler_tx.clone();
        std::thread::spawn(move || listener_loop(server, daemon_tx, shmem_handler_tx));
    }
    {
        let server = unsafe { ShmemServer::new(daemon_events_region) }
            .wrap_err("failed to create events server")?;
        let event_loop_node_id = format!("{dataflow_id}/{node_id}");
        let daemon_tx = daemon_tx.clone();
        let shmem_handler_tx = shmem_handler_tx.clone();
        std::thread::spawn(move || {
            listener_loop(server, daemon_tx, shmem_handler_tx);
            tracing::debug!("event listener loop finished for `{event_loop_node_id}`");
        });
    }

    let mut command = tokio::process::Command::new(&resolved_path);
    if let Some(args) = &node.args {
        command.args(args.split_ascii_whitespace());
    }
    command.env(
        "DORA_NODE_CONFIG",
        serde_yaml::to_string(&node_config).wrap_err("failed to serialize node config")?,
    );
    command.current_dir(working_dir);

    // Injecting the env variable defined in the `yaml` into
    // the node runtime.
    if let Some(envs) = node.envs {
        for (key, value) in envs {
            command.env(key, value.to_string());
        }
    }
    command.stdin(Stdio::null());

    let mut child = command.spawn().wrap_err_with(move || {
        format!(
            "failed to run source path: `{}` with args `{}`",
            resolved_path.display(),
            node.args.as_deref().unwrap_or_default()
        )
    })?;
    let node_id_cloned = node_id.clone();
    let wait_task = async move {
        let status = child.wait().await.context("child process failed")?;
        if status.success() {
            Ok(())
        } else if let Some(code) = status.code() {
            Err(eyre!("node {node_id} failed with exit code: {code}"))
        } else {
            Err(eyre!("node {node_id} failed (unknown exit code)"))
        }
    };
    tokio::spawn(async move {
        let result = wait_task.await;
        let event = DoraEvent::SpawnedNodeResult {
            dataflow_id,
            node_id: node_id_cloned,
            result,
        };
        let _ = daemon_tx.send_async(event.into()).await;
    });
    Ok(())
}
