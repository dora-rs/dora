use crate::{
    listener::spawn_listener_loop, runtime_node_inputs, runtime_node_outputs, shared_mem_handler,
    DoraEvent, Event,
};
use dora_core::{
    config::NodeRunConfig,
    daemon_messages::{DaemonCommunicationConfig, DataflowId, NodeConfig, RuntimeConfig},
    descriptor::{resolve_path, source_is_url, OperatorSource, ResolvedNode},
};
use dora_download::download_file;
use eyre::{eyre, WrapErr};
use std::{env::consts::EXE_EXTENSION, path::Path, process::Stdio};
use tokio::sync::mpsc;

pub async fn spawn_node(
    dataflow_id: DataflowId,
    working_dir: &Path,
    node: ResolvedNode,
    daemon_tx: mpsc::Sender<Event>,
    shmem_handler_tx: flume::Sender<shared_mem_handler::NodeEvent>,
    config: DaemonCommunicationConfig,
) -> eyre::Result<()> {
    let node_id = node.id.clone();
    tracing::debug!("Spawning node `{dataflow_id}/{node_id}`");

    let daemon_communication = spawn_listener_loop(
        &dataflow_id,
        &node_id,
        &daemon_tx,
        &shmem_handler_tx,
        config,
    )
    .await?;

    let mut child = match node.kind {
        dora_core::descriptor::CoreNodeKind::Custom(n) => {
            let resolved_path = if source_is_url(&n.source) {
                // try to download the shared library
                let target_path = Path::new("build")
                    .join(node_id.to_string())
                    .with_extension(EXE_EXTENSION);
                download_file(&n.source, &target_path)
                    .await
                    .wrap_err("failed to download custom node")?;
                target_path.clone()
            } else {
                resolve_path(&n.source, working_dir)
                    .wrap_err_with(|| format!("failed to resolve node source `{}`", n.source))?
            };

            tracing::info!("spawning {}", resolved_path.display());
            let mut command = tokio::process::Command::new(&resolved_path);
            command.current_dir(working_dir);
            command.stdin(Stdio::null());
            let node_config = NodeConfig {
                dataflow_id,
                node_id: node_id.clone(),
                run_config: n.run_config.clone(),
                daemon_communication,
            };
            if let Some(args) = &n.args {
                command.args(args.split_ascii_whitespace());
            }
            command.env(
                "DORA_NODE_CONFIG",
                serde_yaml::to_string(&node_config).wrap_err("failed to serialize node config")?,
            );
            // Injecting the env variable defined in the `yaml` into
            // the node runtime.
            if let Some(envs) = n.envs {
                for (key, value) in envs {
                    command.env(key, value.to_string());
                }
            }
            command.spawn().wrap_err_with(move || {
                format!(
                    "failed to run source path: `{}` with args `{}`",
                    resolved_path.display(),
                    n.args.as_deref().unwrap_or_default()
                )
            })?
        }
        dora_core::descriptor::CoreNodeKind::Runtime(n) => {
            let has_python_operator = n
                .operators
                .iter()
                .any(|x| matches!(x.config.source, OperatorSource::Python { .. }));

            let has_other_operator = n
                .operators
                .iter()
                .any(|x| !matches!(x.config.source, OperatorSource::Python { .. }));

            let mut command = if has_python_operator && !has_other_operator {
                // Use python to spawn runtime if there is a python operator
                let mut command = tokio::process::Command::new("python3");
                command.args(["-c", "import dora; dora.start_runtime()"]);
                command
            } else if !has_python_operator && has_other_operator {
                tokio::process::Command::new("dora-runtime")
            } else {
                eyre::bail!("Runtime can not mix Python Operator with other type of operator.");
            };
            command.current_dir(working_dir);
            command.stdin(Stdio::null());

            let runtime_config = RuntimeConfig {
                node: NodeConfig {
                    dataflow_id,
                    node_id: node_id.clone(),
                    run_config: NodeRunConfig {
                        inputs: runtime_node_inputs(&n),
                        outputs: runtime_node_outputs(&n),
                    },
                    daemon_communication,
                },
                operators: n.operators,
            };
            command.env(
                "DORA_RUNTIME_CONFIG",
                serde_yaml::to_string(&runtime_config)
                    .wrap_err("failed to serialize runtime config")?,
            );

            command.spawn().wrap_err("failed to run runtime")?
        }
    };

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
        let _ = daemon_tx.send(event.into()).await;
    });
    Ok(())
}
