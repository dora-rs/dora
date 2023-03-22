use crate::{
    listener::spawn_listener_loop, runtime_node_inputs, runtime_node_outputs, DoraEvent, Event,
    NodeExitStatus,
};
use dora_core::{
    config::NodeRunConfig,
    daemon_messages::{DaemonCommunicationConfig, DataflowId, NodeConfig, RuntimeConfig},
    descriptor::{resolve_path, source_is_url, OperatorSource, ResolvedNode},
};
use dora_download::download_file;
use eyre::WrapErr;
use std::{env::consts::EXE_EXTENSION, path::Path, process::Stdio};
use tokio::sync::mpsc;

const SHELL_SOURCE: &str = "shell";

pub async fn spawn_node(
    dataflow_id: DataflowId,
    working_dir: &Path,
    node: ResolvedNode,
    daemon_tx: mpsc::Sender<Event>,
    config: DaemonCommunicationConfig,
    dora_runtime_path: Option<&Path>,
) -> eyre::Result<()> {
    let node_id = node.id.clone();
    tracing::debug!("Spawning node `{dataflow_id}/{node_id}`");

    let daemon_communication =
        spawn_listener_loop(&dataflow_id, &node_id, &daemon_tx, config, node.queue_size).await?;

    let mut child = match node.kind {
        dora_core::descriptor::CoreNodeKind::Custom(n) => {
            let mut command = match n.source.as_str() {
                SHELL_SOURCE => {
                    if cfg!(target_os = "windows") {
                        let mut cmd = tokio::process::Command::new("cmd");
                        cmd.args(["/C", &n.args.clone().unwrap_or_default()]);
                        cmd
                    } else {
                        let mut cmd = tokio::process::Command::new("sh");
                        cmd.args(["-c", &n.args.clone().unwrap_or_default()]);
                        cmd
                    }
                }
                source => {
                    let resolved_path = if source_is_url(source) {
                        // try to download the shared library
                        let target_path = Path::new("build")
                            .join(node_id.to_string())
                            .with_extension(EXE_EXTENSION);
                        download_file(source, &target_path)
                            .await
                            .wrap_err("failed to download custom node")?;
                        target_path.clone()
                    } else {
                        resolve_path(source, working_dir).wrap_err_with(|| {
                            format!("failed to resolve node source `{}`", source)
                        })?
                    };

                    tracing::info!("spawning {}", resolved_path.display());
                    let mut cmd = tokio::process::Command::new(&resolved_path);
                    if let Some(args) = &n.args {
                        cmd.args(args.split_ascii_whitespace());
                    }
                    cmd
                }
            };

            command.current_dir(working_dir);
            command.stdin(Stdio::null());
            let node_config = NodeConfig {
                dataflow_id,
                node_id: node_id.clone(),
                run_config: n.run_config.clone(),
                daemon_communication,
            };

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
                    "failed to run `{}` with args `{}`",
                    n.source,
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
                tokio::process::Command::new(
                    dora_runtime_path.unwrap_or_else(|| Path::new("dora-runtime")),
                )
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

    tokio::spawn(async move {
        let exit_status = NodeExitStatus::from(child.wait().await);
        let event = DoraEvent::SpawnedNodeResult {
            dataflow_id,
            node_id,
            exit_status,
        };
        let _ = daemon_tx.send(event.into()).await;
    });
    Ok(())
}
