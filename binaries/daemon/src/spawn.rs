use crate::{
    log, node_communication::spawn_listener_loop, node_inputs, runtime_node_inputs,
    runtime_node_outputs, DoraEvent, Event, NodeExitStatus,
};
use dora_core::{
    config::{LocalCommunicationConfig, NodeRunConfig},
    daemon_messages::{DataflowId, NodeConfig, RuntimeConfig},
    descriptor::{resolve_path, source_is_url, OperatorSource, ResolvedNode, SHELL_SOURCE},
};
use dora_download::download_file;
use eyre::WrapErr;
use std::{
    env::{consts::EXE_EXTENSION, temp_dir},
    path::Path,
    process::Stdio,
};
use tokio::{
    fs::File,
    io::{AsyncBufReadExt, AsyncWriteExt},
    sync::mpsc,
};
use tracing::{debug, error};

pub async fn spawn_node(
    dataflow_id: DataflowId,
    working_dir: &Path,
    node: ResolvedNode,
    daemon_tx: mpsc::Sender<Event>,
    config: LocalCommunicationConfig,
) -> eyre::Result<()> {
    let node_id = node.id.clone();
    tracing::debug!("Spawning node `{dataflow_id}/{node_id}`");

    let queue_sizes = node_inputs(&node)
        .into_iter()
        .map(|(k, v)| (k, v.queue_size.unwrap_or(10)))
        .collect();
    let daemon_communication =
        spawn_listener_loop(&dataflow_id, &node_id, &daemon_tx, config, queue_sizes).await?;

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
            command
                .stdin(Stdio::null())
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .wrap_err_with(move || {
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
                command.args([
                    "-c",
                    format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                ]);
                command
            } else if !has_python_operator && has_other_operator {
                let mut cmd = tokio::process::Command::new(
                    std::env::current_exe().wrap_err("failed to get current executable path")?,
                );
                cmd.arg("--run-dora-runtime");
                cmd
            } else {
                eyre::bail!("Runtime can not mix Python Operator with other type of operator.");
            };
            command.current_dir(working_dir);

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
            // Injecting the env variable defined in the `yaml` into
            // the node runtime.
            if let Some(envs) = node.env {
                for (key, value) in envs {
                    command.env(key, value.to_string());
                }
            }

            command
                .stdin(Stdio::null())
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
                .wrap_err(format!(
                    "failed to run runtime {}/{}",
                    runtime_config.node.dataflow_id, runtime_config.node.node_id
                ))?
        }
    };

    let log_dir = temp_dir();

    let (tx, mut rx) = mpsc::channel(10);
    let mut file =
        File::create(&log_dir.join(log::log_path(&dataflow_id, &node_id).with_extension("txt")))
            .await
            .expect("Failed to create log file");
    let mut stdout_lines =
        (tokio::io::BufReader::new(child.stdout.take().expect("failed to take stdout"))).lines();

    let stdout_tx = tx.clone();

    // Stdout listener stream
    tokio::spawn(async move {
        while let Ok(Some(line)) = stdout_lines.next_line().await {
            let sent = stdout_tx.send(line.clone()).await;
            if sent.is_err() {
                println!("Could not log: {line}");
            }
        }
    });

    let mut stderr_lines =
        (tokio::io::BufReader::new(child.stderr.take().expect("failed to take stderr"))).lines();

    // Stderr listener stream
    let stderr_tx = tx.clone();
    tokio::spawn(async move {
        while let Ok(Some(line)) = stderr_lines.next_line().await {
            let sent = stderr_tx.send(line.clone()).await;
            if sent.is_err() {
                eprintln!("Could not log: {line}");
            }
        }
    });

    tokio::spawn(async move {
        let exit_status = NodeExitStatus::from(child.wait().await);
        let event = DoraEvent::SpawnedNodeResult {
            dataflow_id,
            node_id,
            exit_status,
        };

        let _ = daemon_tx.send(event.into()).await;
    });

    // Log to file stream.
    tokio::spawn(async move {
        while let Some(line) = rx.recv().await {
            let _ = file
                .write_all(line.as_bytes())
                .await
                .map_err(|err| error!("Could not log {line} to file due to {err}"));
            let _ = file
                .write(b"\n")
                .await
                .map_err(|err| error!("Could not add newline to log file due to {err}"));
            debug!("{dataflow_id}/{} logged {line}", node.id.clone());
            // Make sure that all data has been synced to disk.
            let _ = file
                .sync_all()
                .await
                .map_err(|err| error!("Could not sync logs to file due to {err}"));
        }
    });
    Ok(())
}
