use crate::{
    log, node_communication::spawn_listener_loop, node_inputs, DoraEvent, Event, NodeExitStatus,
    OutputId, RunningNode,
};
use aligned_vec::{AVec, ConstAlign};
use crossbeam::queue::ArrayQueue;
use dora_arrow_convert::IntoArrow;
use dora_core::{
    config::DataId,
    daemon_messages::{DataMessage, DataflowId, NodeConfig, RuntimeConfig, Timestamped},
    descriptor::{
        resolve_path, source_is_url, Descriptor, OperatorDefinition, OperatorSource, PythonSource,
        ResolvedNode, DYNAMIC_SOURCE, SHELL_SOURCE,
    },
    get_python_path,
    message::uhlc::HLC,
};
use dora_download::download_file;
use dora_node_api::{
    arrow::array::ArrayData,
    arrow_utils::{copy_array_into_sample, required_data_size},
    Metadata,
};
use eyre::{ContextCompat, WrapErr};
use std::{
    env::consts::EXE_EXTENSION,
    path::{Path, PathBuf},
    process::Stdio,
    sync::Arc,
};
use tokio::{
    fs::File,
    io::{AsyncBufReadExt, AsyncWriteExt},
    sync::{mpsc, oneshot},
};
use tracing::{debug, error};

/// clock is required for generating timestamps when dropping messages early because queue is full
pub async fn spawn_node(
    dataflow_id: DataflowId,
    working_dir: &Path,
    node: ResolvedNode,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    dataflow_descriptor: Descriptor,
    clock: Arc<HLC>,
    node_stderr_most_recent: Arc<ArrayQueue<String>>,
) -> eyre::Result<RunningNode> {
    let node_id = node.id.clone();
    tracing::debug!("Spawning node `{dataflow_id}/{node_id}`");

    let queue_sizes = node_inputs(&node)
        .into_iter()
        .map(|(k, v)| (k, v.queue_size.unwrap_or(10)))
        .collect();
    let daemon_communication = spawn_listener_loop(
        &dataflow_id,
        &node_id,
        &daemon_tx,
        dataflow_descriptor.communication.local,
        queue_sizes,
        clock.clone(),
    )
    .await?;
    let send_stdout_to = node
        .send_stdout_as()
        .context("Could not resolve `send_stdout_as` configuration")?;

    let node_config = NodeConfig {
        dataflow_id,
        node_id: node_id.clone(),
        run_config: node.kind.run_config(),
        daemon_communication,
        dataflow_descriptor,
        dynamic: node.kind.dynamic(),
    };

    let mut child = match node.kind {
        dora_core::descriptor::CoreNodeKind::Custom(n) => {
            let mut command = match n.source.as_str() {
                DYNAMIC_SOURCE => {
                    return Ok(RunningNode {
                        pid: None,
                        node_config,
                    });
                }
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

                    // If extension is .py, use python to run the script
                    let mut cmd = match resolved_path.extension().map(|ext| ext.to_str()) {
                        Some(Some("py")) => {
                            let python = get_python_path().context("Could not get python path")?;
                            tracing::info!("spawning: {:?} {}", &python, resolved_path.display());
                            let mut cmd = tokio::process::Command::new(&python);
                            cmd.arg(&resolved_path);
                            cmd
                        }
                        _ => {
                            tracing::info!("spawning: {}", resolved_path.display());
                            tokio::process::Command::new(&resolved_path)
                        }
                    };

                    if let Some(args) = &n.args {
                        cmd.args(args.split_ascii_whitespace());
                    }
                    cmd
                }
            };

            command.current_dir(working_dir);
            command.stdin(Stdio::null());

            command.env(
                "DORA_NODE_CONFIG",
                serde_yaml::to_string(&node_config.clone())
                    .wrap_err("failed to serialize node config")?,
            );
            // Injecting the env variable defined in the `yaml` into
            // the node runtime.
            if let Some(envs) = node.env {
                for (key, value) in envs {
                    command.env(key, value.to_string());
                }
            }
            if let Some(envs) = n.envs {
                // node has some inner env variables -> add them too
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
                        n.args.as_deref().unwrap_or_default(),
                    )
                })?
        }
        dora_core::descriptor::CoreNodeKind::Runtime(n) => {
            let python_operators: Vec<&OperatorDefinition> = n
                .operators
                .iter()
                .filter(|x| matches!(x.config.source, OperatorSource::Python { .. }))
                .collect();

            let other_operators = n
                .operators
                .iter()
                .any(|x| !matches!(x.config.source, OperatorSource::Python { .. }));

            let mut command = if !python_operators.is_empty() && !other_operators {
                // Use python to spawn runtime if there is a python operator

                // TODO: Handle multi-operator runtime once sub-interpreter is supported
                if python_operators.len() > 2 {
                    eyre::bail!(
                        "Runtime currently only support one Python Operator.
                     This is because pyo4 sub-interpreter is not yet available.
                     See: https://github.com/PyO4/pyo3/issues/576"
                    );
                }

                let python_operator = python_operators
                    .first()
                    .context("Runtime had no operators definition.")?;

                if let OperatorSource::Python(PythonSource {
                    source: _,
                    conda_env: Some(conda_env),
                }) = &python_operator.config.source
                {
                    let conda = which::which("conda").context(
                        "failed to find `conda`, yet a `conda_env` was defined. Make sure that `conda` is available.",
                    )?;
                    let mut command = tokio::process::Command::new(conda);
                    command.args([
                        "run",
                        "-n",
                        &conda_env,
                        "python",
                        "-c",
                        format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                    ]);
                    command
                } else {
                    let python = get_python_path()
                        .context("Could not find python path when spawning runtime node")?;
                    let mut command = tokio::process::Command::new(python);
                    command.args([
                        "-c",
                        format!("import dora; dora.start_runtime() # {}", node.id).as_str(),
                    ]);
                    command
                }
            } else if python_operators.is_empty() && other_operators {
                let mut cmd = tokio::process::Command::new(
                    std::env::current_exe().wrap_err("failed to get current executable path")?,
                );
                cmd.arg("runtime");
                cmd
            } else {
                eyre::bail!("Runtime can not mix Python Operator with other type of operator.");
            };
            command.current_dir(working_dir);

            let runtime_config = RuntimeConfig {
                node: node_config.clone(),
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

    let dataflow_dir: PathBuf = working_dir.join("out").join(dataflow_id.to_string());
    if !dataflow_dir.exists() {
        std::fs::create_dir_all(&dataflow_dir).context("could not create dataflow_dir")?;
    }
    let (tx, mut rx) = mpsc::channel(10);
    let mut file = File::create(log::log_path(working_dir, &dataflow_id, &node_id))
        .await
        .expect("Failed to create log file");
    let mut child_stdout =
        tokio::io::BufReader::new(child.stdout.take().expect("failed to take stdout"));
    let pid = child.id().context(
        "Could not get the pid for the just spawned node and indicate that there is an error",
    )?;
    let running_node = RunningNode {
        pid: Some(pid),
        node_config,
    };
    let stdout_tx = tx.clone();

    // Stdout listener stream
    tokio::spawn(async move {
        let mut buffer = String::new();
        let mut finished = false;
        while !finished {
            let mut raw = Vec::new();
            finished = match child_stdout
                .read_until(b'\n', &mut raw)
                .await
                .wrap_err_with(|| format!("failed to read stdout line from spawned node {node_id}"))
            {
                Ok(0) => true,
                Ok(_) => false,
                Err(err) => {
                    tracing::warn!("{err:?}");
                    false
                }
            };

            match String::from_utf8(raw) {
                Ok(s) => buffer.push_str(&s),
                Err(err) => {
                    let lossy = String::from_utf8_lossy(err.as_bytes());
                    tracing::warn!(
                        "stdout not valid UTF-8 string (node {node_id}): {}: {lossy}",
                        err.utf8_error()
                    );
                    buffer.push_str(&lossy)
                }
            };

            if buffer.contains("TRACE")
                || buffer.contains("INFO")
                || buffer.contains("DEBUG")
                || buffer.contains("WARN")
                || buffer.contains("ERROR")
            {
                // tracing output, potentially multi-line -> keep reading following lines
                // until double-newline
                if !buffer.ends_with("\n\n") && !finished {
                    continue;
                }
            }

            // send the buffered lines
            let lines = std::mem::take(&mut buffer);
            let sent = stdout_tx.send(lines.clone()).await;
            if sent.is_err() {
                println!("Could not log: {lines}");
            }
        }
    });

    let mut child_stderr =
        tokio::io::BufReader::new(child.stderr.take().expect("failed to take stderr"));

    // Stderr listener stream
    let stderr_tx = tx.clone();
    let node_id = node.id.clone();
    let uhlc = clock.clone();
    let daemon_tx_log = daemon_tx.clone();
    tokio::spawn(async move {
        let mut buffer = String::new();
        let mut finished = false;
        while !finished {
            let mut raw = Vec::new();
            finished = match child_stderr
                .read_until(b'\n', &mut raw)
                .await
                .wrap_err_with(|| format!("failed to read stderr line from spawned node {node_id}"))
            {
                Ok(0) => true,
                Ok(_) => false,
                Err(err) => {
                    tracing::warn!("{err:?}");
                    true
                }
            };

            let new = match String::from_utf8(raw) {
                Ok(s) => s,
                Err(err) => {
                    let lossy = String::from_utf8_lossy(err.as_bytes());
                    tracing::warn!(
                        "stderr not valid UTF-8 string (node {node_id}): {}: {lossy}",
                        err.utf8_error()
                    );
                    lossy.into_owned()
                }
            };

            buffer.push_str(&new);

            node_stderr_most_recent.force_push(new);

            if buffer.starts_with("Traceback (most recent call last):") {
                if !finished {
                    continue;
                } else {
                    tracing::error!("{dataflow_id}/{}: \n{buffer}", node_id);
                }
            }

            // send the buffered lines
            let lines = std::mem::take(&mut buffer);
            let sent = stderr_tx.send(lines.clone()).await;
            if sent.is_err() {
                println!("Could not log: {lines}");
            }
        }
    });

    let node_id = node.id.clone();
    let (log_finish_tx, log_finish_rx) = oneshot::channel();
    tokio::spawn(async move {
        let exit_status = NodeExitStatus::from(child.wait().await);
        let _ = log_finish_rx.await;
        let event = DoraEvent::SpawnedNodeResult {
            dataflow_id,
            node_id,
            exit_status,
        }
        .into();
        let event = Timestamped {
            inner: event,
            timestamp: clock.new_timestamp(),
        };
        let _ = daemon_tx.send(event).await;
    });

    let node_id = node.id.clone();
    // Log to file stream.
    tokio::spawn(async move {
        while let Some(message) = rx.recv().await {
            // If log is an output, we're sending the logs to the dataflow
            if let Some(stdout_output_name) = &send_stdout_to {
                // Convert logs to DataMessage
                let array = message.into_arrow();

                let array: ArrayData = array.into();
                let total_len = required_data_size(&array);
                let mut sample: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, total_len);

                let type_info = copy_array_into_sample(&mut sample, &array);

                let metadata = Metadata::new(uhlc.new_timestamp(), type_info);
                let output_id = OutputId(
                    node_id.clone(),
                    DataId::from(stdout_output_name.to_string()),
                );
                let event = DoraEvent::Logs {
                    dataflow_id,
                    output_id,
                    metadata,
                    message: DataMessage::Vec(sample),
                }
                .into();
                let event = Timestamped {
                    inner: event,
                    timestamp: uhlc.new_timestamp(),
                };
                let _ = daemon_tx_log.send(event).await;
            }

            let _ = file
                .write_all(message.as_bytes())
                .await
                .map_err(|err| error!("Could not log {message} to file due to {err}"));
            let formatted = message.lines().fold(String::default(), |mut output, line| {
                output.push_str("      ");
                output.push_str(line);
                output.push('\n');
                output
            });
            tracing::trace!("{dataflow_id}/{} logged:\n{formatted}", node.id.clone());
            // Make sure that all data has been synced to disk.
            let _ = file
                .sync_all()
                .await
                .map_err(|err| error!("Could not sync logs to file due to {err}"));
        }
        let _ = log_finish_tx
            .send(())
            .map_err(|_| error!("Could not inform that log file thread finished"));
    });
    Ok(running_node)
}
