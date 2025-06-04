use crate::{
    log::{self, NodeLogger},
    node_communication::spawn_listener_loop,
    node_inputs, CoreNodeKindExt, DoraEvent, Event, OutputId, RunningNode,
};
use aligned_vec::{AVec, ConstAlign};
use crossbeam::queue::ArrayQueue;
use dora_arrow_convert::IntoArrow;
use dora_core::{
    build::run_build_command,
    config::DataId,
    descriptor::{
        resolve_path, source_is_url, CustomNode, Descriptor, OperatorDefinition, OperatorSource,
        PythonSource, ResolvedNode, ResolvedNodeExt, DYNAMIC_SOURCE, SHELL_SOURCE,
    },
    get_python_path,
    uhlc::HLC,
};
use dora_download::download_file;
use dora_message::{
    common::{LogLevel, LogMessage},
    daemon_to_coordinator::{DataMessage, NodeExitStatus, Timestamped},
    daemon_to_node::{NodeConfig, RuntimeConfig},
    descriptor::EnvValue,
    id::NodeId,
    DataflowId,
};
use dora_node_api::{
    arrow::array::ArrayData,
    arrow_utils::{copy_array_into_sample, required_data_size},
    Metadata,
};
use eyre::{ContextCompat, WrapErr};
use git::GitFolder;
use std::{
    collections::{BTreeMap, BTreeSet},
    future::Future,
    path::{Path, PathBuf},
    process::Stdio,
    sync::Arc,
};
use tokio::{
    fs::File,
    io::{AsyncBufReadExt, AsyncWriteExt},
    sync::{mpsc, oneshot},
};
use tracing::error;

#[derive(Clone)]
pub struct Spawner {
    pub dataflow_id: DataflowId,
    pub working_dir: PathBuf,
    pub daemon_tx: mpsc::Sender<Timestamped<Event>>,
    pub dataflow_descriptor: Descriptor,
    /// clock is required for generating timestamps when dropping messages early because queue is full
    pub clock: Arc<HLC>,
    pub uv: bool,
}

pub struct PreparedNode {
    command: Option<tokio::process::Command>,
    spawn_error_msg: String,
    working_dir: PathBuf,
    dataflow_id: DataflowId,
    node: ResolvedNode,
    node_config: NodeConfig,
    clock: Arc<HLC>,
    daemon_tx: mpsc::Sender<Timestamped<Event>>,
    node_stderr_most_recent: Arc<ArrayQueue<String>>,
}

impl PreparedNode {
    pub fn node_id(&self) -> &NodeId {
        &self.node.id
    }

    pub fn dynamic(&self) -> bool {
        self.node.kind.dynamic()
    }

    pub async fn spawn(mut self, logger: &mut NodeLogger<'_>) -> eyre::Result<RunningNode> {
        let mut child = match &mut self.command {
            Some(command) => command.spawn().wrap_err(self.spawn_error_msg)?,
            None => {
                return Ok(RunningNode {
                    pid: None,
                    node_config: self.node_config,
                })
            }
        };

        let pid = crate::ProcessId::new(child.id().context(
            "Could not get the pid for the just spawned node and indicate that there is an error",
        )?);
        logger
            .log(
                LogLevel::Debug,
                Some("spawner".into()),
                format!("spawned node with pid {pid:?}"),
            )
            .await;

        let dataflow_dir: PathBuf = self
            .working_dir
            .join("out")
            .join(self.dataflow_id.to_string());
        if !dataflow_dir.exists() {
            std::fs::create_dir_all(&dataflow_dir).context("could not create dataflow_dir")?;
        }
        let (tx, mut rx) = mpsc::channel(10);
        let mut file = File::create(log::log_path(
            &self.working_dir,
            &self.dataflow_id,
            &self.node.id,
        ))
        .await
        .expect("Failed to create log file");
        let mut child_stdout =
            tokio::io::BufReader::new(child.stdout.take().expect("failed to take stdout"));
        let running_node = RunningNode {
            pid: Some(pid),
            node_config: self.node_config,
        };
        let stdout_tx = tx.clone();
        let node_id = self.node.id.clone();
        // Stdout listener stream
        tokio::spawn(async move {
            let mut buffer = String::new();
            let mut finished = false;
            while !finished {
                let mut raw = Vec::new();
                finished = match child_stdout
                    .read_until(b'\n', &mut raw)
                    .await
                    .wrap_err_with(|| {
                        format!("failed to read stdout line from spawned node {node_id}")
                    }) {
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
        let node_id = self.node.id.clone();
        let uhlc = self.clock.clone();
        let daemon_tx_log = self.daemon_tx.clone();
        tokio::spawn(async move {
            let mut buffer = String::new();
            let mut finished = false;
            while !finished {
                let mut raw = Vec::new();
                finished = match child_stderr
                    .read_until(b'\n', &mut raw)
                    .await
                    .wrap_err_with(|| {
                        format!("failed to read stderr line from spawned node {node_id}")
                    }) {
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

                self.node_stderr_most_recent.force_push(new);

                // send the buffered lines
                let lines = std::mem::take(&mut buffer);
                let sent = stderr_tx.send(lines.clone()).await;
                if sent.is_err() {
                    println!("Could not log: {lines}");
                }
            }
        });

        let node_id = self.node.id.clone();
        let dynamic_node = self.node.kind.dynamic();
        let (log_finish_tx, log_finish_rx) = oneshot::channel();
        let clock = self.clock.clone();
        let daemon_tx = self.daemon_tx.clone();
        let dataflow_id = self.dataflow_id;
        tokio::spawn(async move {
            let exit_status = NodeExitStatus::from(child.wait().await);
            let _ = log_finish_rx.await;
            let event = DoraEvent::SpawnedNodeResult {
                dataflow_id,
                node_id,
                exit_status,
                dynamic_node,
            }
            .into();
            let event = Timestamped {
                inner: event,
                timestamp: clock.new_timestamp(),
            };
            let _ = daemon_tx.send(event).await;
        });

        let node_id = self.node.id.clone();
        let daemon_id = logger.inner().inner().daemon_id().clone();
        let mut cloned_logger = logger
            .inner()
            .inner()
            .inner()
            .try_clone()
            .await
            .context("failed to clone logger")?;

        let send_stdout_to = self
            .node
            .send_stdout_as()
            .context("Could not resolve `send_stdout_as` configuration")?;

        // Log to file stream.
        tokio::spawn(async move {
            while let Some(message) = rx.recv().await {
                // If log is an output, we're sending the logs to the dataflow
                if let Some(stdout_output_name) = &send_stdout_to {
                    // Convert logs to DataMessage
                    let array = message.into_arrow();

                    let array: ArrayData = array.into();
                    let total_len = required_data_size(&array);
                    let mut sample: AVec<u8, ConstAlign<128>> =
                        AVec::__from_elem(128, 0, total_len);

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
                    output.push_str(line);
                    output
                });
                if std::env::var("DORA_QUIET").is_err() {
                    cloned_logger
                        .log(LogMessage {
                            daemon_id: Some(daemon_id.clone()),
                            dataflow_id: Some(dataflow_id),
                            build_id: None,
                            level: LogLevel::Info,
                            node_id: Some(node_id.clone()),
                            target: Some("stdout".into()),
                            message: formatted,
                            file: None,
                            line: None,
                            module_path: None,
                        })
                        .await;
                }
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
}

async fn path_spawn_command(
    working_dir: &Path,
    uv: bool,
    logger: &mut NodeLogger<'_>,
    node: &dora_core::descriptor::CustomNode,
    permit_url: bool,
) -> eyre::Result<Option<tokio::process::Command>> {
    let cmd = match node.path.as_str() {
        DYNAMIC_SOURCE => return Ok(None),
        SHELL_SOURCE => {
            if cfg!(target_os = "windows") {
                let mut cmd = tokio::process::Command::new("cmd");
                cmd.args(["/C", &node.args.clone().unwrap_or_default()]);
                cmd
            } else {
                let mut cmd = tokio::process::Command::new("sh");
                cmd.args(["-c", &node.args.clone().unwrap_or_default()]);
                cmd
            }
        }
        source => {
            let resolved_path = if source_is_url(source) {
                if !permit_url {
                    eyre::bail!("URL paths are not supported in this case");
                }
                // try to download the shared library
                let target_dir = Path::new("build");
                download_file(source, target_dir)
                    .await
                    .wrap_err("failed to download custom node")?
            } else {
                resolve_path(source, working_dir)
                    .wrap_err_with(|| format!("failed to resolve node source `{}`", source))?
            };

            // If extension is .py, use python to run the script
            let mut cmd = match resolved_path.extension().map(|ext| ext.to_str()) {
                Some(Some("py")) => {
                    let mut cmd = if uv {
                        let mut cmd = tokio::process::Command::new("uv");
                        cmd.arg("run");
                        cmd.arg("python");
                        logger
                            .log(
                                LogLevel::Info,
                                Some("spawner".into()),
                                format!("spawning: uv run python -u {}", resolved_path.display()),
                            )
                            .await;
                        cmd
                    } else {
                        let python = get_python_path()
                            .wrap_err("Could not find python path when spawning custom node")?;
                        logger
                            .log(
                                LogLevel::Info,
                                Some("spawner".into()),
                                format!("spawning: {:?} -u {}", &python, resolved_path.display()),
                            )
                            .await;

                        tokio::process::Command::new(python)
                    };
                    // Force python to always flush stdout/stderr buffer
                    cmd.arg("-u");
                    cmd.arg(&resolved_path);
                    cmd
                }
                _ => {
                    logger
                        .log(
                            LogLevel::Info,
                            Some("spawner".into()),
                            format!("spawning: {}", resolved_path.display()),
                        )
                        .await;
                    if uv {
                        let mut cmd = tokio::process::Command::new("uv");
                        cmd.arg("run");
                        cmd.arg(&resolved_path);
                        cmd
                    } else {
                        tokio::process::Command::new(&resolved_path)
                    }
                }
            };

            if let Some(args) = &node.args {
                cmd.args(args.split_ascii_whitespace());
            }
            cmd
        }
    };

    Ok(Some(cmd))
}
