use crate::{
    CoreNodeKindExt, DoraEvent, Event, OutputId, RunningNode,
    log::{self, NodeLogger},
};
use aligned_vec::{AVec, ConstAlign};
use crossbeam::queue::ArrayQueue;
use dora_arrow_convert::IntoArrow;
use dora_core::{
    config::DataId,
    descriptor::{ResolvedNode, ResolvedNodeExt},
    uhlc::HLC,
};
use dora_message::{
    DataflowId,
    common::{LogLevel, LogMessage, LogMessageHelper},
    daemon_to_coordinator::{DataMessage, NodeExitStatus, Timestamped},
    daemon_to_node::NodeConfig,
    descriptor::RestartPolicy,
    id::NodeId,
};
use dora_node_api::{
    Metadata,
    arrow::array::ArrayData,
    arrow_utils::{copy_array_into_sample, required_data_size},
};
use eyre::{ContextCompat, WrapErr};
use process_wrap::tokio::TokioCommandWrap;
use std::{
    path::{Path, PathBuf},
    sync::Arc,
};
use tokio::{
    fs::File,
    io::{AsyncBufReadExt, AsyncWriteExt},
    sync::{mpsc, oneshot},
};
use tracing::error;

#[derive(Clone)]
pub struct PreparedNode {
    pub(super) command: Option<clonable_command::Command>,
    pub(super) spawn_error_msg: String,
    pub(super) node_working_dir: PathBuf,
    pub(super) dataflow_id: DataflowId,
    pub(super) node: ResolvedNode,
    pub(super) node_config: NodeConfig,
    pub(super) clock: Arc<HLC>,
    pub(super) daemon_tx: mpsc::Sender<Timestamped<Event>>,
    pub(super) node_stderr_most_recent: Arc<ArrayQueue<String>>,
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
            Some(command) => {
                let std_command = command.to_std();
                logger
                    .log(
                        LogLevel::Info,
                        Some("spawner".into()),
                        format!(
                            "spawning `{}` in `{}`",
                            std_command.get_program().to_string_lossy(),
                            std_command
                                .get_current_dir()
                                .unwrap_or(Path::new("<unknown>"))
                                .display(),
                        ),
                    )
                    .await;
                let mut command =
                    TokioCommandWrap::from(tokio::process::Command::from(std_command));

                #[cfg(unix)]
                {
                    // Set the process group to 0 to ensure that the spawned process does not exit immediately on CTRL-C
                    // command.process_group(0);

                    command.wrap(process_wrap::tokio::ProcessGroup::leader());
                }
                #[cfg(windows)]
                {
                    command
                        .wrap(process_wrap::tokio::CreationFlags(
                            windows::Win32::System::Threading::CREATE_NEW_PROCESS_GROUP,
                        ))
                        .wrap(process_wrap::tokio::JobObject);
                }

                command.spawn().wrap_err(self.spawn_error_msg)?
            }
            None => {
                return Ok(RunningNode {
                    process: None,
                    node_config: self.node_config,
                });
            }
        };

        let (op_tx, op_rx) = flume::bounded(2);
        let proc_handle = crate::ProcessHandle::new(op_tx);
        let pid = child.id().context(
            "Could not get the pid for the just spawned node and indicate that there is an error",
        )?;
        logger
            .log(
                LogLevel::Debug,
                Some("spawner".into()),
                format!("spawned node with pid {pid}"),
            )
            .await;

        let dataflow_dir: PathBuf = self
            .node_working_dir
            .join("out")
            .join(self.dataflow_id.to_string());
        if !dataflow_dir.exists() {
            std::fs::create_dir_all(&dataflow_dir).context("could not create dataflow_dir")?;
        }
        let (tx, mut rx) = mpsc::channel(10);
        let mut file = File::create(log::log_path(
            &self.node_working_dir,
            &self.dataflow_id,
            &self.node.id,
        ))
        .await
        .expect("Failed to create log file");
        let mut child_stdout =
            tokio::io::BufReader::new(child.stdout().take().expect("failed to take stdout"));
        let running_node = RunningNode {
            process: Some(proc_handle),
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

                // send the buffered lines
                let lines = std::mem::take(&mut buffer);
                let sent = stdout_tx.send(lines.clone()).await;
                if sent.is_err() {
                    println!("Could not log: {lines}");
                }
            }
        });

        let mut child_stderr =
            tokio::io::BufReader::new(child.stderr().take().expect("failed to take stderr"));

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
        let uhlc = self.clock.clone();
        let daemon_tx = self.daemon_tx.clone();
        let dataflow_id = self.dataflow_id;
        let restart_policy = match &self.node.kind {
            dora_core::descriptor::CoreNodeKind::Custom(n) => n.restart_policy,
            dora_core::descriptor::CoreNodeKind::Runtime(n) => RestartPolicy::Never,
        };
        tokio::spawn(async move {
            let exit_status: NodeExitStatus = loop {
                tokio::select! {
                    status = Box::into_pin(child.wait()) => {
                        break status.into();
                    }
                    result = op_rx.recv_async() => {
                        match result {
                            Ok(op) => op.execute(child.as_mut()),
                            Err(_) => {
                                // Sender dropped
                                break Box::into_pin(child.wait()).await.into();
                            }
                        }
                    }
                }
            };

            let restart = match restart_policy {
                RestartPolicy::Always => true,
                RestartPolicy::OnFailure if exit_status.is_success() => false,
                RestartPolicy::OnFailure => true,
                RestartPolicy::Never => false,
            };

            if restart {
                // reuse op_rx
                let op_rx = op_rx;
                todo!();
            } else {
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
                    timestamp: uhlc.new_timestamp(),
                };
                let _ = daemon_tx.send(event).await;
            }
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
        let uhlc = self.clock.clone();
        // Log to file stream.
        tokio::spawn(async move {
            while let Some(message) = rx.recv().await {
                // If log is an output, we're sending the logs to the dataflow
                if let Some(stdout_output_name) = &send_stdout_to {
                    // Convert logs to DataMessage
                    let array = message.as_str().into_arrow();

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
                    match serde_json::de::from_str::<LogMessageHelper>(&formatted) {
                        Ok(log_msg) => {
                            cloned_logger.log(LogMessage::from(log_msg)).await;
                        }
                        Err(_err) => {
                            cloned_logger
                                .log(LogMessage {
                                    daemon_id: Some(daemon_id.clone()),
                                    dataflow_id: Some(dataflow_id),
                                    build_id: None,
                                    level: dora_core::build::LogLevelOrStdout::Stdout,
                                    node_id: Some(node_id.clone()),
                                    target: None,
                                    message: formatted,
                                    file: None,
                                    line: None,
                                    module_path: None,
                                    timestamp: uhlc
                                        .new_timestamp()
                                        .get_time()
                                        .to_system_time()
                                        .into(),
                                    fields: None,
                                })
                                .await;
                        }
                    }
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
