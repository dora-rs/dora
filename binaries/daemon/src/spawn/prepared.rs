use crate::{
    CoreNodeKindExt, DoraEvent, Event, OutputId, ProcessOperation, RunningNode,
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
    sync::{
        Arc,
        atomic::{self, AtomicBool, AtomicU32},
    },
};
use tokio::{
    fs::File,
    io::{AsyncBufReadExt, AsyncWriteExt},
    sync::{mpsc, oneshot},
};

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
    /// Abort handle for the node's listener task. Cloned into `RunningNode` so
    /// the dataflow can cancel the listener when it finishes.
    /// `AbortHandle` is `Clone`, so `#[derive(Clone)]` continues to work.
    pub(super) listener_abort_handle: Option<tokio::task::AbortHandle>,
}

impl PreparedNode {
    pub fn node_id(&self) -> &NodeId {
        &self.node.id
    }

    pub fn dynamic(&self) -> bool {
        self.node.kind.dynamic()
    }

    pub async fn spawn(self, mut logger: NodeLogger<'static>) -> eyre::Result<RunningNode> {
        let (op_tx, op_rx) = flume::bounded(2);
        let (finished_tx, finished_rx) = oneshot::channel();
        let kind = self
            .clone()
            .spawn_inner(&mut logger, op_rx, finished_tx)
            .await?;

        let disable_restart = Arc::new(AtomicBool::new(false));
        let pid = Arc::new(AtomicU32::new(0));
        let running_node = RunningNode {
            process: match &kind {
                NodeKind::Dynamic => None,
                NodeKind::Spawned { .. } => Some(crate::ProcessHandle::new(op_tx)),
            },
            node_config: self.node_config.clone(),
            restart_policy: self.restart_policy(),
            disable_restart: disable_restart.clone(),
            pid: match kind {
                NodeKind::Dynamic => None,
                NodeKind::Spawned { pid: new_pid } => {
                    pid.store(new_pid, atomic::Ordering::Release);
                    Some(pid.clone())
                }
            },
            listener_abort_handle: self.listener_abort_handle.clone(),
            start_time: None,
        };

        tokio::spawn(self.restart_loop(logger, finished_rx, disable_restart, pid));

        Ok(running_node)
    }

    fn restart_policy(&self) -> RestartPolicy {
        match &self.node.kind {
            dora_core::descriptor::CoreNodeKind::Custom(n) => n.restart_policy,
            dora_core::descriptor::CoreNodeKind::Runtime(_) => RestartPolicy::Never,
        }
    }

    async fn restart_loop(
        self,
        mut logger: NodeLogger<'static>,
        mut finished_rx: oneshot::Receiver<NodeProcessFinished>,
        disable_restart: Arc<AtomicBool>,
        pid: Arc<AtomicU32>,
    ) {
        loop {
            let Ok(NodeProcessFinished { exit_status, op_rx }) = finished_rx.await else {
                logger
                    .log(
                        LogLevel::Error,
                        Some("daemon".into()),
                        "failed to receive finished signal".to_string(),
                    )
                    .await;
                break;
            };

            let restart = match self.restart_policy() {
                RestartPolicy::Always => true,
                RestartPolicy::OnFailure if exit_status.is_success() => false,
                RestartPolicy::OnFailure => true,
                RestartPolicy::Never => false,
            };

            let restart_disabled = disable_restart.load(atomic::Ordering::Acquire);
            if restart && restart_disabled {
                logger
                    .log(
                        LogLevel::Info,
                        Some("daemon".into()),
                        "not restarting node because all inputs are already closed".to_string(),
                    )
                    .await;
            }
            let restart = restart && !restart_disabled;
            let success = exit_status.is_success();

            if !success {
                let _span = tracing::error_span!(
                    "node_failure",
                    node_id = %self.node.id,
                    dataflow_id = %self.dataflow_id
                )
                .entered();
                tracing::error!("node exited with error: {:?}", exit_status);
            }

            let event = DoraEvent::SpawnedNodeResult {
                dataflow_id: self.dataflow_id,
                node_id: self.node.id.clone(),
                exit_status,
                dynamic_node: self.node.kind.dynamic(),
                restart,
            }
            .into();
            let event = Timestamped {
                inner: event,
                timestamp: self.clock.clone().new_timestamp(),
            };
            let _ = self.daemon_tx.clone().send(event).await;

            if restart {
                if success {
                    logger
                        .log(
                            LogLevel::Info,
                            Some("daemon".into()),
                            "restarting node after successful exit".to_string(),
                        )
                        .await;
                } else {
                    logger
                        .log(
                            LogLevel::Warn,
                            Some("daemon".into()),
                            "restarting node after failure".to_string(),
                        )
                        .await;
                }
                let (finished_tx, finished_rx_new) = oneshot::channel();
                let result = self
                    .clone()
                    .spawn_inner(&mut logger, op_rx, finished_tx)
                    .await;
                match result {
                    Ok(NodeKind::Spawned { pid: new_pid }) => {
                        finished_rx = finished_rx_new;
                        pid.store(new_pid, atomic::Ordering::Release);
                    }
                    Ok(NodeKind::Dynamic) => {
                        logger
                            .log(
                                LogLevel::Error,
                                Some("daemon".into()),
                                "cannot restart dynamic node".to_string(),
                            )
                            .await;
                        break;
                    }
                    Err(err) => {
                        logger
                            .log(
                                LogLevel::Error,
                                Some("daemon".into()),
                                format!("failed to restart node: {err:?}"),
                            )
                            .await;
                        break;
                    }
                }
            } else {
                break;
            }
        }
    }

    async fn spawn_inner(
        mut self,
        logger: &mut NodeLogger<'_>,
        op_rx: flume::Receiver<ProcessOperation>,
        finished_tx: oneshot::Sender<NodeProcessFinished>,
    ) -> eyre::Result<NodeKind> {
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
                return Ok(NodeKind::Dynamic);
            }
        };

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
        let stdout_tx = tx.clone();
        let node_id = self.node.id.clone();
        let mut logger_c = logger.try_clone().await?;
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
                        logger_c
                            .log(LogLevel::Warn, Some("daemon".into()), format!("{err:?}"))
                            .await;
                        false
                    }
                };

                match String::from_utf8(raw) {
                    Ok(s) => buffer.push_str(&s),
                    Err(err) => {
                        let lossy = String::from_utf8_lossy(err.as_bytes());
                        logger_c
                            .log(
                                LogLevel::Warn,
                                Some("daemon".into()),
                                format!(
                                    "stdout not valid UTF-8 string ({}: {lossy}",
                                    err.utf8_error()
                                ),
                            )
                            .await;
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

        let (log_finish_tx, log_finish_rx) = oneshot::channel();
        let dataflow_id = self.dataflow_id;

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

            let _ = log_finish_rx.await;
            let _ = finished_tx.send(NodeProcessFinished { exit_status, op_rx });
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
        let mut logger_c = logger.try_clone().await?;
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

                match file.write_all(message.as_bytes()).await {
                    Ok(_) => {}
                    Err(err) => {
                        logger_c
                            .log(
                                LogLevel::Error,
                                Some("daemon".into()),
                                format!("Could not log {message} to file due to {err}"),
                            )
                            .await;
                    }
                }

                let formatted = message.lines().fold(String::default(), |mut output, line| {
                    output.push_str(line);
                    output
                });

                if std::env::var("DORA_QUIET").is_err() {
                    match serde_json::de::from_str::<LogMessageHelper>(&formatted) {
                        Ok(log_msg) => {
                            let mut message = LogMessage::from(log_msg);
                            message.dataflow_id = Some(dataflow_id);
                            message.node_id = Some(node_id.clone());
                            message.daemon_id = Some(daemon_id.clone());
                            cloned_logger.log(message).await;
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
                let _ = file.sync_all().await.map_err(|err| {
                    logger_c.log(
                        LogLevel::Error,
                        Some("daemon".into()),
                        format!("Could not sync logs to file due to {err}"),
                    )
                });
            }
            let _ = log_finish_tx.send(()).map_err(|_| {
                logger_c.log(
                    LogLevel::Error,
                    Some("daemon".into()),
                    "Could not inform that log file thread finished".to_string(),
                )
            });
        });
        Ok(NodeKind::Spawned { pid })
    }
}

#[must_use]
enum NodeKind {
    Dynamic,
    Spawned { pid: u32 },
}

struct NodeProcessFinished {
    exit_status: NodeExitStatus,
    op_rx: flume::Receiver<ProcessOperation>,
}
