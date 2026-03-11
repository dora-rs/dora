use crate::{
    AdoraEvent, CoreNodeKindExt, Event, OutputId, ProcessOperation, RunningNode,
    log::{self, NodeLogger},
};
use adora_arrow_convert::IntoArrow;
use adora_core::{
    config::DataId,
    descriptor::{ResolvedNode, ResolvedNodeExt},
    uhlc::HLC,
};
use adora_message::{
    DataflowId,
    common::{LogLevel, LogMessage, LogMessageHelper},
    daemon_to_coordinator::{DataMessage, NodeExitStatus, Timestamped},
    daemon_to_node::NodeConfig,
    descriptor::RestartPolicy,
    id::NodeId,
};
use adora_node_api::{
    Metadata,
    arrow::array::ArrayData,
    arrow_utils::{copy_array_into_sample, required_data_size},
};
use aligned_vec::{AVec, ConstAlign};
use crossbeam::queue::ArrayQueue;
use eyre::{ContextCompat, WrapErr};
use process_wrap::tokio::TokioCommandWrap;
use std::{
    path::{Path, PathBuf},
    sync::{
        Arc,
        atomic::{self, AtomicBool, AtomicU32, AtomicU64},
    },
    time::Duration,
};
use tokio::{
    fs::File,
    io::{AsyncBufReadExt, AsyncWriteExt},
    sync::{mpsc, oneshot},
};

#[derive(Clone, Copy)]
enum LogStream {
    Stdout,
    Stderr,
}

struct LogLine {
    content: String,
    stream: LogStream,
}

#[derive(Clone, Default)]
struct RestartConfig {
    max_restarts: u32,
    restart_delay: Option<Duration>,
    max_restart_delay: Option<Duration>,
    restart_window: Option<Duration>,
}

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
    pub(super) last_activity: Arc<AtomicU64>,
    pub(super) ft_stats: Arc<crate::FaultToleranceStats>,
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
        let restart_count = Arc::new(AtomicU32::new(0));
        let running_node = RunningNode {
            process: match &kind {
                NodeKind::Dynamic => None,
                NodeKind::Spawned { .. } => Some(crate::ProcessHandle::new(op_tx)),
            },
            node_config: self.node_config.clone(),
            restart_policy: self.restart_policy(),
            disable_restart: disable_restart.clone(),
            restart_count: restart_count.clone(),
            pid: match kind {
                NodeKind::Dynamic => None,
                NodeKind::Spawned { pid: new_pid } => {
                    pid.store(new_pid, atomic::Ordering::Release);
                    Some(pid.clone())
                }
            },
            last_activity: self.last_activity.clone(),
            health_check_timeout: self.health_check_timeout(),
        };

        tokio::spawn(self.restart_loop(logger, finished_rx, disable_restart, pid, restart_count));

        Ok(running_node)
    }

    fn restart_policy(&self) -> RestartPolicy {
        match &self.node.kind {
            adora_core::descriptor::CoreNodeKind::Custom(n) => n.restart_policy,
            adora_core::descriptor::CoreNodeKind::Runtime(_) => RestartPolicy::Never,
        }
    }

    fn health_check_timeout(&self) -> Option<Duration> {
        match &self.node.kind {
            adora_core::descriptor::CoreNodeKind::Custom(n) => {
                n.health_check_timeout.map(Duration::from_secs_f64)
            }
            adora_core::descriptor::CoreNodeKind::Runtime(_) => None,
        }
    }

    fn restart_config(&self) -> RestartConfig {
        match &self.node.kind {
            adora_core::descriptor::CoreNodeKind::Custom(n) => RestartConfig {
                max_restarts: n.max_restarts,
                restart_delay: n.restart_delay.map(Duration::from_secs_f64),
                max_restart_delay: n.max_restart_delay.map(Duration::from_secs_f64),
                restart_window: n.restart_window.map(Duration::from_secs_f64),
            },
            adora_core::descriptor::CoreNodeKind::Runtime(_) => RestartConfig::default(),
        }
    }

    async fn restart_loop(
        mut self,
        mut logger: NodeLogger<'static>,
        mut finished_rx: oneshot::Receiver<NodeProcessFinished>,
        disable_restart: Arc<AtomicBool>,
        pid: Arc<AtomicU32>,
        shared_restart_count: Arc<AtomicU32>,
    ) {
        let config = self.restart_config();
        let mut restart_count: u32 = 0;
        let mut window_start = tokio::time::Instant::now();
        let mut window_count: u32 = 0;

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

            // Check restart limits before committing to restart
            let restart = if restart {
                // Reset window if expired
                if let Some(window) = config.restart_window {
                    if window_start.elapsed() > window {
                        window_count = 0;
                        window_start = tokio::time::Instant::now();
                    }
                }
                window_count += 1;

                // Check max restarts
                if config.max_restarts > 0 && window_count > config.max_restarts {
                    logger
                        .log(
                            LogLevel::Error,
                            Some("daemon".into()),
                            format!("max restarts ({}) exceeded, giving up", config.max_restarts,),
                        )
                        .await;
                    false
                } else {
                    true
                }
            } else {
                false
            };

            let event = AdoraEvent::SpawnedNodeResult {
                dataflow_id: self.dataflow_id,
                node_id: self.node.id.clone(),
                exit_status,
                dynamic_node: self.node.kind.dynamic(),
                restart,
                restart_count,
            }
            .into();
            let event = Timestamped {
                inner: event,
                timestamp: self.clock.clone().new_timestamp(),
            };
            let _ = self.daemon_tx.clone().send(event).await;

            if restart {
                // Exponential backoff
                if let Some(base_delay) = config.restart_delay {
                    let exp = (window_count - 1).min(16); // cap exponent to avoid overflow
                    let backoff = base_delay.mul_f64(2f64.powi(exp as i32));
                    let backoff = config
                        .max_restart_delay
                        .map_or(backoff, |max| backoff.min(max));
                    logger
                        .log(
                            LogLevel::Info,
                            Some("daemon".into()),
                            format!(
                                "waiting {backoff:?} before restart (attempt {})",
                                restart_count + 1,
                            ),
                        )
                        .await;
                    tokio::time::sleep(backoff).await;

                    // Re-check disable_restart after sleep (may have changed)
                    if disable_restart.load(atomic::Ordering::Acquire) {
                        logger
                            .log(
                                LogLevel::Info,
                                Some("daemon".into()),
                                "restart cancelled: inputs closed during backoff wait".to_string(),
                            )
                            .await;
                        break;
                    }
                }

                restart_count += 1;
                self.node_config.restart_count = restart_count;
                shared_restart_count.store(restart_count, atomic::Ordering::Release);
                self.ft_stats
                    .restarts
                    .fetch_add(1, atomic::Ordering::Relaxed);

                if success {
                    logger
                        .log(
                            LogLevel::Info,
                            Some("daemon".into()),
                            format!(
                                "restarting node after successful exit (attempt {restart_count})",
                            ),
                        )
                        .await;
                } else {
                    logger
                        .log(
                            LogLevel::Warn,
                            Some("daemon".into()),
                            format!("restarting node after failure (attempt {restart_count})"),
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
        let (tx, mut rx) = mpsc::channel::<LogLine>(10);
        let mut file = File::create(log::log_path(
            &self.node_working_dir,
            &self.dataflow_id,
            &self.node.id,
        ))
        .await
        .context("failed to create log file")?;
        let mut child_stdout = tokio::io::BufReader::new(
            child
                .stdout()
                .take()
                .ok_or_else(|| eyre::eyre!("failed to take stdout"))?,
        );
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
                let content = std::mem::take(&mut buffer);
                let sent = stdout_tx
                    .send(LogLine {
                        content: content.clone(),
                        stream: LogStream::Stdout,
                    })
                    .await;
                if sent.is_err() {
                    tracing::warn!("Could not log: {content}");
                }
            }
        });

        let mut child_stderr = tokio::io::BufReader::new(
            child
                .stderr()
                .take()
                .ok_or_else(|| eyre::eyre!("failed to take stderr"))?,
        );

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
                let content = std::mem::take(&mut buffer);
                let sent = stderr_tx
                    .send(LogLine {
                        content: content.clone(),
                        stream: LogStream::Stderr,
                    })
                    .await;
                if sent.is_err() {
                    tracing::warn!("Could not log: {content}");
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
        let send_logs_to = self
            .node
            .send_logs_as()
            .context("Could not resolve `send_logs_as` configuration")?;
        let min_log_level = self
            .node
            .min_log_level()
            .context("Could not resolve `min_log_level` configuration")?;
        let max_log_size = self
            .node
            .max_log_size()
            .context("Could not resolve `max_log_size` configuration")?;
        let max_rotated_files = self
            .node
            .max_rotated_files()
            .context("Could not resolve `max_rotated_files` configuration")?
            .unwrap_or(log::DEFAULT_MAX_ROTATED_FILES);
        let daemon_tx_logs_as = if send_logs_to.is_some() {
            Some(self.daemon_tx.clone())
        } else {
            None
        };
        let working_dir_c = self.node_working_dir.clone();
        let uhlc = self.clock.clone();
        let mut logger_c = logger.try_clone().await?;
        // Log to file stream.
        tokio::spawn(async move {
            let mut bytes_written: u64 = 0;
            while let Some(log_line) = rx.recv().await {
                let LogLine { content, stream } = log_line;
                let stream_str = match stream {
                    LogStream::Stdout => "stdout",
                    LogStream::Stderr => "stderr",
                };

                // If log is an output, we're sending the logs to the dataflow
                if let Some(stdout_output_name) = &send_stdout_to {
                    // Convert logs to DataMessage
                    let array = content.as_str().into_arrow();

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
                    let event = AdoraEvent::Logs {
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

                let formatted = content.lines().fold(String::default(), |mut output, line| {
                    output.push_str(line);
                    output
                });

                // Build a LogMessage for both file writing and channel forwarding
                let log_message = match serde_json::de::from_str::<LogMessageHelper>(&formatted) {
                    Ok(log_msg) => {
                        let mut message = LogMessage::from(log_msg);
                        message.dataflow_id = Some(dataflow_id);
                        message.node_id = Some(node_id.clone());
                        message.daemon_id = Some(daemon_id.clone());
                        message
                    }
                    Err(_) => LogMessage {
                        daemon_id: Some(daemon_id.clone()),
                        dataflow_id: Some(dataflow_id),
                        build_id: None,
                        level: adora_core::build::LogLevelOrStdout::Stdout,
                        node_id: Some(node_id.clone()),
                        target: None,
                        message: formatted,
                        file: None,
                        line: None,
                        module_path: None,
                        timestamp: uhlc.new_timestamp().get_time().to_system_time().into(),
                        fields: None,
                    },
                };

                // Apply min_log_level filter
                if let Some(min_level) = &min_log_level {
                    if !log_message.level.passes(min_level) {
                        continue;
                    }
                }

                // Route structured logs via send_logs_as
                if let (Some(logs_output_name), Some(daemon_tx)) =
                    (&send_logs_to, &daemon_tx_logs_as)
                {
                    if let Ok(json) = serde_json::to_string(&log_message) {
                        let array = json.as_str().into_arrow();
                        let array: ArrayData = array.into();
                        let total_len = required_data_size(&array);
                        let mut sample: AVec<u8, ConstAlign<128>> =
                            AVec::__from_elem(128, 0, total_len);
                        let type_info = copy_array_into_sample(&mut sample, &array);
                        let metadata = Metadata::new(uhlc.new_timestamp(), type_info);
                        let output_id =
                            OutputId(node_id.clone(), DataId::from(logs_output_name.to_string()));
                        let event = AdoraEvent::Logs {
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
                        let _ = daemon_tx.send(event).await;
                    }
                }

                // Write JSONL to log file
                let ts = log_message
                    .timestamp
                    .to_rfc3339_opts(chrono::SecondsFormat::Millis, true);
                let log_entry = serde_json::json!({
                    "ts": ts,
                    "level": match &log_message.level {
                        adora_core::build::LogLevelOrStdout::LogLevel(l) => match l {
                            LogLevel::Error => "error",
                            LogLevel::Warn => "warn",
                            LogLevel::Info => "info",
                            LogLevel::Debug => "debug",
                            LogLevel::Trace => "trace",
                        },
                        adora_core::build::LogLevelOrStdout::Stdout => "stdout",
                    },
                    "node": node_id.to_string(),
                    "stream": stream_str,
                    "msg": &log_message.message,
                    "target": &log_message.target,
                    "fields": &log_message.fields,
                });
                let mut json_bytes = serde_json::to_vec(&log_entry).unwrap_or_default();
                json_bytes.push(b'\n');
                let write_len = json_bytes.len() as u64;
                match file.write_all(&json_bytes).await {
                    Ok(_) => {
                        bytes_written += write_len;
                    }
                    Err(err) => {
                        logger_c
                            .log(
                                LogLevel::Error,
                                Some("daemon".into()),
                                format!("Could not write log to file: {err}"),
                            )
                            .await;
                    }
                }

                // Rotate if max_log_size exceeded
                if let Some(max_size) = max_log_size {
                    if bytes_written >= max_size {
                        // Flush and drop the current file handle
                        let _ = file.flush().await;
                        drop(file);
                        if let Err(err) = log::rotate_log_files(
                            &working_dir_c,
                            &dataflow_id,
                            &node_id,
                            max_rotated_files,
                        ) {
                            logger_c
                                .log(
                                    LogLevel::Error,
                                    Some("daemon".into()),
                                    format!("Could not rotate log files: {err}"),
                                )
                                .await;
                        }
                        // Create a fresh log file
                        file = match File::create(log::log_path(
                            &working_dir_c,
                            &dataflow_id,
                            &node_id,
                        ))
                        .await
                        {
                            Ok(f) => f,
                            Err(err) => {
                                logger_c
                                    .log(
                                        LogLevel::Error,
                                        Some("daemon".into()),
                                        format!(
                                            "Could not create new log file after rotation: {err}"
                                        ),
                                    )
                                    .await;
                                break;
                            }
                        };
                        bytes_written = 0;
                    }
                }

                // Forward to channel/coordinator for live display
                if std::env::var("ADORA_QUIET").is_err() {
                    cloned_logger.log(log_message).await;
                }

                // Sync to disk
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
