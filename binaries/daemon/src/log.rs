use std::{
    ops::{Deref, DerefMut},
    path::{Path, PathBuf},
    sync::Arc,
};

use dora_core::{
    build::{BuildLogger, LogLevelOrStdout},
    config::NodeId,
    uhlc,
};
use dora_message::{
    BuildId,
    common::{DaemonId, LogLevel, LogMessage, Timestamped},
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent},
};
use eyre::Context;
use flume::Sender;
use uuid::Uuid;

use crate::coordinator::CoordinatorSender;

pub fn log_path(working_dir: &Path, dataflow_id: &Uuid, node_id: &NodeId) -> PathBuf {
    let dataflow_dir = working_dir.join("out").join(dataflow_id.to_string());
    dataflow_dir.join(format!("log_{node_id}.jsonl"))
}

/// Path for a rotated log file: `log_<node>.1.jsonl`, `log_<node>.2.jsonl`, etc.
pub fn log_path_rotated(
    working_dir: &Path,
    dataflow_id: &Uuid,
    node_id: &NodeId,
    index: u32,
) -> PathBuf {
    let dataflow_dir = working_dir.join("out").join(dataflow_id.to_string());
    dataflow_dir.join(format!("log_{node_id}.{index}.jsonl"))
}

/// Default max rotated files (excluding the current file).
pub const DEFAULT_MAX_ROTATED_FILES: u32 = 5;

/// Rotate log files: current -> .1, .1 -> .2, ... delete oldest beyond max_files.
pub fn rotate_log_files(
    working_dir: &Path,
    dataflow_id: &Uuid,
    node_id: &NodeId,
    max_files: u32,
) -> std::io::Result<()> {
    // Delete the oldest if it exists
    let oldest = log_path_rotated(working_dir, dataflow_id, node_id, max_files);
    if oldest.exists() {
        std::fs::remove_file(&oldest)?;
    }

    // Shift .N -> .N+1 (from max_files-1 down to 1)
    for i in (1..max_files).rev() {
        let from = log_path_rotated(working_dir, dataflow_id, node_id, i);
        let to = log_path_rotated(working_dir, dataflow_id, node_id, i + 1);
        if from.exists() {
            std::fs::rename(&from, &to)?;
        }
    }

    // Rename current -> .1
    let current = log_path(working_dir, dataflow_id, node_id);
    let first = log_path_rotated(working_dir, dataflow_id, node_id, 1);
    if current.exists() {
        std::fs::rename(&current, &first)?;
    }

    Ok(())
}

pub struct NodeLogger<'a> {
    node_id: NodeId,
    logger: DataflowLogger<'a>,
}

impl NodeLogger<'_> {
    pub fn inner(&self) -> &DataflowLogger<'_> {
        &self.logger
    }

    pub async fn log(
        &mut self,
        level: LogLevel,
        target: Option<String>,
        message: impl Into<String>,
    ) {
        self.logger
            .log(level, Some(self.node_id.clone()), target, message)
            .await
    }

    pub async fn try_clone(&self) -> eyre::Result<NodeLogger<'static>> {
        Ok(NodeLogger {
            node_id: self.node_id.clone(),
            logger: self.logger.try_clone().await?,
        })
    }
}

pub struct DataflowLogger<'a> {
    dataflow_id: Uuid,
    logger: CowMut<'a, DaemonLogger>,
}

impl<'a> DataflowLogger<'a> {
    pub fn for_node(self, node_id: NodeId) -> NodeLogger<'a> {
        NodeLogger {
            node_id,
            logger: self,
        }
    }

    pub fn reborrow(&mut self) -> DataflowLogger<'_> {
        DataflowLogger {
            dataflow_id: self.dataflow_id,
            logger: CowMut::Borrowed(&mut self.logger),
        }
    }

    pub fn inner(&self) -> &DaemonLogger {
        &self.logger
    }

    pub async fn log(
        &mut self,
        level: LogLevel,
        node_id: Option<NodeId>,
        target: Option<String>,
        message: impl Into<String>,
    ) {
        self.logger
            .log(level, Some(self.dataflow_id), node_id, target, message)
            .await
    }

    pub async fn try_clone(&self) -> eyre::Result<DataflowLogger<'static>> {
        Ok(DataflowLogger {
            dataflow_id: self.dataflow_id,
            logger: CowMut::Owned(self.logger.try_clone().await?),
        })
    }
}

pub struct NodeBuildLogger<'a> {
    build_id: BuildId,
    node_id: NodeId,
    logger: CowMut<'a, DaemonLogger>,
}

impl NodeBuildLogger<'_> {
    pub async fn log(
        &mut self,
        level: impl Into<LogLevelOrStdout> + Send,
        message: impl Into<String>,
    ) {
        self.logger
            .log_build(
                self.build_id,
                level.into(),
                None,
                Some(self.node_id.clone()),
                message,
            )
            .await
    }

    pub async fn try_clone_impl(&self) -> eyre::Result<NodeBuildLogger<'static>> {
        Ok(NodeBuildLogger {
            build_id: self.build_id,
            node_id: self.node_id.clone(),
            logger: CowMut::Owned(self.logger.try_clone().await?),
        })
    }
}

impl BuildLogger for NodeBuildLogger<'_> {
    type Clone = NodeBuildLogger<'static>;

    fn log_message(
        &mut self,
        level: impl Into<LogLevelOrStdout> + Send,
        message: impl Into<String> + Send,
    ) -> impl std::future::Future<Output = ()> + Send {
        self.log(level, message)
    }

    fn try_clone(&self) -> impl std::future::Future<Output = eyre::Result<Self::Clone>> + Send {
        self.try_clone_impl()
    }
}

pub struct DaemonLogger {
    daemon_id: DaemonId,
    logger: Logger,
}

impl DaemonLogger {
    pub fn for_dataflow(&mut self, dataflow_id: Uuid) -> DataflowLogger<'_> {
        DataflowLogger {
            dataflow_id,
            logger: CowMut::Borrowed(self),
        }
    }

    pub fn for_node_build(&mut self, build_id: BuildId, node_id: NodeId) -> NodeBuildLogger<'_> {
        NodeBuildLogger {
            build_id,
            node_id,
            logger: CowMut::Borrowed(self),
        }
    }

    pub fn inner(&self) -> &Logger {
        &self.logger
    }

    pub async fn log(
        &mut self,
        level: LogLevel,
        dataflow_id: Option<Uuid>,
        node_id: Option<NodeId>,
        target: Option<String>,
        message: impl Into<String>,
    ) {
        let message = LogMessage {
            build_id: None,
            daemon_id: Some(self.daemon_id.clone()),
            dataflow_id,
            node_id,
            level: level.into(),
            target,
            module_path: None,
            file: None,
            line: None,
            message: message.into(),
            timestamp: self
                .logger
                .clock
                .new_timestamp()
                .get_time()
                .to_system_time()
                .into(),

            fields: None,
        };
        self.logger.log(message).await
    }

    pub async fn log_build(
        &mut self,
        build_id: BuildId,
        level: LogLevelOrStdout,
        target: Option<String>,
        node_id: Option<NodeId>,
        message: impl Into<String>,
    ) {
        let message = LogMessage {
            build_id: Some(build_id),
            daemon_id: Some(self.daemon_id.clone()),
            dataflow_id: None,
            node_id,
            level,
            target,
            module_path: None,
            file: None,
            line: None,
            message: message.into(),
            timestamp: self
                .logger
                .clock
                .new_timestamp()
                .get_time()
                .to_system_time()
                .into(),
            fields: None,
        };
        self.logger.log(message).await
    }

    pub(crate) fn daemon_id(&self) -> &DaemonId {
        &self.daemon_id
    }

    pub async fn try_clone(&self) -> eyre::Result<Self> {
        Ok(Self {
            daemon_id: self.daemon_id.clone(),
            logger: self.logger.try_clone().await?,
        })
    }
}

pub struct Logger {
    pub(super) destination: LogDestination,
    pub(super) daemon_id: DaemonId,
    pub(super) clock: Arc<uhlc::HLC>,
}

impl Logger {
    pub fn for_daemon(self, daemon_id: DaemonId) -> DaemonLogger {
        DaemonLogger {
            daemon_id,
            logger: self,
        }
    }

    pub async fn log(&mut self, message: LogMessage) {
        match &mut self.destination {
            LogDestination::Coordinator { sender } => {
                let message = Timestamped {
                    inner: CoordinatorRequest::Event {
                        daemon_id: self.daemon_id.clone(),
                        event: DaemonEvent::Log(message.clone()),
                    },
                    timestamp: self.clock.new_timestamp(),
                };
                Self::log_to_coordinator(message, sender).await
            }
            LogDestination::Channel { sender } => {
                let _ = sender.send_async(message).await;
            }
            LogDestination::Tracing => {
                // log message using tracing if reporting to coordinator is not possible
                match message.level {
                    LogLevelOrStdout::Stdout => {
                        tracing::info!(
                            build_id = ?message.build_id.map(|id| id.to_string()),
                            dataflow_id = ?message.dataflow_id.map(|id| id.to_string()),
                            node_id = ?message.node_id.map(|id| id.to_string()),
                            target = message.target,
                            module_path = message.module_path,
                            file = message.file,
                            line = message.line,
                            "{}",
                            Indent(&message.message)
                        )
                    }
                    LogLevelOrStdout::LogLevel(level) => match level {
                        LogLevel::Error => {
                            tracing::error!(
                                build_id = ?message.build_id.map(|id| id.to_string()),
                                dataflow_id = ?message.dataflow_id.map(|id| id.to_string()),
                                node_id = ?message.node_id.map(|id| id.to_string()),
                                target = message.target,
                                module_path = message.module_path,
                                file = message.file,
                                line = message.line,
                                "{}",
                                Indent(&message.message)
                            );
                        }
                        LogLevel::Warn => {
                            tracing::warn!(
                                build_id = ?message.build_id.map(|id| id.to_string()),
                                dataflow_id = ?message.dataflow_id.map(|id| id.to_string()),
                                node_id = ?message.node_id.map(|id| id.to_string()),
                                target = message.target,
                                module_path = message.module_path,
                                file = message.file,
                                line = message.line,
                                "{}",
                                Indent(&message.message)
                            );
                        }
                        LogLevel::Info => {
                            tracing::info!(
                                build_id = ?message.build_id.map(|id| id.to_string()),
                                dataflow_id = ?message.dataflow_id.map(|id| id.to_string()),
                                node_id = ?message.node_id.map(|id| id.to_string()),
                                target = message.target,
                                module_path = message.module_path,
                                file = message.file,
                                line = message.line,
                                "{}",
                                Indent(&message.message)
                            );
                        }
                        LogLevel::Debug => {
                            tracing::debug!(
                                build_id = ?message.build_id.map(|id| id.to_string()),
                                dataflow_id = ?message.dataflow_id.map(|id| id.to_string()),
                                node_id = ?message.node_id.map(|id| id.to_string()),
                                target = message.target,
                                module_path = message.module_path,
                                file = message.file,
                                line = message.line,
                                "{}",
                                Indent(&message.message)
                            );
                        }
                        _ => {}
                    },
                }
            }
        }
    }

    pub async fn try_clone(&self) -> eyre::Result<Self> {
        let destination = match &self.destination {
            LogDestination::Coordinator { sender } => LogDestination::Coordinator {
                sender: sender.clone(),
            },
            LogDestination::Channel { sender } => LogDestination::Channel {
                sender: sender.clone(),
            },
            LogDestination::Tracing => LogDestination::Tracing,
        };

        Ok(Self {
            destination,
            daemon_id: self.daemon_id.clone(),
            clock: self.clock.clone(),
        })
    }

    async fn log_to_coordinator(
        message: Timestamped<CoordinatorRequest>,
        sender: &CoordinatorSender,
    ) {
        let msg = serde_json::to_vec(&message).expect("failed to serialize log message");
        match sender
            .send_event(&msg)
            .await
            .wrap_err("failed to send log message to dora-coordinator")
        {
            Ok(()) => (),
            Err(err) => tracing::warn!("{err:?}"),
        }
    }
}

pub enum LogDestination {
    Coordinator { sender: CoordinatorSender },
    Channel { sender: Sender<LogMessage> },
    Tracing,
}

enum CowMut<'a, T> {
    Borrowed(&'a mut T),
    Owned(T),
}

impl<T> Deref for CowMut<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        match self {
            CowMut::Borrowed(v) => v,
            CowMut::Owned(v) => v,
        }
    }
}

impl<T> DerefMut for CowMut<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            CowMut::Borrowed(v) => v,
            CowMut::Owned(v) => v,
        }
    }
}

struct Indent<'a>(&'a str);

impl std::fmt::Display for Indent<'_> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for line in self.0.lines() {
            write!(f, "   {line}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_path_format() {
        let dir = Path::new("/tmp/work");
        let uuid = Uuid::nil();
        let node = NodeId::from("sensor".to_string());
        let p = log_path(dir, &uuid, &node);
        assert_eq!(
            p,
            PathBuf::from("/tmp/work/out/00000000-0000-0000-0000-000000000000/log_sensor.jsonl")
        );
    }

    #[test]
    fn log_path_rotated_format() {
        let dir = Path::new("/tmp/work");
        let uuid = Uuid::nil();
        let node = NodeId::from("sensor".to_string());
        let p = log_path_rotated(dir, &uuid, &node, 3);
        assert_eq!(
            p,
            PathBuf::from("/tmp/work/out/00000000-0000-0000-0000-000000000000/log_sensor.3.jsonl")
        );
    }

    #[test]
    fn rotate_creates_dot_1_from_current() {
        let tmp = tempfile::tempdir().unwrap();
        let uuid = Uuid::nil();
        let node = NodeId::from("n".to_string());

        // Create the out/<uuid> directory and current log file
        let dataflow_dir = tmp.path().join("out").join(uuid.to_string());
        std::fs::create_dir_all(&dataflow_dir).unwrap();
        let current = log_path(tmp.path(), &uuid, &node);
        std::fs::write(&current, "line1\n").unwrap();

        rotate_log_files(tmp.path(), &uuid, &node, 5).unwrap();

        // Current should be gone, .1 should exist
        assert!(!current.exists());
        let rotated = log_path_rotated(tmp.path(), &uuid, &node, 1);
        assert!(rotated.exists());
        assert_eq!(std::fs::read_to_string(&rotated).unwrap(), "line1\n");
    }

    #[test]
    fn rotate_shifts_existing_files() {
        let tmp = tempfile::tempdir().unwrap();
        let uuid = Uuid::nil();
        let node = NodeId::from("n".to_string());

        let dataflow_dir = tmp.path().join("out").join(uuid.to_string());
        std::fs::create_dir_all(&dataflow_dir).unwrap();

        // Create current and .1
        std::fs::write(log_path(tmp.path(), &uuid, &node), "current\n").unwrap();
        std::fs::write(log_path_rotated(tmp.path(), &uuid, &node, 1), "old1\n").unwrap();

        rotate_log_files(tmp.path(), &uuid, &node, 5).unwrap();

        // .1 should be old current, .2 should be old .1
        assert_eq!(
            std::fs::read_to_string(log_path_rotated(tmp.path(), &uuid, &node, 1)).unwrap(),
            "current\n"
        );
        assert_eq!(
            std::fs::read_to_string(log_path_rotated(tmp.path(), &uuid, &node, 2)).unwrap(),
            "old1\n"
        );
    }

    #[test]
    fn rotate_deletes_oldest_beyond_max() {
        let tmp = tempfile::tempdir().unwrap();
        let uuid = Uuid::nil();
        let node = NodeId::from("n".to_string());

        let dataflow_dir = tmp.path().join("out").join(uuid.to_string());
        std::fs::create_dir_all(&dataflow_dir).unwrap();

        let max = 2;
        // Create current, .1, .2 (at max)
        std::fs::write(log_path(tmp.path(), &uuid, &node), "current\n").unwrap();
        std::fs::write(log_path_rotated(tmp.path(), &uuid, &node, 1), "old1\n").unwrap();
        std::fs::write(log_path_rotated(tmp.path(), &uuid, &node, 2), "old2\n").unwrap();

        rotate_log_files(tmp.path(), &uuid, &node, max).unwrap();

        // .2 (the max) should now be what was .1, old .2 is deleted
        assert!(!log_path(tmp.path(), &uuid, &node).exists());
        assert_eq!(
            std::fs::read_to_string(log_path_rotated(tmp.path(), &uuid, &node, 1)).unwrap(),
            "current\n"
        );
        assert_eq!(
            std::fs::read_to_string(log_path_rotated(tmp.path(), &uuid, &node, 2)).unwrap(),
            "old1\n"
        );
    }

    #[test]
    fn rotate_noop_when_no_files() {
        let tmp = tempfile::tempdir().unwrap();
        let uuid = Uuid::nil();
        let node = NodeId::from("n".to_string());

        let dataflow_dir = tmp.path().join("out").join(uuid.to_string());
        std::fs::create_dir_all(&dataflow_dir).unwrap();

        // Should not error even with no files
        rotate_log_files(tmp.path(), &uuid, &node, 5).unwrap();
    }
}
