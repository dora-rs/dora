use std::{
    ops::{Deref, DerefMut},
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
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

    /// Point the shared coordinator log target at a freshly-established
    /// connection. Mutates the `Arc<Mutex<…>>` **in place** rather than swapping
    /// the destination, so every per-node log-forwarding clone captured at spawn
    /// follows the reconnect and emits under the current sender + daemon id
    /// (dora-rs/dora#2029). No-op for non-coordinator destinations.
    pub(crate) fn update_coordinator_target(&self, sender: CoordinatorSender, daemon_id: DaemonId) {
        if let LogDestination::Coordinator { target } = &self.logger.destination
            && let Ok(mut guard) = target.lock()
        {
            guard.sender = sender;
            guard.daemon_id = daemon_id;
        }
    }

    /// Adopt the daemon id the coordinator assigned on (re)registration. The
    /// coordinator rejects any event whose `daemon_id` differs from the one it
    /// handed out, so log payloads carry the current id after a reconnect
    /// (dora-rs/dora#2029). The validated wire id for coordinator log events
    /// comes from the shared target (`update_coordinator_target`).
    pub(crate) fn set_daemon_id(&mut self, daemon_id: DaemonId) {
        self.logger.daemon_id = daemon_id.clone();
        self.daemon_id = daemon_id;
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
            LogDestination::Coordinator { target } => {
                // Read the *current* sender + daemon id from the shared target,
                // so a clone captured before a reconnect still forwards over the
                // live connection with the freshly-assigned id (#2029). The guard
                // is confined to this block — it must not span the await below
                // (`MutexGuard` is `!Send`).
                let Some((sender, daemon_id)) = (match target.lock() {
                    Ok(t) => Some((t.sender.clone(), t.daemon_id.clone())),
                    Err(_) => None,
                }) else {
                    return;
                };
                let message = Timestamped {
                    inner: CoordinatorRequest::Event {
                        daemon_id,
                        event: DaemonEvent::Log(message.clone()),
                    },
                    timestamp: self.clock.new_timestamp(),
                };
                Self::log_to_coordinator(message, &sender).await
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
            LogDestination::Coordinator { target } => LogDestination::Coordinator {
                // Clone the `Arc`, NOT the contents: every clone must share the
                // same target so a reconnect update reaches them all (#2029).
                target: target.clone(),
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
        let msg = match serde_json::to_vec(&message) {
            Ok(msg) => msg,
            Err(err) => {
                tracing::warn!("failed to serialize log message: {err}");
                return;
            }
        };
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
    Coordinator {
        target: Arc<Mutex<CoordinatorLogTarget>>,
    },
    Channel {
        sender: Sender<LogMessage>,
    },
    Tracing,
}

/// The coordinator connection the daemon currently forwards logs over.
///
/// Shared (behind `Arc<Mutex<…>>`) by the daemon's logger and every per-node
/// log-forwarding clone, which capture it at spawn. On a reconnect the daemon
/// swaps the contents **in place** (`update_coordinator_target`), so a
/// surviving node's logs follow the new connection and carry the
/// freshly-assigned daemon id — the coordinator validates that id and would
/// reject a stale one (dora-rs/dora#2029).
pub struct CoordinatorLogTarget {
    pub sender: CoordinatorSender,
    pub daemon_id: DaemonId,
}

impl CoordinatorLogTarget {
    pub fn shared(sender: CoordinatorSender, daemon_id: DaemonId) -> Arc<Mutex<Self>> {
        Arc::new(Mutex::new(Self { sender, daemon_id }))
    }
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
        let mut lines = self.0.lines().peekable();
        while let Some(line) = lines.next() {
            if lines.peek().is_some() {
                writeln!(f, "   {line}")?;
            } else {
                write!(f, "   {line}")?;
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coordinator::CoordinatorSender;

    /// #2029 P2: a per-node log-forwarding clone captured at spawn must follow a
    /// later reconnect — after `update_coordinator_target`, the clone emits under
    /// the freshly-assigned daemon id (which the coordinator validates) and the
    /// new sender, instead of the stale ones captured when the node was spawned.
    #[tokio::test]
    async fn coordinator_log_target_update_reaches_spawn_time_clones() {
        let clock = Arc::new(uhlc::HLC::default());
        let id_a = DaemonId::new(Some("daemon-a".to_string()));
        let id_b = DaemonId::new(Some("daemon-b".to_string()));
        let (sender_a, _rx_a) = CoordinatorSender::for_test();
        let (sender_b, _rx_b) = CoordinatorSender::for_test();

        let daemon_logger = Logger {
            destination: LogDestination::Coordinator {
                target: CoordinatorLogTarget::shared(sender_a, id_a.clone()),
            },
            daemon_id: id_a.clone(),
            clock,
        }
        .for_daemon(id_a.clone());

        // The node log task captures a clone at spawn (before any reconnect).
        let node_clone = daemon_logger.try_clone().await.expect("clone logger");

        // Reconnect: the daemon adopts a new id + connection and swaps the
        // shared target in place.
        daemon_logger.update_coordinator_target(sender_b, id_b.clone());

        // The pre-reconnect clone must now read the new id (proves the shared
        // target is what every clone observes).
        let observed = match &node_clone.logger.destination {
            LogDestination::Coordinator { target } => target.lock().unwrap().daemon_id.clone(),
            _ => panic!("expected a coordinator log destination"),
        };
        assert_eq!(
            observed, id_b,
            "node log clone must follow the reconnect, not keep the spawn-time id"
        );
    }

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

    #[test]
    fn indent_preserves_newlines_between_lines() {
        let out = Indent("line1\nline2\nline3").to_string();
        // No trailing newline: tracing adds its own per-event newline.
        assert_eq!(out, "   line1\n   line2\n   line3");
    }

    #[test]
    fn indent_single_line_no_trailing_newline() {
        let out = Indent("hello").to_string();
        // No trailing newline: tracing adds its own per-event newline.
        assert_eq!(out, "   hello");
    }

    #[test]
    fn indent_empty_string_produces_empty() {
        let out = Indent("").to_string();
        assert_eq!(out, "");
    }
}
