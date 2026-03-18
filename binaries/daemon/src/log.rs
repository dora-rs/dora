use std::{
    collections::HashMap,
    ops::{Deref, DerefMut},
    path::{Path, PathBuf},
    sync::Arc,
};

use dora_core::{
    build::{BuildLogger, LogLevelOrStdout},
    config::NodeId,
    topics::{
        zenoh_log_topic_for_build_daemon, zenoh_log_topic_for_build_node,
        zenoh_log_topic_for_dataflow_daemon, zenoh_log_topic_for_dataflow_node,
    },
    uhlc,
};
use dora_message::{
    BuildId,
    common::{DaemonId, LogLevel, LogMessage},
};
use flume::Sender;
use uuid::Uuid;

pub fn log_path(working_dir: &Path, dataflow_id: &Uuid, node_id: &NodeId) -> PathBuf {
    let dataflow_dir = working_dir.join("out").join(dataflow_id.to_string());
    dataflow_dir.join(format!("log_{node_id}.txt"))
}

pub struct NodeLogger<'a> {
    node_id: NodeId,
    logger: DataflowLogger<'a>,
}

impl NodeLogger<'_> {
    pub fn inner(&self) -> &DataflowLogger {
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

    pub fn reborrow(&mut self) -> DataflowLogger {
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
    pub fn for_dataflow(&mut self, dataflow_id: Uuid) -> DataflowLogger {
        DataflowLogger {
            dataflow_id,
            logger: CowMut::Borrowed(self),
        }
    }

    pub fn for_node_build(&mut self, build_id: BuildId, node_id: NodeId) -> NodeBuildLogger {
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
        self.logger.log(message, &self.daemon_id).await
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
        self.logger.log(message, &self.daemon_id).await
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
    pub(super) clock: Arc<uhlc::HLC>,
}

impl Logger {
    pub fn for_daemon(self, daemon_id: DaemonId) -> DaemonLogger {
        DaemonLogger {
            daemon_id,
            logger: self,
        }
    }

    pub async fn log(&mut self, message: LogMessage, daemon_id: &DaemonId) {
        match &mut self.destination {
            LogDestination::Zenoh {
                session,
                publishers,
            } => {
                let topic = if let Some(dataflow_id) = &message.dataflow_id {
                    if let Some(node_id) = &message.node_id {
                        zenoh_log_topic_for_dataflow_node(*dataflow_id, node_id, &message.level)
                    } else {
                        zenoh_log_topic_for_dataflow_daemon(*dataflow_id, daemon_id, &message.level)
                    }
                } else if let Some(build_id) = &message.build_id {
                    if let Some(node_id) = &message.node_id {
                        zenoh_log_topic_for_build_node(build_id, node_id, &message.level)
                    } else {
                        zenoh_log_topic_for_build_daemon(build_id, daemon_id, &message.level)
                    }
                } else {
                    // No dataflow or build context; fall back to tracing.
                    Self::log_via_tracing(&message);
                    return;
                };

                // Re-use an existing publisher or create one with a
                // matching listener so we only serialize + send when at
                // least one subscriber is listening on this topic.
                let entry = publishers.entry(topic.clone());
                let pub_entry = match entry {
                    std::collections::hash_map::Entry::Occupied(o) => o.into_mut(),
                    std::collections::hash_map::Entry::Vacant(v) => {
                        match Self::create_publisher(session, &topic).await {
                            Ok(pe) => v.insert(pe),
                            Err(err) => {
                                tracing::warn!(
                                    "failed to create zenoh publisher for {topic}: {err}"
                                );
                                return;
                            }
                        }
                    }
                };

                if !pub_entry.has_matching_subscribers() {
                    return;
                }

                let payload =
                    serde_json::to_vec(&message).expect("failed to serialize log message");
                if let Err(err) = pub_entry.publisher.put(payload).await {
                    tracing::warn!("failed to publish log via zenoh: {err}");
                }
            }
            LogDestination::Channel { sender } => {
                let _ = sender.send_async(message).await;
            }
            LogDestination::Tracing => {
                Self::log_via_tracing(&message);
            }
        }
    }

    pub async fn try_clone(&self) -> eyre::Result<Self> {
        let destination = match &self.destination {
            LogDestination::Zenoh { session, .. } => LogDestination::Zenoh {
                session: session.clone(),
                publishers: std::collections::HashMap::new(),
            },
            LogDestination::Channel { sender } => LogDestination::Channel {
                sender: sender.clone(),
            },
            LogDestination::Tracing => LogDestination::Tracing,
        };

        Ok(Self {
            destination,
            clock: self.clock.clone(),
        })
    }

    /// Create a publisher with a matching listener that tracks whether
    /// any subscribers exist for its topic.
    async fn create_publisher(session: &zenoh::Session, topic: &str) -> eyre::Result<LogPublisher> {
        let topic: zenoh::key_expr::KeyExpr<'static> =
            zenoh::key_expr::KeyExpr::try_from(topic.to_owned()).map_err(|e| eyre::eyre!("{e}"))?;
        let publisher = session
            .declare_publisher(topic)
            .await
            .map_err(|e| eyre::eyre!(e))?;

        let has_subscribers = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let flag = has_subscribers.clone();
        let _matching_listener = publisher
            .matching_listener()
            .callback(move |status| {
                flag.store(status.matching(), std::sync::atomic::Ordering::Relaxed);
            })
            .await
            .map_err(|e| eyre::eyre!(e))?;

        Ok(LogPublisher {
            publisher,
            has_subscribers,
            _matching_listener,
        })
    }

    fn log_via_tracing(message: &LogMessage) {
        match message.level {
            LogLevelOrStdout::Stdout => {
                tracing::info!(
                    build_id = ?message.build_id.map(|id| id.to_string()),
                    dataflow_id = ?message.dataflow_id.map(|id| id.to_string()),
                    node_id = ?message.node_id.as_ref().map(|id| id.to_string()),
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
                        node_id = ?message.node_id.as_ref().map(|id| id.to_string()),
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
                        node_id = ?message.node_id.as_ref().map(|id| id.to_string()),
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
                        node_id = ?message.node_id.as_ref().map(|id| id.to_string()),
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
                        node_id = ?message.node_id.as_ref().map(|id| id.to_string()),
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

pub enum LogDestination {
    Zenoh {
        session: zenoh::Session,
        publishers: HashMap<String, LogPublisher>,
    },
    Channel {
        sender: Sender<LogMessage>,
    },
    Tracing,
}

/// A zenoh publisher paired with a matching listener that tracks
/// whether any subscribers are currently listening.
pub struct LogPublisher {
    publisher: zenoh::pubsub::Publisher<'static>,
    has_subscribers: Arc<std::sync::atomic::AtomicBool>,
    /// Kept alive so the matching callback stays registered.
    _matching_listener: zenoh::matching::MatchingListener<()>,
}

impl LogPublisher {
    fn has_matching_subscribers(&self) -> bool {
        self.has_subscribers
            .load(std::sync::atomic::Ordering::Relaxed)
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
        for line in self.0.lines() {
            write!(f, "   {line}")?;
        }
        Ok(())
    }
}
