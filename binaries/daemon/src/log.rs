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
    common::{DaemonId, LogLevel, LogMessage, Timestamped},
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent},
    BuildId,
};
use eyre::Context;
use flume::Sender;
use tokio::net::TcpStream;
use uuid::Uuid;

use crate::socket_stream_utils::socket_stream_send;

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
            LogDestination::Coordinator {
                coordinator_connection,
            } => {
                let message = Timestamped {
                    inner: CoordinatorRequest::Event {
                        daemon_id: self.daemon_id.clone(),
                        event: DaemonEvent::Log(message.clone()),
                    },
                    timestamp: self.clock.new_timestamp(),
                };
                Self::log_to_coordinator(message, coordinator_connection).await
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
            LogDestination::Coordinator {
                coordinator_connection,
            } => {
                let addr = coordinator_connection
                    .peer_addr()
                    .context("failed to get coordinator peer addr")?;
                let new_connection = TcpStream::connect(addr)
                    .await
                    .context("failed to connect to coordinator during logger clone")?;
                LogDestination::Coordinator {
                    coordinator_connection: new_connection,
                }
            }
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
        connection: &mut TcpStream,
    ) {
        let msg = serde_json::to_vec(&message).expect("failed to serialize log message");
        match socket_stream_send(connection, &msg)
            .await
            .wrap_err("failed to send log message to dora-coordinator")
        {
            Ok(()) => (),
            Err(err) => tracing::warn!("{err:?}"),
        }
    }
}

pub enum LogDestination {
    Coordinator { coordinator_connection: TcpStream },
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
