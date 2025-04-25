use std::{
    ops::{Deref, DerefMut},
    path::{Path, PathBuf},
    sync::Arc,
};

use dora_core::{config::NodeId, uhlc};
use dora_message::{
    common::{DaemonId, LogLevel, LogMessage, Timestamped},
    daemon_to_coordinator::{CoordinatorRequest, DaemonEvent},
};
use eyre::Context;
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
            .log(level, self.dataflow_id, node_id, target, message)
            .await
    }

    pub async fn try_clone(&self) -> eyre::Result<DataflowLogger<'static>> {
        Ok(DataflowLogger {
            dataflow_id: self.dataflow_id,
            logger: CowMut::Owned(self.logger.try_clone().await?),
        })
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

    pub fn inner(&self) -> &Logger {
        &self.logger
    }

    pub async fn log(
        &mut self,
        level: LogLevel,
        dataflow_id: Uuid,
        node_id: Option<NodeId>,
        target: Option<String>,
        message: impl Into<String>,
    ) {
        let message = LogMessage {
            daemon_id: Some(self.daemon_id.clone()),
            dataflow_id,
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
    pub(super) coordinator_connection: Option<TcpStream>,
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
        if let Some(connection) = &mut self.coordinator_connection {
            let msg = serde_json::to_vec(&Timestamped {
                inner: CoordinatorRequest::Event {
                    daemon_id: self.daemon_id.clone(),
                    event: DaemonEvent::Log(message.clone()),
                },
                timestamp: self.clock.new_timestamp(),
            })
            .expect("failed to serialize log message");
            match socket_stream_send(connection, &msg)
                .await
                .wrap_err("failed to send log message to dora-coordinator")
            {
                Ok(()) => return,
                Err(err) => tracing::warn!("{err:?}"),
            }
        }

        // log message using tracing if reporting to coordinator is not possible
        match message.level {
            LogLevel::Error => {
                tracing::error!(
                    dataflow_id = message.dataflow_id.to_string(),
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
                    dataflow_id = message.dataflow_id.to_string(),
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
                    dataflow_id = message.dataflow_id.to_string(),
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
                    dataflow_id = message.dataflow_id.to_string(),
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
        }
    }

    pub async fn try_clone(&self) -> eyre::Result<Self> {
        let coordinator_connection = match &self.coordinator_connection {
            Some(c) => {
                let addr = c
                    .peer_addr()
                    .context("failed to get coordinator peer addr")?;
                let new_connection = TcpStream::connect(addr)
                    .await
                    .context("failed to connect to coordinator during logger clone")?;
                Some(new_connection)
            }
            None => None,
        };

        Ok(Self {
            coordinator_connection,
            daemon_id: self.daemon_id.clone(),
            clock: self.clock.clone(),
        })
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
            write!(f, "   {}", line)?;
        }
        Ok(())
    }
}
