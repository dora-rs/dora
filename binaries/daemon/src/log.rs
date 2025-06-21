use std::{
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
}

pub struct DataflowLogger<'a> {
    dataflow_id: Uuid,
    logger: &'a mut DaemonLogger,
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
            logger: self.logger,
        }
    }

    pub fn inner(&self) -> &DaemonLogger {
        self.logger
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
}

pub struct DaemonLogger {
    daemon_id: DaemonId,
    logger: Logger,
}

impl DaemonLogger {
    pub fn for_dataflow(&mut self, dataflow_id: Uuid) -> DataflowLogger {
        DataflowLogger {
            dataflow_id,
            logger: self,
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

        // Unified log output format
        let daemon_id = message.daemon_id.as_ref().and_then(format_id);
        let dataflow_id = format_id(&message.dataflow_id);
        let node_id = message.node_id.as_ref();

        let id_str = match (daemon_id, dataflow_id, node_id) {
            (Some(did), Some(dfid), Some(nid)) => format!("daemon:{} dataflow:{} {}", did, dfid, nid),
            (Some(did), Some(dfid), None)      => format!("daemon:{} dataflow:{}", did, dfid),
            (Some(did), None, Some(nid))       => format!("daemon:{} {}", did, nid),
            (None, Some(dfid), Some(nid))      => format!("dataflow:{} {}", dfid, nid),
            (Some(did), None, None)            => format!("daemon:{}", did),
            (None, Some(dfid), None)           => format!("dataflow:{}", dfid),
            (None, None, Some(nid))            => format!("{}", nid),
            (None, None, None)                 => String::new(),
        };

        let log_prefix = if id_str.is_empty() {
            String::new()
        } else {
            format!("[{}] ", id_str)
        };

        for line in message.message.lines() {
            match message.level {
                LogLevel::Error => tracing::error!("{}{}", log_prefix, line),
                LogLevel::Warn  => tracing::warn!("{}{}", log_prefix, line),
                LogLevel::Info  => tracing::info!("{}{}", log_prefix, line),
                LogLevel::Debug => tracing::debug!("{}{}", log_prefix, line),
                LogLevel::Trace => tracing::trace!("{}{}", log_prefix, line),
            }
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

fn id_mode() -> String {
    std::env::var("DORA_LOG_ID_MODE").unwrap_or_else(|_| "full".to_string())
}

fn format_id(id: &impl ToString) -> Option<String> {
    let mode = id_mode();
    match mode.as_str() {
        "full" => Some(id.to_string()),
        "short" => {
            let s = id.to_string();
            if s.len() > 6 {
                Some(s[s.len() - 6..].to_string())
            } else {
                Some(s)
            }
        }
        "none" => None,
        _ => Some(id.to_string()),
    }
}
