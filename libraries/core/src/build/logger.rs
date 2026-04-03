use std::future::Future;

use dora_message::BuildId;
pub use dora_message::common::LogLevelOrStdout;
use dora_message::id::NodeId;

pub trait BuildLogger: Send {
    type Clone: BuildLogger + 'static;

    fn log_message(
        &mut self,
        level: impl Into<LogLevelOrStdout> + Send,
        message: impl Into<String> + Send,
    ) -> impl Future<Output = ()> + Send;

    fn log_stdout(&mut self, message: impl Into<String> + Send) -> impl Future<Output = ()> + Send {
        self.log_message(LogLevelOrStdout::Stdout, message)
    }

    fn try_clone(&self) -> impl Future<Output = eyre::Result<Self::Clone>> + Send;
}

/// A [`BuildLogger`] that forwards messages to the `tracing` crate.
///
/// Used by the daemon's RPC server path where a full `DaemonLogger`
/// (with TCP connection) is not available.
#[derive(Clone)]
pub struct TracingBuildLogger {
    build_id: BuildId,
    node_id: NodeId,
}

impl TracingBuildLogger {
    pub fn new(build_id: BuildId, node_id: NodeId) -> Self {
        Self { build_id, node_id }
    }
}

impl BuildLogger for TracingBuildLogger {
    type Clone = TracingBuildLogger;

    async fn log_message(
        &mut self,
        level: impl Into<LogLevelOrStdout> + Send,
        message: impl Into<String> + Send,
    ) {
        let message = message.into();
        match level.into() {
            LogLevelOrStdout::Stdout => {
                tracing::info!(build_id = %self.build_id, node_id = %self.node_id, "{message}");
            }
            LogLevelOrStdout::LogLevel(dora_message::common::LogLevel::Error) => {
                tracing::error!(build_id = %self.build_id, node_id = %self.node_id, "{message}");
            }
            LogLevelOrStdout::LogLevel(dora_message::common::LogLevel::Warn) => {
                tracing::warn!(build_id = %self.build_id, node_id = %self.node_id, "{message}");
            }
            LogLevelOrStdout::LogLevel(dora_message::common::LogLevel::Info) => {
                tracing::info!(build_id = %self.build_id, node_id = %self.node_id, "{message}");
            }
            _ => {
                tracing::debug!(build_id = %self.build_id, node_id = %self.node_id, "{message}");
            }
        }
    }

    async fn try_clone(&self) -> eyre::Result<Self::Clone> {
        Ok(self.clone())
    }
}
