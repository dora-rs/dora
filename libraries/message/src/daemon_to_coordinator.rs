use std::collections::BTreeMap;

pub use crate::common::{
    DataMessage, LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus, Timestamped,
};
use crate::{BuildId, DataflowId, common::DaemonId, id::NodeId};

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register(DaemonRegisterRequest),
    /// Register a notification channel for daemon→coordinator RPC.
    ///
    /// Sent on a second TCP connection after the initial registration.
    /// The coordinator sets up a `CoordinatorNotify` tarpc server
    /// on this connection.
    RegisterNotificationChannel {
        daemon_id: DaemonId,
    },
    /// Forward a log message from a daemon over the legacy raw-TCP path.
    ///
    /// All other daemon→coordinator communication now uses the
    /// `CoordinatorNotify` tarpc service on the notification channel.
    Log {
        daemon_id: DaemonId,
        message: LogMessage,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct DaemonRegisterRequest {
    pub machine_id: Option<String>,
}

impl DaemonRegisterRequest {
    pub fn new(machine_id: Option<String>) -> Self {
        Self { machine_id }
    }
}

/// Resource metrics for a node process
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NodeMetrics {
    /// Process ID
    pub pid: u32,
    /// CPU usage percentage (0-100 per core)
    pub cpu_usage: f32,
    /// Memory usage in bytes
    pub memory_bytes: u64,
    /// Disk read bytes per second (if available)
    pub disk_read_bytes: Option<u64>,
    /// Disk write bytes per second (if available)
    pub disk_write_bytes: Option<u64>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowDaemonResult {
    pub timestamp: uhlc::Timestamp,
    pub node_results: BTreeMap<NodeId, Result<(), NodeError>>,
}

impl DataflowDaemonResult {
    pub fn is_ok(&self) -> bool {
        self.node_results.values().all(|r| r.is_ok())
    }
}

/// tarpc service for daemon→coordinator notifications.
///
/// The coordinator runs a tarpc server implementing this trait,
/// and each daemon holds a client to notify the coordinator about
/// events such as node readiness, dataflow completion, and metrics.
#[tarpc::service]
pub trait CoordinatorNotify {
    /// Report that all local nodes on this daemon are ready.
    async fn all_nodes_ready(dataflow_id: DataflowId, exited_before_subscribe: Vec<NodeId>);
    /// Report that all nodes on this daemon have finished.
    async fn all_nodes_finished(dataflow_id: DataflowId, result: DataflowDaemonResult);
    /// Daemon heartbeat.
    async fn heartbeat();
    /// Forward a log message to the coordinator.
    async fn log(message: LogMessage);
    /// Notify the coordinator that this daemon is exiting.
    async fn daemon_exit();
    /// Report resource metrics for running nodes.
    async fn node_metrics(dataflow_id: DataflowId, metrics: BTreeMap<NodeId, NodeMetrics>);
    /// Report that a build has completed (or failed) on this daemon.
    async fn build_result(build_id: BuildId, result: Result<(), String>);
    /// Report that a dataflow spawn has completed (or failed) on this daemon.
    async fn spawn_result(dataflow_id: DataflowId, result: Result<(), String>);
}
