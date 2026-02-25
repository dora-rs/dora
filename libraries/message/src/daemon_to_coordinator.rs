use std::collections::BTreeMap;

pub use crate::common::{
    DataMessage, LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus, Timestamped,
};
use crate::{
    BuildId, DataflowId, common::DaemonId, current_crate_version, id::NodeId, versions_compatible,
};

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register(DaemonRegisterRequest),
    /// Register a reverse-channel connection for daemon→coordinator RPC.
    ///
    /// Sent on a second TCP connection after the initial registration.
    /// The coordinator sets up a `DaemonToCoordinatorControl` tarpc server
    /// on this connection.
    RegisterReverseChannel {
        daemon_id: DaemonId,
    },
    Event {
        daemon_id: DaemonId,
        event: DaemonEvent,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct DaemonRegisterRequest {
    dora_version: semver::Version,
    pub machine_id: Option<String>,
}

impl DaemonRegisterRequest {
    pub fn new(machine_id: Option<String>) -> Self {
        Self {
            dora_version: current_crate_version(),
            machine_id,
        }
    }

    pub fn check_version(&self) -> Result<(), String> {
        let crate_version = current_crate_version();
        let specified_version = &self.dora_version;

        if versions_compatible(&crate_version, specified_version)? {
            Ok(())
        } else {
            Err(format!(
                "version mismatch: message format v{} is not compatible \
                with expected message format v{crate_version}",
                self.dora_version
            ))
        }
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DaemonEvent {
    BuildResult {
        build_id: BuildId,
        result: Result<(), String>,
    },
    SpawnResult {
        dataflow_id: DataflowId,
        result: Result<(), String>,
    },
    AllNodesReady {
        dataflow_id: DataflowId,
        exited_before_subscribe: Vec<NodeId>,
    },
    AllNodesFinished {
        dataflow_id: DataflowId,
        result: DataflowDaemonResult,
    },
    Heartbeat,
    Log(LogMessage),
    Exit,
    NodeMetrics {
        dataflow_id: DataflowId,
        metrics: BTreeMap<NodeId, NodeMetrics>,
    },
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

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorReply {
    TriggerBuildResult(Result<(), String>),
    TriggerSpawnResult(Result<(), String>),
    ReloadResult(Result<(), String>),
    StopResult(Result<(), String>),
    DestroyResult { result: Result<(), String> },
    Logs(Result<Vec<u8>, String>),
}

/// tarpc service for daemon→coordinator RPC calls.
///
/// This replaces the raw TCP `DaemonEvent` messages sent over
/// `coordinator_connection`. The coordinator runs a tarpc server
/// implementing this trait, and each daemon holds a client.
#[tarpc::service]
pub trait DaemonToCoordinatorControl {
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
