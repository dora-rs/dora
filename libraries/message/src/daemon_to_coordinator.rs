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
    dora_version: semver::Version,
    pub machine_id: Option<String>,
    /// System-level unique machine identifier (e.g. `/etc/machine-id` on Linux).
    /// Used to reliably detect whether CLI and daemon run on the same machine,
    /// even behind NAT.
    #[serde(default)]
    pub machine_uid: Option<String>,
}

impl DaemonRegisterRequest {
    pub fn new(machine_id: Option<String>) -> Self {
        Self {
            dora_version: current_crate_version(),
            machine_id,
            machine_uid: crate::common::machine_uid(),
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

/// Request payload for reading a shared state value.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
pub struct StateGetRequest {
    /// Logical namespace for the state key.
    pub namespace: String,
    /// Key inside the namespace.
    pub key: String,
}

/// Request payload for writing a shared state value.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
pub struct StateSetRequest {
    /// Logical namespace for the state key.
    pub namespace: String,
    /// Key inside the namespace.
    pub key: String,
    /// Value bytes to store.
    pub value: Vec<u8>,
    /// Optional TTL in milliseconds.
    ///
    /// `None` means the key does not expire.
    pub ttl_ms: Option<u64>,
}

/// Request payload for deleting a shared state value.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
pub struct StateDeleteRequest {
    /// Logical namespace for the state key.
    pub namespace: String,
    /// Key inside the namespace.
    pub key: String,
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
    /// Read a shared state value by namespace/key.
    async fn state_get(request: StateGetRequest) -> Result<Option<Vec<u8>>, String>;
    /// Write a shared state value by namespace/key with optional TTL.
    async fn state_set(request: StateSetRequest) -> Result<(), String>;
    /// Delete a shared state value by namespace/key.
    ///
    /// Returns `true` if a key was removed, `false` if the key was missing.
    async fn state_delete(request: StateDeleteRequest) -> Result<bool, String>;
}
