use std::collections::BTreeMap;

pub use crate::common::{
    DataMessage, LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus, Timestamped,
};
use crate::{
    BuildId, DataflowId, common::DaemonId, current_crate_version, id::NodeId, versions_compatible,
};

/// Per-dataflow status reported by a daemon after (re-)registration.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowStatusEntry {
    pub dataflow_id: uuid::Uuid,
    pub running_nodes: Vec<NodeId>,
}

#[allow(clippy::large_enum_variant)]
#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register(DaemonRegisterRequest),
    Event {
        daemon_id: DaemonId,
        event: DaemonEvent,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct DaemonRegisterRequest {
    adora_version: semver::Version,
    pub machine_id: Option<String>,
    #[serde(default)]
    pub labels: BTreeMap<String, String>,
}

impl DaemonRegisterRequest {
    pub fn new(machine_id: Option<String>, labels: BTreeMap<String, String>) -> Self {
        Self {
            adora_version: current_crate_version(),
            machine_id,
            labels,
        }
    }

    pub fn check_version(&self) -> Result<(), String> {
        let crate_version = current_crate_version();
        let specified_version = &self.adora_version;

        if versions_compatible(&crate_version, specified_version)? {
            Ok(())
        } else {
            Err(format!(
                "version mismatch: message format v{} is not compatible \
                with expected message format v{crate_version}",
                self.adora_version
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
    Heartbeat {
        #[serde(default)]
        ft_stats: Option<FaultToleranceSnapshot>,
    },
    /// Sent by the daemon after registration to report its current state.
    /// Enables coordinator-daemon reconciliation on reconnect.
    StatusReport {
        running_dataflows: Vec<DataflowStatusEntry>,
    },
    Log(LogMessage),
    Exit,
    NodeMetrics {
        dataflow_id: DataflowId,
        metrics: BTreeMap<NodeId, NodeMetrics>,
        #[serde(default)]
        network: Option<NetworkMetrics>,
    },
}

/// Health status of a node
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
#[serde(rename_all = "kebab-case")]
pub enum NodeStatus {
    #[default]
    Running,
    Restarting,
    /// One or more inputs have timed out (circuit breaker open)
    Degraded,
    Failed,
}

impl std::fmt::Display for NodeStatus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NodeStatus::Running => write!(f, "Running"),
            NodeStatus::Restarting => write!(f, "Restarting"),
            NodeStatus::Degraded => write!(f, "Degraded"),
            NodeStatus::Failed => write!(f, "Failed"),
        }
    }
}

/// Snapshot of daemon-level fault tolerance counters
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct FaultToleranceSnapshot {
    pub restarts: u64,
    pub health_check_kills: u64,
    pub input_timeouts: u64,
    pub circuit_breaker_recoveries: u64,
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
    /// Number of times this node has been restarted
    #[serde(default)]
    pub restart_count: u32,
    /// Input IDs that have timed out (circuit breaker open)
    #[serde(default)]
    pub broken_inputs: Vec<String>,
    /// Current health status
    #[serde(default)]
    pub status: NodeStatus,
    /// Number of pending messages in the node's input queue
    #[serde(default)]
    pub pending_messages: u64,
}

/// Per-dataflow network I/O counters for cross-daemon Zenoh traffic.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct NetworkMetrics {
    pub bytes_sent: u64,
    pub bytes_received: u64,
    pub messages_sent: u64,
    pub messages_received: u64,
    #[serde(default)]
    pub publish_failures: u64,
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
    DestroyResult {
        result: Result<(), String>,
        #[serde(skip)]
        notify: Option<tokio::sync::oneshot::Sender<()>>,
    },
    Logs(Result<Vec<u8>, String>),
    RestartNodeResult(Result<(), String>),
    StopNodeResult(Result<(), String>),
    SetParamResult(Result<(), String>),
}
