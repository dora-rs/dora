use std::{collections::BTreeMap, net::IpAddr};

use uuid::Uuid;

pub use crate::common::{LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus};
use crate::{BuildId, common::DaemonId, descriptor::Descriptor, id::NodeId};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequestReply {
    Error(String),
    CoordinatorStopped,
    DataflowBuildTriggered {
        build_id: BuildId,
    },
    DataflowBuildFinished {
        build_id: BuildId,
        result: Result<(), String>,
    },
    DataflowStartTriggered {
        uuid: Uuid,
    },
    DataflowSpawned {
        uuid: Uuid,
    },
    DataflowReloaded {
        uuid: Uuid,
    },
    DataflowStopped {
        uuid: Uuid,
        result: DataflowResult,
    },
    DataflowRestarted {
        old_uuid: Uuid,
        new_uuid: Uuid,
    },
    DataflowList(DataflowList),
    DataflowInfo {
        uuid: Uuid,
        name: Option<String>,
        descriptor: Descriptor,
    },
    DestroyOk,
    DaemonConnected(bool),
    ConnectedDaemons(Vec<DaemonInfo>),
    Logs(Vec<u8>),
    CliAndDefaultDaemonIps {
        default_daemon: Option<IpAddr>,
        cli: Option<IpAddr>,
    },
    NodeInfoList(Vec<NodeInfo>),
    TopicSubscribed {
        subscription_id: Uuid,
    },
    TraceList(Vec<TraceSummary>),
    TraceSpans(Vec<TraceSpan>),
    NodeRestarted {
        dataflow_id: Uuid,
        node_id: NodeId,
    },
    NodeStopped {
        dataflow_id: Uuid,
        node_id: NodeId,
    },
    TopicPublished,
    // --- Dynamic Topology ---
    NodeAdded {
        dataflow_id: Uuid,
        node_id: NodeId,
    },
    NodeRemoved {
        dataflow_id: Uuid,
        node_id: NodeId,
    },
    MappingAdded,
    MappingRemoved,
    ParamList {
        params: Vec<(String, serde_json::Value)>,
    },
    ParamValue {
        key: String,
        value: serde_json::Value,
    },
    ParamSet,
    ParamDeleted,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct NodeInfo {
    pub dataflow_id: Uuid,
    pub dataflow_name: Option<String>,
    pub node_id: NodeId,
    pub daemon_id: DaemonId,
    pub metrics: Option<NodeMetricsInfo>,
    /// Per-dataflow cross-daemon network I/O counters (shared across nodes in same dataflow)
    #[serde(default)]
    pub network: Option<crate::daemon_to_coordinator::NetworkMetrics>,
}

/// Resource metrics for a node (from daemon)
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct NodeMetricsInfo {
    /// Process ID
    pub pid: u32,
    /// CPU usage percentage (0-100 per core)
    pub cpu_usage: f32,
    /// Memory usage in megabytes
    pub memory_mb: f64,
    /// Disk read MB/s (if available)
    pub disk_read_mb_s: Option<f64>,
    /// Disk write MB/s (if available)
    pub disk_write_mb_s: Option<f64>,
    /// Number of times this node has been restarted
    #[serde(default)]
    pub restart_count: u32,
    /// Input IDs that have timed out (circuit breaker open)
    #[serde(default)]
    pub broken_inputs: Vec<String>,
    /// Current health status
    #[serde(default)]
    pub status: crate::daemon_to_coordinator::NodeStatus,
    /// Number of pending messages in the node's input queue
    #[serde(default)]
    pub pending_messages: u64,
}

/// Health information about a connected daemon.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DaemonInfo {
    pub daemon_id: DaemonId,
    pub last_heartbeat_ago_ms: u64,
    /// Fault tolerance stats from the daemon (if available).
    #[serde(default)]
    pub ft_stats: Option<crate::daemon_to_coordinator::FaultToleranceSnapshot>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct TraceSummary {
    pub trace_id: String,
    pub root_span_name: String,
    pub span_count: usize,
    pub start_time: u64,
    pub total_duration_us: u64,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct TraceSpan {
    pub trace_id: String,
    pub span_id: u64,
    pub parent_span_id: Option<u64>,
    pub name: String,
    pub target: String,
    pub level: String,
    pub start_time: u64,
    pub duration_us: u64,
    pub fields: Vec<(String, String)>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowResult {
    pub uuid: Uuid,
    pub timestamp: uhlc::Timestamp,
    pub node_results: BTreeMap<NodeId, Result<(), NodeError>>,
}

impl DataflowResult {
    pub fn ok_empty(uuid: Uuid, timestamp: uhlc::Timestamp) -> Self {
        Self {
            uuid,
            timestamp,
            node_results: Default::default(),
        }
    }

    pub fn is_ok(&self) -> bool {
        self.node_results.values().all(|r| r.is_ok())
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowList(pub Vec<DataflowListEntry>);

impl DataflowList {
    pub fn get_active(&self) -> Vec<DataflowIdAndName> {
        self.0
            .iter()
            .filter(|d| d.status == DataflowStatus::Running)
            .map(|d| d.id.clone())
            .collect()
    }
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowListEntry {
    pub id: DataflowIdAndName,
    pub status: DataflowStatus,
}

#[derive(Debug, Clone, Copy, serde::Deserialize, serde::Serialize, PartialEq, Eq)]
pub enum DataflowStatus {
    Running,
    Finished,
    Failed,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct DataflowIdAndName {
    pub uuid: Uuid,
    pub name: Option<String>,
}

impl std::fmt::Display for DataflowIdAndName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if let Some(name) = &self.name {
            write!(f, "[{name}] {}", self.uuid)
        } else {
            write!(f, "[<unnamed>] {}", self.uuid)
        }
    }
}
