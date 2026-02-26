use std::collections::BTreeMap;

use uuid::Uuid;

pub use crate::common::{LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus};
use crate::{common::DaemonId, descriptor::Descriptor, id::NodeId};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct NodeInfo {
    pub dataflow_id: Uuid,
    pub dataflow_name: Option<String>,
    pub node_id: NodeId,
    pub daemon_id: DaemonId,
    pub metrics: Option<NodeMetricsInfo>,
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
    /// Node start time
    pub start_time: Option<uhlc::Timestamp>,
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

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowInfo {
    pub uuid: Uuid,
    pub name: Option<String>,
    pub descriptor: Descriptor,
}

/// Reply for the `check` RPC method.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum CheckDataflowReply {
    Running { uuid: Uuid },
    Stopped { uuid: Uuid, result: DataflowResult },
}

/// Reply for the `stop` and `stop_by_name` RPC methods.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct StopDataflowReply {
    pub uuid: Uuid,
    pub result: DataflowResult,
}

/// Reply for the `get_version` RPC method.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct VersionInfo {
    /// The coordinator's dora crate version (e.g. "0.4.1")
    pub coordinator_version: String,
    /// The dora-message crate version used by the coordinator (e.g. "0.7.0")
    pub message_format_version: String,
}
