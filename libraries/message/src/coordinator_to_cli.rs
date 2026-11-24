use std::{collections::BTreeMap, net::IpAddr};

use uuid::Uuid;

pub use crate::common::{LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus};
use crate::{
    BuildId,
    common::DaemonId,
    descriptor::Descriptor,
    id::{DataId, NodeId},
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequestReply {
    Error(String),
    CoordinatorStopped,
    /// Response to [`ControlRequest::Hello`][crate::cli_to_coordinator::ControlRequest::Hello].
    /// Carries the coordinator's own dora crate version so the CLI can
    /// display it on version mismatches and in debug output
    /// (dora-rs/adora#151).
    HelloOk {
        dora_version: semver::Version,
    },
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
    /// Response to [`ControlRequest::Clean`][crate::cli_to_coordinator::ControlRequest::Clean].
    ///
    /// `cleaned` lists dataflows the coordinator successfully removed
    /// from both in-memory state and the persisted store (with the
    /// cascade-delete of associated `dora param` rows). `failed` lists
    /// dataflows that were eligible but whose persisted-store delete
    /// errored: their in-memory entries are preserved so a later
    /// `dora clean` can retry. The CLI needs both lists to tell
    /// "nothing eligible" apart from "all candidates failed to clean".
    CleanResult {
        cleaned: DataflowList,
        failed: Vec<CleanFailure>,
    },
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
    /// Constructed through [`ControlRequestReply::topic_subscribed`] or
    /// [`ControlRequestReply::topic_subscribed_direct`], not by literal: the
    /// variant is `#[non_exhaustive]` so that a field added here is a minor
    /// change rather than a 2.0 (see "Stability scope at 1.0" in
    /// `docs/api-rust.md`).
    #[non_exhaustive]
    TopicSubscribed {
        /// The relayed stream's id, which its binary frames are prefixed
        /// with. Nil when `zenoh_endpoints` is set: nothing is relayed then.
        subscription_id: Uuid,
        /// Binary-frame encoding the coordinator will send, see
        /// [`TOPIC_DATA_PROTOCOL_VERSION`](crate::TOPIC_DATA_PROTOCOL_VERSION).
        ///
        /// `None` means the coordinator predates the handshake and therefore
        /// sends bincode frames; the client rejects the subscription rather
        /// than misparse them as postcard.
        #[serde(default)]
        protocol_version: Option<u16>,
        /// Set in answer to a `direct` subscribe request: the zenoh endpoints
        /// of the daemons running the requested outputs, for the client to
        /// subscribe to them itself. The coordinator relays nothing for such
        /// a subscription.
        ///
        /// `None` means the topics are relayed under `subscription_id`: the
        /// request was not `direct`, a daemon had no endpoint this client can
        /// dial, or the coordinator predates the field.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        zenoh_endpoints: Option<Vec<String>>,
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
    NodeReplaced {
        dataflow_id: Uuid,
        node_id: NodeId,
    },
    MappingAdded {
        dataflow_id: Uuid,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    },
    MappingRemoved {
        dataflow_id: Uuid,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    },
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

impl ControlRequestReply {
    /// The reply to a relayed topic subscription, stamped with the
    /// binary-frame encoding this crate speaks
    /// ([`TOPIC_DATA_PROTOCOL_VERSION`](crate::TOPIC_DATA_PROTOCOL_VERSION)).
    pub fn topic_subscribed(subscription_id: Uuid) -> Self {
        Self::TopicSubscribed {
            subscription_id,
            protocol_version: Some(crate::TOPIC_DATA_PROTOCOL_VERSION),
            zenoh_endpoints: None,
        }
    }

    /// The reply to a `direct` topic subscription: the zenoh endpoints to
    /// read the topics from. Nothing is relayed, so the subscription id is nil.
    pub fn topic_subscribed_direct(zenoh_endpoints: Vec<String>) -> Self {
        Self::TopicSubscribed {
            subscription_id: Uuid::nil(),
            protocol_version: Some(crate::TOPIC_DATA_PROTOCOL_VERSION),
            zenoh_endpoints: Some(zenoh_endpoints),
        }
    }
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

/// A dataflow that `dora clean` failed to remove from the persisted
/// store. Reported alongside the successful list so the CLI can show
/// the user what didn't get cleaned and exit non-zero.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct CleanFailure {
    pub id: DataflowIdAndName,
    /// Human-readable error from the persisted store (e.g. an
    /// underlying redb / I/O failure). The in-memory entry is
    /// preserved so a later `dora clean` can retry.
    pub error: String,
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A relayed reply keeps the exact shape older CLIs expect: the new field
    /// is left out rather than sent as `null`.
    #[test]
    fn relayed_topic_subscribed_has_no_endpoints_field() {
        let id = Uuid::from_u128(7);
        let json = serde_json::to_value(ControlRequestReply::topic_subscribed(id)).unwrap();
        assert_eq!(
            json,
            serde_json::json!({"TopicSubscribed": {
                "subscription_id": id,
                "protocol_version": crate::TOPIC_DATA_PROTOCOL_VERSION,
            }})
        );
    }

    /// A reply from a coordinator that predates `zenoh_endpoints` decodes as
    /// a relayed subscription, and a direct reply round-trips its endpoints.
    #[test]
    fn topic_subscribed_endpoints_decode_from_older_and_newer_coordinators() {
        let older = r#"{"TopicSubscribed":{"subscription_id":"00000000-0000-0000-0000-000000000007","protocol_version":2}}"#;
        match serde_json::from_str(older).unwrap() {
            ControlRequestReply::TopicSubscribed {
                zenoh_endpoints, ..
            } => assert_eq!(zenoh_endpoints, None),
            other => panic!("expected TopicSubscribed, got {other:?}"),
        }

        let endpoints = vec!["tcp/10.0.0.2:5456".to_string()];
        let json = serde_json::to_string(&ControlRequestReply::topic_subscribed_direct(
            endpoints.clone(),
        ))
        .unwrap();
        match serde_json::from_str(&json).unwrap() {
            ControlRequestReply::TopicSubscribed {
                subscription_id,
                zenoh_endpoints,
                ..
            } => {
                assert_eq!(subscription_id, Uuid::nil());
                assert_eq!(zenoh_endpoints, Some(endpoints));
            }
            other => panic!("expected TopicSubscribed, got {other:?}"),
        }
    }
}
