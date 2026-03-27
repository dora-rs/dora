use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
    time::Duration,
};

use crate::{
    BuildId, DataflowId, SessionId,
    common::{DaemonId, GitSource},
    descriptor::{Descriptor, ResolvedNode},
    id::{DataId, NodeId, OperatorId},
};

// ---------------------------------------------------------------------------
// State catch-up types (incremental replay for reconnecting daemons)
// ---------------------------------------------------------------------------

/// A single state mutation that a reconnecting daemon may have missed.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StateCatchUpEntry {
    pub sequence: u64,
    pub operation: StateCatchUpOperation,
}

/// The kind of state mutation recorded in the replication log.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum StateCatchUpOperation {
    SetParam {
        node_id: NodeId,
        key: String,
        value: serde_json::Value,
    },
    DeleteParam {
        node_id: NodeId,
        key: String,
    },
}

pub use crate::common::Timestamped;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum RegisterResult {
    Ok {
        /// unique ID assigned by the coordinator
        daemon_id: DaemonId,
    },
    Err(String),
}

impl RegisterResult {
    pub fn to_result(self) -> eyre::Result<DaemonId> {
        match self {
            RegisterResult::Ok { daemon_id } => Ok(daemon_id),
            RegisterResult::Err(err) => Err(eyre::eyre!(err)),
        }
    }
}

#[allow(clippy::large_enum_variant)]
#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorEvent {
    Build(BuildDataflowNodes),
    Spawn(SpawnDataflowNodes),
    AllNodesReady {
        dataflow_id: DataflowId,
        exited_before_subscribe: Vec<NodeId>,
    },
    StopDataflow {
        dataflow_id: DataflowId,
        grace_duration: Option<Duration>,
        #[serde(default)]
        force: bool,
    },
    ReloadDataflow {
        dataflow_id: DataflowId,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    },
    Logs {
        dataflow_id: DataflowId,
        node_id: NodeId,
        tail: Option<usize>,
    },
    RestartNode {
        dataflow_id: DataflowId,
        node_id: NodeId,
        grace_duration: Option<Duration>,
    },
    StopNode {
        dataflow_id: DataflowId,
        node_id: NodeId,
        grace_duration: Option<Duration>,
    },
    SetParam {
        dataflow_id: DataflowId,
        node_id: NodeId,
        key: String,
        value: serde_json::Value,
    },
    DeleteParam {
        dataflow_id: DataflowId,
        node_id: NodeId,
        key: String,
    },
    Destroy,
    Heartbeat,
    PeerDaemonDisconnected {
        daemon_id: DaemonId,
    },
    // --- Dynamic Topology ---
    /// Add a node to a running dataflow on this daemon.
    AddNode {
        dataflow_id: DataflowId,
        node: crate::descriptor::ResolvedNode,
        uv: bool,
    },
    /// Remove a node from a running dataflow on this daemon.
    RemoveNode {
        dataflow_id: DataflowId,
        node_id: NodeId,
        grace_duration: Option<Duration>,
    },
    /// Add a mapping (connection) in a running dataflow.
    AddMapping {
        dataflow_id: DataflowId,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    },
    /// Remove a mapping (connection) in a running dataflow.
    RemoveMapping {
        dataflow_id: DataflowId,
        source_node: NodeId,
        source_output: DataId,
        target_node: NodeId,
        target_input: DataId,
    },
    /// Incremental state catch-up: replays missed state mutations to a
    /// reconnecting daemon.
    StateCatchUp {
        dataflow_id: DataflowId,
        /// The entries the daemon missed, ordered by sequence number.
        entries: Vec<StateCatchUpEntry>,
    },
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct BuildDataflowNodes {
    pub build_id: BuildId,
    pub session_id: SessionId,
    /// Allows overwriting the base working dir when CLI and daemon are
    /// running on the same machine.
    ///
    /// Must not be used for multi-machine dataflows.
    ///
    /// Note that nodes with git sources still use a subdirectory of
    /// the base working dir.
    pub local_working_dir: Option<PathBuf>,
    pub git_sources: BTreeMap<NodeId, GitSource>,
    pub prev_git_sources: BTreeMap<NodeId, GitSource>,
    pub dataflow_descriptor: Descriptor,
    pub nodes_on_machine: BTreeSet<NodeId>,
    pub uv: bool,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct SpawnDataflowNodes {
    pub build_id: Option<BuildId>,
    pub session_id: SessionId,
    pub dataflow_id: DataflowId,
    /// Allows overwriting the base working dir when CLI and daemon are
    /// running on the same machine.
    ///
    /// Must not be used for multi-machine dataflows.
    ///
    /// Note that nodes with git sources still use a subdirectory of
    /// the base working dir.
    pub local_working_dir: Option<PathBuf>,
    pub nodes: BTreeMap<NodeId, ResolvedNode>,
    pub dataflow_descriptor: Descriptor,
    pub spawn_nodes: BTreeSet<NodeId>,
    pub uv: bool,
    pub write_events_to: Option<PathBuf>,
    /// Base URL for downloading artifacts from the coordinator (HTTP distribution mode).
    /// When set, daemons can pull binaries from `{artifact_base_url}/{build_id}/{node_id}`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub artifact_base_url: Option<String>,
}
