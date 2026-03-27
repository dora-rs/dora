use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
    time::Duration,
};

use crate::{
    BuildId, DataflowId, SessionId,
    common::{DaemonId, GitSource},
    descriptor::{Descriptor, ResolvedNode},
    id::{NodeId, OperatorId},
};

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
    /// When true, hot-reload file watching is enabled for this dataflow.
    #[serde(default)]
    pub hot_reload: bool,
    /// Path to the dataflow YAML file, used for hot-reload YAML watching.
    #[serde(default)]
    pub dataflow_path: Option<PathBuf>,
}

type DaemonResult<T> = std::result::Result<T, String>;

#[tarpc::service]
pub trait DaemonControl {
    /// Trigger a dataflow build on this daemon.
    async fn build(request: BuildDataflowNodes) -> DaemonResult<()>;
    /// Trigger spawning dataflow nodes on this daemon.
    async fn spawn(request: SpawnDataflowNodes) -> DaemonResult<()>;
    /// Notify the daemon that all nodes across all daemons are ready.
    async fn all_nodes_ready(dataflow_id: DataflowId, exited_before_subscribe: Vec<NodeId>);
    /// Stop a running dataflow on this daemon.
    async fn stop_dataflow(
        dataflow_id: DataflowId,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> DaemonResult<()>;
    /// Reload a specific node/operator in a running dataflow.
    async fn reload_dataflow(
        dataflow_id: DataflowId,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> DaemonResult<()>;
    /// Retrieve log file contents for a specific node.
    async fn logs(
        dataflow_id: DataflowId,
        node_id: NodeId,
        tail: Option<usize>,
    ) -> DaemonResult<crate::common::LogsResponse>;
    /// Stop a single node within a running dataflow (for hot-reload).
    async fn stop_node(dataflow_id: DataflowId, node_id: NodeId) -> DaemonResult<()>;
    /// Dynamically spawn a node into a running dataflow (for hot-reload).
    async fn dynamic_spawn(
        dataflow_id: DataflowId,
        node_id: NodeId,
        node: Box<ResolvedNode>,
        dataflow_descriptor: Box<Descriptor>,
    ) -> DaemonResult<()>;
    /// Restart a node with new configuration (for hot-reload).
    async fn restart_node(
        dataflow_id: DataflowId,
        node_id: NodeId,
        new_node: Box<ResolvedNode>,
        dataflow_descriptor: Box<Descriptor>,
    ) -> DaemonResult<()>;
    /// Destroy the daemon (shut it down).
    async fn destroy() -> DaemonResult<()>;
    /// Heartbeat check.
    async fn heartbeat();
    /// Return the daemon's version info for compatibility checking.
    async fn get_version() -> DaemonVersionInfo;
}

/// Version info returned by the daemon's `get_version` RPC method.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DaemonVersionInfo {
    /// The daemon's own crate version (e.g. "0.4.1")
    pub daemon_version: String,
    /// The dora-message crate version used by the daemon (e.g. "0.7.0")
    pub message_format_version: String,
}
