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
    Destroy,
    Heartbeat,
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
    ) -> DaemonResult<Vec<u8>>;
    /// Destroy the daemon (shut it down).
    async fn destroy() -> DaemonResult<()>;
    /// Heartbeat check.
    async fn heartbeat();
}
