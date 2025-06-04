use std::{
    collections::{BTreeMap, BTreeSet},
    path::PathBuf,
    time::Duration,
};

use uuid::Uuid;

use crate::{
    common::{DaemonId, GitSource},
    descriptor::{Descriptor, ResolvedNode},
    id::{NodeId, OperatorId},
    DataflowId,
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
    },
    ReloadDataflow {
        dataflow_id: DataflowId,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    },
    Logs {
        dataflow_id: DataflowId,
        node_id: NodeId,
    },
    Destroy,
    Heartbeat,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct BuildDataflowNodes {
    pub session_id: Uuid,
    pub working_dir: PathBuf,
    pub nodes: BTreeMap<NodeId, ResolvedNode>,
    pub git_sources: BTreeMap<NodeId, GitSource>,
    pub prev_git_sources: BTreeMap<NodeId, GitSource>,
    pub dataflow_descriptor: Descriptor,
    pub nodes_on_machine: BTreeSet<NodeId>,
    pub uv: bool,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct SpawnDataflowNodes {
    pub session_id: Option<Uuid>,
    pub dataflow_id: DataflowId,
    pub working_dir: PathBuf,
    pub nodes: BTreeMap<NodeId, ResolvedNode>,
    pub dataflow_descriptor: Descriptor,
    pub spawn_nodes: BTreeSet<NodeId>,
    pub uv: bool,
}
