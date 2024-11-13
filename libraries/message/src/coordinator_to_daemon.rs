use std::{collections::BTreeMap, net::SocketAddr, path::PathBuf, time::Duration};

use crate::{
    descriptor::{Descriptor, ResolvedNode},
    id::{NodeId, OperatorId},
    DataflowId,
};

pub use crate::common::Timestamped;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum RegisterResult {
    Ok,
    Err(String),
}

impl RegisterResult {
    pub fn to_result(self) -> eyre::Result<()> {
        match self {
            RegisterResult::Ok => Ok(()),
            RegisterResult::Err(err) => Err(eyre::eyre!(err)),
        }
    }
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorEvent {
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
pub struct SpawnDataflowNodes {
    pub dataflow_id: DataflowId,
    pub working_dir: PathBuf,
    pub nodes: Vec<ResolvedNode>,
    pub machine_listen_ports: BTreeMap<String, SocketAddr>,
    pub dataflow_descriptor: Descriptor,
}
