use std::{collections::BTreeMap, path::PathBuf, time::Duration};

use uuid::Uuid;

use crate::{
    common::GitSource,
    descriptor::Descriptor,
    id::{NodeId, OperatorId},
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequest {
    Build {
        dataflow: Descriptor,
        git_sources: BTreeMap<NodeId, GitSource>,
        prev_git_sources: BTreeMap<NodeId, GitSource>,
        // TODO: remove this once we figure out deploying of node/operator
        // binaries from CLI to coordinator/daemon
        local_working_dir: PathBuf,
        uv: bool,
    },
    Start {
        build_id: Option<Uuid>,
        dataflow: Descriptor,
        name: Option<String>,
        // TODO: remove this once we figure out deploying of node/operator
        // binaries from CLI to coordinator/daemon
        local_working_dir: PathBuf,
        uv: bool,
    },
    WaitForSpawn {
        dataflow_id: Uuid,
    },
    Reload {
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    },
    Check {
        dataflow_uuid: Uuid,
    },
    Stop {
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
    },
    StopByName {
        name: String,
        grace_duration: Option<Duration>,
    },
    Logs {
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
    },
    Destroy,
    List,
    DaemonConnected,
    ConnectedMachines,
    LogSubscribe {
        dataflow_id: Uuid,
        level: log::LevelFilter,
    },
}
