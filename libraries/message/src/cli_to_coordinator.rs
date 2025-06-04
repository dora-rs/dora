use std::{collections::BTreeMap, path::PathBuf, time::Duration};

use uuid::Uuid;

use crate::{
    common::GitSource,
    descriptor::Descriptor,
    id::{NodeId, OperatorId},
    SessionId,
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequest {
    Build {
        session_id: SessionId,
        dataflow: Descriptor,
        git_sources: BTreeMap<NodeId, GitSource>,
        prev_git_sources: BTreeMap<NodeId, GitSource>,
        /// Allows overwriting the base working dir when CLI and daemon are
        /// running on the same machine.
        ///
        /// Must not be used for multi-machine dataflows.
        ///
        /// Note that nodes with git sources still use a subdirectory of
        /// the base working dir.
        local_working_dir: Option<PathBuf>,
        uv: bool,
    },
    Start {
        build_id: Option<Uuid>,
        session_id: Uuid,
        dataflow: Descriptor,
        name: Option<String>,
        /// Allows overwriting the base working dir when CLI and daemon are
        /// running on the same machine.
        ///
        /// Must not be used for multi-machine dataflows.
        ///
        /// Note that nodes with git sources still use a subdirectory of
        /// the base working dir.
        local_working_dir: Option<PathBuf>,
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
