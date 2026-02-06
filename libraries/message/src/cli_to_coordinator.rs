use std::{collections::BTreeMap, path::PathBuf, time::Duration};

use uuid::Uuid;

use crate::{
    BuildId, SessionId,
    common::GitSource,
    descriptor::Descriptor,
    id::{NodeId, OperatorId},
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct BuildRequest {
    pub session_id: SessionId,
    pub dataflow: Descriptor,
    pub git_sources: BTreeMap<NodeId, GitSource>,
    pub prev_git_sources: BTreeMap<NodeId, GitSource>,
    /// Allows overwriting the base working dir when CLI and daemon are
    /// running on the same machine.
    ///
    /// Must not be used for multi-machine dataflows.
    ///
    /// Note that nodes with git sources still use a subdirectory of
    /// the base working dir.
    pub local_working_dir: Option<PathBuf>,
    pub uv: bool,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct StartRequest {
    pub build_id: Option<BuildId>,
    pub session_id: SessionId,
    pub dataflow: Descriptor,
    pub name: Option<String>,
    /// Allows overwriting the base working dir when CLI and daemon are
    /// running on the same machine.
    ///
    /// Must not be used for multi-machine dataflows.
    ///
    /// Note that nodes with git sources still use a subdirectory of
    /// the base working dir.
    pub local_working_dir: Option<PathBuf>,
    pub uv: bool,
    pub write_events_to: Option<PathBuf>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequest {
    Build(BuildRequest),
    WaitForBuild {
        build_id: BuildId,
    },
    Start(StartRequest),
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
        #[serde(default)]
        force: bool,
    },
    StopByName {
        name: String,
        grace_duration: Option<Duration>,
        #[serde(default)]
        force: bool,
    },
    Logs {
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
        tail: Option<usize>,
    },
    Destroy,
    List,
    Info {
        dataflow_uuid: Uuid,
    },
    DaemonConnected,
    ConnectedMachines,
    LogSubscribe {
        dataflow_id: Uuid,
        level: log::LevelFilter,
    },
    BuildLogSubscribe {
        build_id: BuildId,
        level: log::LevelFilter,
    },
    CliAndDefaultDaemonOnSameMachine,
    GetNodeInfo,
}

#[tarpc::service]
pub trait CliControl {
    async fn build(request: BuildRequest) -> eyre::Result<BuildId>;
    async fn wait_for_build(build_id: BuildId) -> eyre::Result<()>;
    async fn start(request: StartRequest) -> eyre::Result<Uuid>;
    async fn wait_for_spawn(dataflow_id: Uuid) -> eyre::Result<()>;
}
