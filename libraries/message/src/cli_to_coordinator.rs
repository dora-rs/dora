use std::{collections::BTreeMap, path::PathBuf, time::Duration};

use uuid::Uuid;

use std::collections::BTreeSet;

use crate::{
    BuildId, SessionId,
    common::{DaemonId, GitSource},
    coordinator_to_cli::{
        CheckDataflowReply, DataflowInfo, DataflowList, NodeInfo, StopDataflowReply, VersionInfo,
    },
    descriptor::Descriptor,
    id::{NodeId, OperatorId},
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct BuildRequest {
    /// Client-generated build ID. When provided, the CLI can subscribe
    /// to the log topic *before* sending this RPC, avoiding a race
    /// where early log messages are missed.
    ///
    /// When `None`, the coordinator generates the ID (backwards compat).
    #[serde(default)]
    pub build_id: Option<BuildId>,
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
    /// Client-generated dataflow ID. When provided, the CLI can subscribe
    /// to the log topic *before* sending this RPC, avoiding a race
    /// where early log messages are missed.
    ///
    /// When `None`, the coordinator generates the ID (backwards compat).
    #[serde(default)]
    pub dataflow_id: Option<Uuid>,
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

type Result<T> = std::result::Result<T, String>;

#[tarpc::service]
pub trait CoordinatorControl {
    async fn build(request: BuildRequest) -> Result<BuildId>;
    async fn wait_for_build(build_id: BuildId) -> Result<()>;
    async fn start(request: StartRequest) -> Result<Uuid>;
    async fn wait_for_spawn(dataflow_id: Uuid) -> Result<()>;
    async fn reload(
        dataflow_id: Uuid,
        node_id: NodeId,
        operator_id: Option<OperatorId>,
    ) -> Result<Uuid>;
    async fn check(dataflow_uuid: Uuid) -> Result<CheckDataflowReply>;
    async fn stop(
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<StopDataflowReply>;
    async fn stop_by_name(
        name: String,
        grace_duration: Option<Duration>,
        force: bool,
    ) -> Result<StopDataflowReply>;
    async fn logs(
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
        tail: Option<usize>,
    ) -> Result<Vec<u8>>;
    async fn destroy() -> Result<()>;
    async fn list() -> Result<DataflowList>;
    async fn info(dataflow_uuid: Uuid) -> Result<DataflowInfo>;
    async fn daemon_connected() -> Result<bool>;
    async fn connected_machines() -> Result<BTreeSet<DaemonId>>;
    async fn cli_and_default_daemon_on_same_machine(machine_uid: Option<String>) -> Result<bool>;
    async fn get_node_info() -> Result<Vec<NodeInfo>>;
    async fn get_version() -> VersionInfo;
}
