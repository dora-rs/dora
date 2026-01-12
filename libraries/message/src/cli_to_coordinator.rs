use std::{collections::{BTreeMap, BTreeSet}, path::PathBuf, time::Duration};

use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::{
    BuildId, SessionId, common::GitSource, coordinator_to_cli::NodeInfo, descriptor::Descriptor, id::{NodeId, OperatorId}
};

#[dora_schema_macro::dora_schema]
pub trait CliToCoordinator {
    async fn build(req: BuildReq) -> BuildId;
    async fn wait_for_build(build_id: BuildId) -> WaitForBuildResp;
    async fn start(req: StartReq) -> Uuid;
    async fn wait_for_spawn(dataflow_id: Uuid) -> Uuid;
    async fn reload(dataflow_id: Uuid, node_id: NodeId, operator_id: Option<OperatorId>) -> Uuid;
    async fn check(dataflow_uuid: Uuid) -> Uuid;
    async fn stop(
        dataflow_uuid: Uuid,
        grace_duration: Option<Duration>,
        force: bool,
    ) ;
    async fn stop_by_name(
        name: String,
        grace_duration: Option<Duration>,
        force: bool,
    ) ;
    async fn logs(
        uuid: Option<Uuid>,
        name: Option<String>,
        node: String,
        tail: Option<usize>,
    ) -> Vec<u8>;
    async fn destroy();
    async fn list() -> Vec<crate::coordinator_to_cli::DataflowListEntry>;
    async fn info(dataflow_uuid: Uuid);
    async fn daemon_connected() -> bool;
    async fn connected_machines() -> BTreeSet<crate::common::DaemonId>;
    async fn log_subscribe(dataflow_id: Uuid, level: log::LevelFilter);
    async fn build_log_subscribe(build_id: BuildId, level: log::LevelFilter);
    async fn cli_and_default_daemon_on_same_machine() -> CliAndDefaultDaemonIps;
    async fn get_node_info() -> Vec<NodeInfo>;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildReq {
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BuildResp {
    pub build_id: BuildId,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaitForBuildReq {
    pub build_id: BuildId,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaitForBuildResp {
    pub build_id: BuildId,
    pub result: Result<(), String>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StartReq {
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StartResp {
    pub uuid: Uuid,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CliAndDefaultDaemonOnSameMachine;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CliAndDefaultDaemonIps {
    pub default_daemon: Option<std::net::IpAddr>,
    pub cli: Option<std::net::IpAddr>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
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
    WaitForBuild {
        build_id: BuildId,
    },
    Start {
        build_id: Option<BuildId>,
        session_id: SessionId,
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
        write_events_to: Option<PathBuf>,
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
