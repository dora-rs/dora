use std::{borrow::Cow, path::PathBuf, time::Duration};

use communication_layer_request_reply::Request;
use uuid::Uuid;

use crate::{
    BuildId, SessionId,
    coordinator_to_cli::{DataflowIdAndName, DataflowList, DataflowResult, NodeInfo},
    descriptor::Descriptor,
    id::{NodeId, OperatorId},
};

pub use build::*;

mod build;

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize, derive_more::From)]
pub enum ControlRequest<'a> {
    Build(Cow<'a, BuildRequest>),
    WaitForBuild(Cow<'a, WaitForBuild>),
    BuildLogSubscribe(Cow<'a, BuildLogSubscribe>),
    Start(Cow<'a, StartRequest>),
    WaitForSpawn(Cow<'a, WaitForSpawn>),
    Reload(Cow<'a, ReloadRequest>),
    Check(Cow<'a, CheckRequest>),
    Stop(Cow<'a, StopRequest>),
    StopByName(Cow<'a, StopByNameRequest>),
    Logs(Cow<'a, LogsRequest>),
    Destroy(Cow<'a, DestroyRequest>),
    List(Cow<'a, ListRequest>),
    Info(Cow<'a, InfoRequest>),
    DaemonConnected(Cow<'a, DaemonConnectedRequest>),
    ConnectedMachines(Cow<'a, ConnectedMachinesRequest>),
    LogSubscribe(Cow<'a, LogSubscribe>),
    CliAndDefaultDaemonOnSameMachine(Cow<'a, CliAndDefaultDaemonOnSameMachineRequest>),
    GetNodeInfo(Cow<'a, GetNodeInfoRequest>),
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
pub struct DataflowStartTriggered {
    pub uuid: Uuid,
}

impl Request for StartRequest {
    type Response = Result<DataflowStartTriggered, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct WaitForSpawn {
    pub dataflow_id: Uuid,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowSpawned {
    pub uuid: Uuid,
}

impl Request for WaitForSpawn {
    type Response = Result<DataflowSpawned, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct ReloadRequest {
    pub dataflow_id: Uuid,
    pub node_id: NodeId,
    pub operator_id: Option<OperatorId>,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowReloaded {
    pub uuid: Uuid,
}

impl Request for ReloadRequest {
    type Response = Result<DataflowReloaded, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct CheckRequest {
    pub dataflow_uuid: Uuid,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum CheckResponse {
    Running,
    Stopped { result: DataflowResult },
}

impl Request for CheckRequest {
    type Response = CheckResponse;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct StopRequest {
    pub dataflow_uuid: Uuid,
    pub grace_duration: Option<Duration>,
    #[serde(default)]
    pub force: bool,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowStopped {
    pub uuid: Uuid,
    pub result: DataflowResult,
}

impl Request for StopRequest {
    type Response = Result<DataflowStopped, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct StopByNameRequest {
    pub name: String,
    pub grace_duration: Option<Duration>,
    #[serde(default)]
    pub force: bool,
}

impl Request for StopByNameRequest {
    type Response = Result<DataflowStopped, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct LogsRequest {
    pub uuid: Option<Uuid>,
    pub name: Option<String>,
    pub node: String,
    pub tail: Option<usize>,
}

impl Request for LogsRequest {
    type Response = Result<Vec<u8>, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DestroyRequest;

impl Request for DestroyRequest {
    type Response = Result<(), String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct ListRequest;

impl Request for ListRequest {
    type Response = DataflowList;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct InfoRequest {
    pub dataflow_uuid: Uuid,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowInfo {
    pub uuid: Uuid,
    pub name: Option<String>,
    pub descriptor: Descriptor,
}

impl Request for InfoRequest {
    type Response = Result<DataflowInfo, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DaemonConnectedRequest;

impl Request for DaemonConnectedRequest {
    type Response = bool;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct ConnectedMachinesRequest;

impl Request for ConnectedMachinesRequest {
    type Response = Vec<DataflowIdAndName>;
}

/// Subscribe to log messages for a dataflow.
///
/// This is a subscription request, so the response is just `Ok(())` to
/// indicate that the subscription was successful. After that, the coordinator
/// will send log messages on the same connection until the dataflow is
/// finished or the connection is closed.
#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct LogSubscribe {
    pub dataflow_id: Uuid,
    pub level: log::LevelFilter,
}

impl Request for LogSubscribe {
    type Response = Result<(), String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct CliAndDefaultDaemonOnSameMachineRequest;

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct CliAndDefaultDaemonIps {
    pub default_daemon: Option<std::net::IpAddr>,
    pub cli: Option<std::net::IpAddr>,
}

impl Request for CliAndDefaultDaemonOnSameMachineRequest {
    type Response = CliAndDefaultDaemonIps;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct GetNodeInfoRequest;

impl Request for GetNodeInfoRequest {
    type Response = Vec<NodeInfo>;
}
