use std::{collections::BTreeMap, path::PathBuf};

use communication_layer_request_reply::Request;

use crate::{BuildId, SessionId, common::GitSource, descriptor::Descriptor, id::NodeId};

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
pub struct DataflowBuildTriggered {
    pub build_id: BuildId,
}

impl Request for BuildRequest {
    type Response = Result<DataflowBuildTriggered, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct WaitForBuild {
    pub build_id: BuildId,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowBuildFinished {
    pub build_id: BuildId,
    pub result: Result<(), String>,
}

impl Request for WaitForBuild {
    type Response = Result<DataflowBuildFinished, String>;
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct BuildLogSubscribe {
    pub build_id: BuildId,
    pub level: log::LevelFilter,
}

impl Request for BuildLogSubscribe {
    type Response = Result<(), String>;
}
