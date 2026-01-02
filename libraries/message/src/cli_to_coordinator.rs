use std::{collections::BTreeMap, path::PathBuf, time::Duration};

use serde::{Deserialize, Serialize};
use uuid::Uuid;

use crate::{
    BuildId, SessionId,
    common::GitSource,
    descriptor::Descriptor,
    id::{NodeId, OperatorId},
};

dora_schema_macro::dora_schema! {
    Cli => Coordinator:

    build: BuildReq => BuildResp;
//     wait_for_build: ControlRequestWaitForBuild => ControlRequestWaitForBuildReply;
//     start: ControlRequestStart => ControlRequestStartReply;
//     wait_for_spawn: ControlRequestWaitForSpawn => ControlRequestWaitForSpawnReply;
//     reload: ControlRequestReload => ControlRequestReloadReply;
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
