use std::{collections::BTreeMap, path::PathBuf, time::Duration};

use uuid::Uuid;

use crate::{
    BuildId, SessionId,
    common::GitSource,
    descriptor::Descriptor,
    id::{NodeId, OperatorId},
};

pub use build::*;

mod build;

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize, derive_more::From)]
pub enum ControlRequest {
    Build(BuildRequest),
    WaitForBuild(WaitForBuild),
    BuildLogSubscribe(BuildLogSubscribe),
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub enum ControlRequestOld {
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

    CliAndDefaultDaemonOnSameMachine,
    GetNodeInfo,
}
