use std::collections::BTreeMap;

pub use crate::common::{
    DataMessage, LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus, Timestamped,
};
use crate::{
    BuildId, DataflowId, common::DaemonId, current_crate_version, id::NodeId, versions_compatible,
};

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register(DaemonRegisterRequest),
    Event {
        daemon_id: DaemonId,
        event: DaemonEvent,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct DaemonRegisterRequest {
    dora_version: semver::Version,
    pub machine_id: Option<String>,
}

impl DaemonRegisterRequest {
    pub fn new(machine_id: Option<String>) -> Self {
        Self {
            dora_version: current_crate_version(),
            machine_id,
        }
    }

    pub fn check_version(&self) -> Result<(), String> {
        let crate_version = current_crate_version();
        let specified_version = &self.dora_version;

        if versions_compatible(&crate_version, specified_version)? {
            Ok(())
        } else {
            Err(format!(
                "version mismatch: message format v{} is not compatible \
                with expected message format v{crate_version}",
                self.dora_version
            ))
        }
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DaemonEvent {
    BuildResult {
        build_id: BuildId,
        result: Result<(), String>,
    },
    SpawnResult {
        dataflow_id: DataflowId,
        result: Result<(), String>,
    },
    AllNodesReady {
        dataflow_id: DataflowId,
        exited_before_subscribe: Vec<NodeId>,
    },
    AllNodesFinished {
        dataflow_id: DataflowId,
        result: DataflowDaemonResult,
    },
    Heartbeat,
    Log(LogMessage),
    Exit,
}

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct DataflowDaemonResult {
    pub timestamp: uhlc::Timestamp,
    pub node_results: BTreeMap<NodeId, Result<(), NodeError>>,
}

impl DataflowDaemonResult {
    pub fn is_ok(&self) -> bool {
        self.node_results.values().all(|r| r.is_ok())
    }
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorReply {
    TriggerBuildResult(Result<(), String>),
    TriggerSpawnResult(Result<(), String>),
    ReloadResult(Result<(), String>),
    StopResult(Result<(), String>),
    DestroyResult {
        result: Result<(), String>,
        #[serde(skip)]
        notify: Option<tokio::sync::oneshot::Sender<()>>,
    },
    Logs(Result<Vec<u8>, String>),
}
