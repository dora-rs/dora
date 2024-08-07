use std::collections::BTreeMap;

use dora_core::{config::NodeId, uhlc};

pub use crate::common::{
    DataMessage, LogLevel, LogMessage, NodeError, NodeErrorCause, NodeExitStatus, Timestamped,
};
use crate::DataflowId;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register(DaemonRegisterRequest),
    Event {
        machine_id: String,
        event: DaemonEvent,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct DaemonRegisterRequest {
    dora_version: String,
    pub machine_id: String,
    pub listen_port: u16,
}

impl DaemonRegisterRequest {
    pub fn new(machine_id: String, listen_port: u16) -> Self {
        Self {
            dora_version: env!("CARGO_PKG_VERSION").to_owned(),
            machine_id,
            listen_port,
        }
    }

    pub fn check_version(&self) -> Result<(), String> {
        let crate_version = env!("CARGO_PKG_VERSION");
        if self.dora_version == crate_version {
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
    SpawnResult(Result<(), String>),
    ReloadResult(Result<(), String>),
    StopResult(Result<(), String>),
    DestroyResult {
        result: Result<(), String>,
        #[serde(skip)]
        notify: Option<tokio::sync::oneshot::Sender<()>>,
    },
    Logs(Result<Vec<u8>, String>),
}
