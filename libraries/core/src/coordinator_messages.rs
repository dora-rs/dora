use crate::{config::NodeId, daemon_messages::DataflowId, topics::DataflowDaemonResult};
use eyre::eyre;
pub use log::Level;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register {
        dora_version: String,
        machine_id: String,
        listen_port: u16,
    },
    Event {
        machine_id: String,
        event: DaemonEvent,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
#[must_use]
pub struct LogMessage {
    pub dataflow_id: DataflowId,
    pub node_id: Option<NodeId>,
    pub level: log::Level,
    pub target: Option<String>,
    pub module_path: Option<String>,
    pub file: Option<String>,
    pub line: Option<u32>,
    pub message: String,
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

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum RegisterResult {
    Ok,
    Err(String),
}

impl RegisterResult {
    pub fn to_result(self) -> eyre::Result<()> {
        match self {
            RegisterResult::Ok => Ok(()),
            RegisterResult::Err(err) => Err(eyre!(err)),
        }
    }
}
