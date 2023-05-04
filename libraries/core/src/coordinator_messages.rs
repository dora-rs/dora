use crate::daemon_messages::DataflowId;
use eyre::eyre;
use std::net::SocketAddr;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register {
        dora_version: String,
        machine_id: String,
        listen_socket: SocketAddr,
    },
    Event {
        machine_id: String,
        event: DaemonEvent,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DaemonEvent {
    AllNodesReady {
        dataflow_id: DataflowId,
        success: bool,
    },
    AllNodesFinished {
        dataflow_id: DataflowId,
        result: Result<(), String>,
    },
    Heartbeat,
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
