use eyre::eyre;

use crate::daemon_messages::DataflowId;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum CoordinatorRequest {
    Register {
        machine_id: String,
        dora_version: String,
    },
    Event {
        machine_id: String,
        event: DaemonEvent,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DaemonEvent {
    AllNodesFinished {
        dataflow_id: DataflowId,
        result: Result<(), String>,
    },
    Watchdog,
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

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct WatchdogAck;
