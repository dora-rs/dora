use std::collections::BTreeSet;

use dora_message::Metadata;
use eyre::eyre;

use crate::{
    config::{DataId, NodeId},
    daemon_messages::DataflowId,
};

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
    Output {
        source_machine: String,
        dataflow_id: DataflowId,
        source_node: NodeId,
        output_id: DataId,

        metadata: Metadata<'static>,
        data: Option<Vec<u8>>,

        target_machines: BTreeSet<String>,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DaemonEvent {
    AllNodesReady {
        dataflow_id: DataflowId,
    },
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
