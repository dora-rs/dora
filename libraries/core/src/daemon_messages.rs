use std::{collections::BTreeMap, path::PathBuf};

use crate::{
    config::{DataId, NodeId, NodeRunConfig},
    descriptor,
};
use dora_message::Metadata;
use eyre::Context;
use shared_memory::{Shmem, ShmemConf};
use uuid::Uuid;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct NodeConfig {
    pub dataflow_id: DataflowId,
    pub node_id: NodeId,
    pub run_config: NodeRunConfig,
    pub daemon_port: u16,
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum ControlRequest {
    Register {
        dataflow_id: DataflowId,
        node_id: NodeId,
    },
    Subscribe {
        dataflow_id: DataflowId,
        node_id: NodeId,
    },
    PrepareOutputMessage {
        output_id: DataId,
        metadata: Metadata<'static>,
        data_len: usize,
    },
    SendOutMessage {
        id: SharedMemoryId,
    },
    Stopped,
}

type SharedMemoryId = String;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum ControlReply {
    Result(Result<(), String>),
    PreparedMessage { shared_memory_id: SharedMemoryId },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum NodeEvent {
    Stop,
    Input {
        id: DataId,
        metadata: Metadata<'static>,
        data: Option<InputData>,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct InputData {
    shared_memory_id: SharedMemoryId,
}

impl InputData {
    pub unsafe fn new(shared_memory_id: SharedMemoryId) -> Self {
        Self { shared_memory_id }
    }

    pub fn map(self) -> eyre::Result<MappedInputData> {
        let memory = ShmemConf::new()
            .os_id(self.shared_memory_id)
            .open()
            .wrap_err("failed to map shared memory input")?;
        Ok(MappedInputData { memory })
    }
}

pub struct MappedInputData {
    memory: Shmem,
}

impl std::ops::Deref for MappedInputData {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        unsafe { self.memory.as_slice() }
    }
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorEvent {
    Spawn(SpawnDataflowNodes),
}

pub type DataflowId = Uuid;

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct SpawnDataflowNodes {
    pub dataflow_id: DataflowId,
    pub nodes: BTreeMap<NodeId, SpawnNodeParams>,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub struct SpawnNodeParams {
    pub node_id: NodeId,
    pub node: descriptor::CustomNode,
    pub working_dir: PathBuf,
}
