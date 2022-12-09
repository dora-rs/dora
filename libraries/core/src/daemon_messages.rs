use crate::config::{DataId, NodeId, NodeRunConfig};
use dora_message::Metadata;
use eyre::Context;
use shared_memory::{Shmem, ShmemConf};

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct NodeConfig {
    pub node_id: NodeId,
    pub run_config: NodeRunConfig,
    pub daemon_port: u16,
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum ControlRequest {
    Register { node_id: NodeId },
    Subscribe { node_id: NodeId },
    PrepareOutputMessage { output_id: DataId, len: usize },
    SendOutMessage { id: SharedMemoryId },
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
        data: InputData,
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
