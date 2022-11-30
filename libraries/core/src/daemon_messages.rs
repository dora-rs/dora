use dora_message::Metadata;

use crate::config::{DataId, NodeId};

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum ControlRequest {
    Register { node_id: NodeId },
    Subscribe { node_id: NodeId },
    PrepareOutputMessage { output_id: DataId, len: usize },
    SendOutMessage { id: MessageId },
    Stopped,
}

type MessageId = String;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum ControlReply {
    Result(Result<(), String>),
    PreparedMessage { id: MessageId, data: RawMutInput },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum NodeEvent {
    Stop,
    Input {
        id: DataId,
        metadata: Metadata<'static>,
        data: RawInput, // TODO add lifetime to borrow from inputs channel while RawInput exists
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct RawInput {
    shared_memory_pointer: (), // TODO
    len: usize,
}

impl RawInput {
    pub unsafe fn new(ptr: (), len: usize) -> Self {
        Self {
            shared_memory_pointer: ptr,
            len,
        }
    }

    pub fn get(&self) -> &[u8] {
        &[] // TODO
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct RawMutInput {
    shared_memory_pointer: (), // TODO
    len: usize,
}

impl RawMutInput {
    pub fn get(&self) -> &[u8] {
        &[] // TODO
    }
    pub fn get_mut(&self) -> &mut [u8] {
        &mut [] // TODO
    }
}
