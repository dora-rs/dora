use std::{collections::BTreeMap, path::PathBuf};

use crate::{
    config::{DataId, NodeId, NodeRunConfig},
    descriptor::{self, OperatorDefinition},
};
use dora_message::Metadata;
use uuid::Uuid;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct NodeConfig {
    pub dataflow_id: DataflowId,
    pub node_id: NodeId,
    pub run_config: NodeRunConfig,
    pub daemon_control_region_id: SharedMemoryId,
    pub daemon_events_region_id: SharedMemoryId,
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct RuntimeConfig {
    pub node: NodeConfig,
    pub operators: Vec<OperatorDefinition>,
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DaemonRequest {
    Register {
        dataflow_id: DataflowId,
        node_id: NodeId,
    },
    Subscribe,
    PrepareOutputMessage {
        output_id: DataId,
        metadata: Metadata<'static>,
        data_len: usize,
    },
    SendPreparedMessage {
        id: SharedMemoryId,
    },
    SendEmptyMessage {
        output_id: DataId,
        metadata: Metadata<'static>,
    },
    CloseOutputs(Vec<DataId>),
    Stopped,
    NextEvent {
        drop_tokens: Vec<DropToken>,
    },
}

type SharedMemoryId = String;

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum DaemonReply {
    Result(Result<(), String>),
    PreparedMessage { shared_memory_id: SharedMemoryId },
    Closed,
    NodeEvent(NodeEvent),
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub enum NodeEvent {
    Stop,
    Input {
        id: DataId,
        metadata: Metadata<'static>,
        data: Option<InputData>,
    },
    InputClosed {
        id: DataId,
    },
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct DropEvent {
    pub tokens: Vec<DropToken>,
}

#[derive(
    Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct DropToken(Uuid);

impl DropToken {
    pub fn generate() -> Self {
        Self(Uuid::new_v4())
    }
}

#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct InputData {
    pub shared_memory_id: SharedMemoryId,
    pub len: usize,
    pub drop_token: DropToken,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorEvent {
    Spawn(SpawnDataflowNodes),
    StopDataflow { dataflow_id: DataflowId },
    Destroy,
    Watchdog,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum DaemonCoordinatorReply {
    SpawnResult(Result<(), String>),
    StopResult(Result<(), String>),
    DestroyResult(Result<(), String>),
    WatchdogAck,
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
