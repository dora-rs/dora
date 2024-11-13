use std::{net::SocketAddr, path::PathBuf};

use crate::{
    config::NodeRunConfig,
    descriptor::{Descriptor, OperatorDefinition},
    id::{DataId, NodeId, OperatorId},
    metadata::Metadata,
    DataflowId,
};

pub use crate::common::{DataMessage, DropToken, SharedMemoryId, Timestamped};

// Passed via env variable
#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct RuntimeConfig {
    pub node: NodeConfig,
    pub operators: Vec<OperatorDefinition>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NodeConfig {
    pub dataflow_id: DataflowId,
    pub node_id: NodeId,
    pub run_config: NodeRunConfig,
    pub daemon_communication: DaemonCommunication,
    pub dataflow_descriptor: Descriptor,
    pub dynamic: bool,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum DaemonCommunication {
    Shmem {
        daemon_control_region_id: SharedMemoryId,
        daemon_drop_region_id: SharedMemoryId,
        daemon_events_region_id: SharedMemoryId,
        daemon_events_close_region_id: SharedMemoryId,
    },
    Tcp {
        socket_addr: SocketAddr,
    },
    #[cfg(unix)]
    UnixDomain {
        socket_file: PathBuf,
    },
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[must_use]
pub enum DaemonReply {
    Result(Result<(), String>),
    PreparedMessage { shared_memory_id: SharedMemoryId },
    NextEvents(Vec<Timestamped<NodeEvent>>),
    NextDropEvents(Vec<Timestamped<NodeDropEvent>>),
    NodeConfig { result: Result<NodeConfig, String> },
    Empty,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum NodeEvent {
    Stop,
    Reload {
        operator_id: Option<OperatorId>,
    },
    Input {
        id: DataId,
        metadata: Metadata,
        data: Option<DataMessage>,
    },
    InputClosed {
        id: DataId,
    },
    AllInputsClosed,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum NodeDropEvent {
    OutputDropped { drop_token: DropToken },
}
