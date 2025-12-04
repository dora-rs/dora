use std::{net::SocketAddr, path::PathBuf};

use crate::{
    DataflowId,
    config::NodeRunConfig,
    descriptor::OperatorDefinition,
    id::{DataId, NodeId, OperatorId},
    metadata::Metadata,
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
    pub daemon_communication: Option<DaemonCommunication>,
    pub dataflow_descriptor: serde_yaml::Value,
    pub dynamic: bool,
    pub write_events_to: Option<PathBuf>,
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
    Interactive,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[must_use]
#[allow(clippy::large_enum_variant)]
pub enum DaemonReply {
    Result(Result<(), String>),
    PreparedMessage { shared_memory_id: SharedMemoryId },
    NextEvents(Vec<Timestamped<NodeEvent>>),
    NextDropEvents(Vec<Timestamped<NodeDropEvent>>),
    NodeConfig { result: Result<NodeConfig, String> },
    InputHealth { 
        result: Result<InputHealth, String> 
    },
    Empty,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize, PartialEq, Eq)]
pub enum InputHealth {
    Healthy,
    Timeout,
    Disconnected,
    Unknown,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[allow(clippy::large_enum_variant)]
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
    /// A peer node has started
    PeerStarted {
        node_id: NodeId,
    },
    /// A peer node has stopped
    PeerStopped {
        node_id: NodeId,
        reason: crate::common::NodeStopReason,
    },
    /// A peer node's health status changed
    PeerHealthChanged {
        node_id: NodeId,
        status: crate::common::HealthStatus,
    },
    /// An error event from an upstream node
    InputError {
        id: DataId,
        error: String,
    },
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum NodeDropEvent {
    OutputDropped { drop_token: DropToken },
}
