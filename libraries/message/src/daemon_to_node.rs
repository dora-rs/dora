use std::{net::SocketAddr, path::PathBuf};

use std::sync::Arc;

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
    /// Number of times this node has been restarted. 0 on first run.
    #[serde(default)]
    pub restart_count: u32,
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
    Empty,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[non_exhaustive]
#[allow(clippy::large_enum_variant)]
pub enum NodeEvent {
    Stop,
    Reload {
        operator_id: Option<OperatorId>,
    },
    Input {
        id: DataId,
        metadata: Arc<Metadata>,
        data: Option<Arc<DataMessage>>,
    },
    InputClosed {
        id: DataId,
    },
    /// Notifies a node that a previously closed input has recovered and will receive data again.
    InputRecovered {
        id: DataId,
    },
    /// Notifies a node that an upstream node has restarted.
    ///
    /// Sent to downstream nodes when a node with a restart policy successfully
    /// restarts after a failure.
    NodeRestarted {
        id: NodeId,
    },
    /// Notifies a node that all its inputs have been closed.
    ///
    /// This event is only sent to nodes that have at least one input.
    AllInputsClosed,
    /// A runtime parameter has been updated.
    ///
    /// Sent when `dora param set` changes a parameter for this node.
    ParamUpdate {
        key: String,
        value: serde_json::Value,
    },
    /// A runtime parameter has been deleted.
    ///
    /// Sent when `dora param delete` removes a parameter for this node.
    ParamDeleted {
        key: String,
    },
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum NodeDropEvent {
    OutputDropped { drop_token: DropToken },
}
