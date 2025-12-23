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
    NextEvents(Vec<Timestamped<NodeEventOrUnknown>>),
    NextDropEvents(Vec<Timestamped<NodeDropEvent>>),
    NodeConfig { result: Result<NodeConfig, String> },
    Empty,
}

#[derive(Debug, Clone, serde::Serialize)]
#[serde(untagged)]
pub enum NodeEventOrUnknown {
    Known(Box<NodeEvent>),
    Unknown,
}

impl<'de> serde::Deserialize<'de> for NodeEventOrUnknown {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let event = serde::Deserialize::deserialize(deserializer);
        match event {
            Ok(event) => Ok(NodeEventOrUnknown::Known(event)),
            Err(_) => Ok(NodeEventOrUnknown::Unknown),
        }
    }
}

impl From<NodeEvent> for NodeEventOrUnknown {
    fn from(event: NodeEvent) -> Self {
        NodeEventOrUnknown::Known(Box::new(event))
    }
}

impl From<Timestamped<NodeEvent>> for Timestamped<NodeEventOrUnknown> {
    fn from(value: Timestamped<NodeEvent>) -> Self {
        Timestamped {
            inner: value.inner.into(),
            timestamp: value.timestamp,
        }
    }
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
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum NodeDropEvent {
    OutputDropped { drop_token: DropToken },
}
