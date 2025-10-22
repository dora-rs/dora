use std::{net::SocketAddr, path::PathBuf};

use tokio::sync::oneshot;

use crate::{
    DataflowId,
    config::NodeRunConfig,
    descriptor::OperatorDefinition,
    id::{DataId, NodeId, OperatorId},
    metadata::Metadata,
    node_to_daemon::DaemonRequest,
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
    pub dataflow_descriptor: serde_yaml::Value,
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
    Interactive,
    IntegrationTest {
        input_file: PathBuf,
        output_file: PathBuf,
        skip_output_time_offsets: bool,
    },
    IntegrationTestInitialized {
        #[serde(skip)]
        channel: Option<
            tokio::sync::mpsc::Sender<(Timestamped<DaemonRequest>, oneshot::Sender<DaemonReply>)>,
        >,
    },
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
