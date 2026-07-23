use std::{
    collections::{BTreeMap, BTreeSet},
    net::SocketAddr,
    path::PathBuf,
};

use std::sync::Arc;

use crate::{
    DataflowId,
    config::NodeRunConfig,
    descriptor::OperatorDefinition,
    id::{DataId, NodeId, OperatorId},
    metadata::Metadata,
};

pub use crate::common::{DataMessage, SharedMemoryId, Timestamped};

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
    /// Per-output data-plane routing for the startup handshake, computed by the
    /// daemon from the **actual** placement of the dataflow's nodes at spawn
    /// time (the descriptor's `deploy` section is intent, not placement — label
    /// scheduling can resolve differently).
    ///
    /// `None` means the node was spawned by an older daemon that doesn't
    /// provide routing; the node then keeps every output on the reliable daemon
    /// path (correct, just without the direct-zenoh fast path).
    #[serde(default)]
    pub output_routing: Option<BTreeMap<DataId, OutputRouting>>,
}

/// Data-plane routing for one node output — see
/// [`NodeConfig::output_routing`].
#[derive(Debug, Default, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct OutputRouting {
    /// Some consumer of this output runs under another daemon. All sends must
    /// then go through this node's daemon so its inter-daemon forwarding can
    /// reach them (dora #2738) — the direct node-to-node zenoh mesh is
    /// same-machine only.
    #[serde(default)]
    pub daemon_only: bool,
    /// The static same-daemon consumers whose startup acks the producer must
    /// collect before switching this output from the reliable daemon path to
    /// the direct node-to-node zenoh path. Dynamic consumers are never
    /// required: they join at arbitrary times (or never), and nothing may wait
    /// on them.
    #[serde(default)]
    pub required_ackers: BTreeSet<RequiredAcker>,
}

/// Identity of one consumer input that must ack a producer's startup markers
/// before the producer may switch the corresponding output to the direct zenoh
/// path — see [`OutputRouting::required_ackers`].
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, serde::Serialize, serde::Deserialize)]
pub struct RequiredAcker {
    pub node_id: NodeId,
    pub input_id: DataId,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum DaemonCommunication {
    Tcp { socket_addr: SocketAddr },
    Interactive,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[must_use]
#[allow(clippy::large_enum_variant)]
pub enum DaemonReply {
    Result(Result<(), String>),
    NextEvents(Vec<Timestamped<NodeEvent>>),
    NodeConfig { result: Result<NodeConfig, String> },
    PinnedMemoryMetadata { metadata: Metadata },
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
    ///
    /// `value_json` carries JSON-encoded bytes rather than `serde_json::Value`:
    /// this message is serialized with bincode on the daemon↔node TCP channel,
    /// and `serde_json::Value::deserialize` uses `deserialize_any`, which
    /// bincode does not support.
    ParamUpdate {
        key: String,
        value_json: Vec<u8>,
    },
    /// A runtime parameter has been deleted.
    ///
    /// Sent when `dora param delete` removes a parameter for this node.
    ParamDeleted {
        key: String,
    },
    /// An upstream node has failed.
    ///
    /// Sent to downstream nodes when an upstream node exits with a
    /// non-zero exit code.
    NodeFailed {
        affected_input_ids: Vec<DataId>,
        error: String,
        source_node_id: NodeId,
    },
}
