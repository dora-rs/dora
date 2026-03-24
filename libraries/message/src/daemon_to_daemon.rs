use aligned_vec::{AVec, ConstAlign};
use std::collections::BTreeMap;

use crate::{
    DataflowId,
    common::DaemonId,
    id::{DataId, NodeId},
    metadata::Metadata,
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct StateUpdate {
    pub dataflow_id: DataflowId,
    pub source_daemon_id: DaemonId,
    pub revision: u64,
    pub key: String,
    /// `None` means delete.
    pub value: Option<Vec<u8>>,
}

#[derive(Debug, serde::Deserialize, serde::Serialize)]
#[allow(clippy::large_enum_variant)]
pub enum InterDaemonEvent {
    Output {
        dataflow_id: DataflowId,
        node_id: NodeId,
        output_id: DataId,
        metadata: Metadata,
        data: Option<AVec<u8, ConstAlign<128>>>,
    },
    OutputClosed {
        dataflow_id: DataflowId,
        node_id: NodeId,
        output_id: DataId,
    },
    NodeFailed {
        dataflow_id: DataflowId,
        source_node_id: NodeId,
        error: String,
    },
    /// Replicated state update produced by one daemon.
    StateUpdate(StateUpdate),
    /// Ask peer daemons for all missing updates since known revisions.
    StateCatchUpRequest {
        dataflow_id: DataflowId,
        requester_daemon_id: DaemonId,
        known_revisions: BTreeMap<DaemonId, u64>,
    },
    /// Catch-up response carrying incremental updates to a specific daemon.
    StateCatchUpResponse {
        dataflow_id: DataflowId,
        target_daemon_id: DaemonId,
        updates: Vec<StateUpdate>,
    },
}
