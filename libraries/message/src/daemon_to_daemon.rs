use aligned_vec::{AVec, ConstAlign};

use crate::{
    DataflowId,
    common::DaemonId,
    id::{DataId, NodeId},
    metadata::Metadata,
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
pub struct StateUpdate {
    pub dataflow_id: DataflowId,
    pub key: String,
    pub value: Option<Vec<u8>>,
    pub revision: u64,
    pub source_daemon_id: DaemonId,
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
    StateUpdate(StateUpdate),
}
