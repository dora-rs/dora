use std::collections::BTreeSet;

use aligned_vec::{AVec, ConstAlign};

use crate::{
    id::{DataId, NodeId},
    metadata::Metadata,
    DataflowId,
};

#[derive(Debug, serde::Deserialize, serde::Serialize)]
pub enum InterDaemonEvent {
    Output {
        dataflow_id: DataflowId,
        node_id: NodeId,
        output_id: DataId,
        metadata: Metadata,
        data: Option<AVec<u8, ConstAlign<128>>>,
    },
    InputsClosed {
        dataflow_id: DataflowId,
        inputs: BTreeSet<(NodeId, DataId)>,
    },
}
