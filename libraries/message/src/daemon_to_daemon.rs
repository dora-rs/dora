use aligned_vec::{AVec, ConstAlign};

use crate::{
    DataflowId,
    id::{DataId, NodeId},
    metadata::Metadata,
};

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
    /// Cross-machine memory pool write — the sender daemon forwards
    /// serialised tensor data to the remote daemon, which stores it
    /// in a proxy pool until the receiver calls `read_memory_pool`.
    MemoryPoolWrite {
        dataflow_id: DataflowId,
        shared_memory_id: String,
        tensor_data: Vec<u8>,
        size: usize,
        device: String,
        // Original tensor dtype/shape (see WritePinnedMemory): remote
        // receivers rebuild the tensor from these, not a uint8 view.
        dtype: String,
        shape: Vec<i64>,
    },
}
