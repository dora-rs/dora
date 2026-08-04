use aligned_vec::{AVec, ConstAlign};

use crate::{
    DataflowId,
    id::{DataId, NodeId},
    metadata::Metadata,
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
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
    /// serialised tensor data to the remote daemon, which writes it
    /// directly into the mirrored pool's DORADMA data region under the
    /// seqlock protocol (the receiver reads its local pool zero-copy).
    MemoryPoolWrite {
        dataflow_id: DataflowId,
        shared_memory_id: String,
        tensor_data: Vec<u8>,
        size: usize,
    },
    /// Cross-machine pool registration — the matching machine's daemon
    /// mirrors the pool locally and replies with `RegisterPoolAck`.
    RegisterPool {
        dataflow_id: DataflowId,
        /// Target machine id — the daemon whose machine id matches
        /// mirrors the pool.
        machine_id: String,
        /// Origin machine id — the machine that created the pool. The
        /// mirror records `{pool id -> origin}` for the targeted free.
        origin_machine_id: String,
        shared_memory_id: String,
        size: usize,
        dtype: String,
        shape: Vec<i64>,
        device: String,
    },
    /// Acknowledge a cross-machine pool registration (sync register).
    RegisterPoolAck {
        dataflow_id: DataflowId,
        shared_memory_id: String,
        ok: bool,
        error: Option<String>,
    },
    /// Release a cross-machine pool on the target machine. The event is
    /// a dataflow-scope broadcast; the daemon whose machine id matches
    /// `machine_id` drops its tracking entry and unlinks its mirror
    /// (same gating pattern as `RegisterPool`).
    FreePool {
        dataflow_id: DataflowId,
        /// Target machine id — only that machine's daemon acts.
        machine_id: String,
        shared_memory_id: String,
    },
}
