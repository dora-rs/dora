use aligned_vec::{AVec, ConstAlign};

use crate::{
    DataflowId,
    id::{DataId, NodeId},
    metadata::Metadata,
};

#[derive(Debug, Clone, serde::Deserialize, serde::Serialize)]
#[allow(clippy::large_enum_variant)]
#[non_exhaustive]
pub enum InterDaemonEvent {
    Output {
        dataflow_id: DataflowId,
        node_id: NodeId,
        output_id: DataId,
        metadata: Metadata,
        #[serde(with = "crate::bulk_bytes::option")]
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
        #[serde(with = "crate::bulk_bytes::vec")]
        tensor_data: Vec<u8>,
        size: usize,
        /// Per-pool write sequence assigned by the origin daemon. Echoed
        /// back in `MemoryPoolWriteAck` so the commit matches the exact
        /// write (an ack for a previous write can never resolve a newer
        /// pending reply).
        seq: u64,
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
        /// The sender's local /dev/shm segment name — the receiver daemon
        /// records it as a remote reference so same-host readers can open
        /// the sender's segment directly (zero-copy, no transfer).
        shmem_name: String,
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
        /// Whether this daemon could open the sender's segment directly
        /// (same host). When true, the origin skips the per-frame data
        /// push — readers open the sender's segment, no transfer needed.
        direct: bool,
        error: Option<String>,
        /// The mirror daemon's direct-TCP data-plane listener port, when
        /// available. The origin opens a persistent connection to
        /// `<target daemon address>:<data_port>` for cross-machine writes
        /// (one user-space copy on the send side; the receiver writes the
        /// stream straight into the mirror segment). `None` when the
        /// mirror daemon has no data listener (falls back to zenoh).
        data_port: Option<u16>,
        /// The mirror daemon's **explicitly advertised** dialable address
        /// (`DORA_MEMORY_POOL_DATA_ADDR`), when set. Overrides the
        /// coordinator-derived address: the coordinator only sees the WS
        /// source address, which is the wrong dial target under NAT,
        /// multi-homed, or same-host coordinator deployment (e.g.
        /// `127.0.0.1` when the daemon connects locally). `None` without
        /// the env — the origin then falls back to
        /// `<coordinator-visible address>:<data_port>`.
        data_addr: Option<std::net::SocketAddr>,
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
    /// Remote commit acknowledgement for a cross-machine write: the
    /// mirror daemon publishes this after the mirror segment write
    /// completed. The origin's `write_memory_pool` reply waits for it,
    /// so the output notification that follows the write can never
    /// overtake the tensor data (the receiver would otherwise return
    /// the previous stable frame).
    MemoryPoolWriteAck {
        dataflow_id: DataflowId,
        shared_memory_id: String,
        seq: u64,
        ok: bool,
        error: Option<String>,
    },
}

impl InterDaemonEvent {
    /// Bulk bytes this event will contribute to its encoding, for
    /// [`crate::encode_presized`].
    ///
    /// Not to be confused with [`crate::metadata::debug_frame_wire_size`], which
    /// answers "how big was this on the wire" and deliberately prefers the
    /// daemon-stamped `WIRE_SIZE` parameter over the buffer length (#2584). This
    /// one must be the actual buffer length, since it sizes an allocation.
    pub fn encode_size_hint(&self) -> usize {
        match self {
            Self::Output { data, .. } => data.as_ref().map_or(0, |d| d.len()),
            Self::OutputClosed { .. } => 0,
            // Cross-machine memory-pool events: payload is `tensor_data`
            // (MemoryPoolWrite) or control-plane sizes.
            Self::MemoryPoolWrite { tensor_data, .. } => tensor_data.len(),
            Self::RegisterPool { .. }
            | Self::RegisterPoolAck { .. }
            | Self::MemoryPoolWriteAck { .. }
            | Self::FreePool { .. } => 0,
        }
    }
}
