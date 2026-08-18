//! The tensor-pool's own messages, carried opaquely by dora.
//!
//! dora's protocol knows nothing about pools: a node's call reaches the
//! daemon inside [`DaemonRequest::ExtensionRequest`][req], and a daemon
//! reaches its peers inside [`InterDaemonEvent::ExtensionMessage`][msg].
//! Both carry `namespace` + opaque bytes, and the bytes are the types
//! below. Everything pool-shaped therefore lives here, in the extension,
//! rather than in dora's 1.0 wire surface — see `docs/extensions.md`.
//!
//! [req]: dora_message::node_to_daemon::DaemonRequest::ExtensionRequest
//! [msg]: dora_message::daemon_to_daemon::InterDaemonEvent::ExtensionMessage
//!
//! These types are **not** covered by dora's 1.0 compatibility guarantees;
//! they may change in any release, including a patch.

use serde::{Deserialize, Serialize};

/// The namespace this extension claims, in both the extension table and
/// the two opaque channels above.
pub const NAMESPACE: &str = "dora-tensor-pool";

/// A call from a node to its own daemon's pool half.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NodeRequest {
    /// Mirror this pool on `machine_id`: the daemon resolves that machine
    /// through the coordinator, asks its daemon to create the mirror, and
    /// waits for the acknowledgement before replying.
    RegisterCrossMachine {
        shared_memory_id: String,
        /// The sender's local segment name — forwarded to the mirror
        /// daemon so a same-host reader can open it directly.
        shmem_name: String,
        size: usize,
        dtype: String,
        shape: Vec<i64>,
        device: String,
        machine_id: String,
    },
    /// Push the pool's current contents to its mirror. The daemon reads
    /// the bytes out of the sender's segment itself, so the request stays
    /// KB-scale regardless of pool size.
    Write {
        shared_memory_id: String,
        size: usize,
    },
    /// Release the pool, including any mirror on another machine.
    Free { shared_memory_id: String },
}

/// The reply to a [`NodeRequest`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum NodeResponse {
    Ok,
    Error(String),
    /// Answer to [`NodeRequest::RegisterCrossMachine`]. `result` is `Err`
    /// when the register was a warn-and-no-op (machine unresolved, or the
    /// remote could not create the mirror). `direct` reports that the
    /// remote daemon opened the sender's segment itself (same host), so
    /// the per-frame push is skipped.
    CrossMachineRegistered {
        result: Result<(), String>,
        direct: bool,
    },
}

/// A message between the pool halves of two daemons.
///
/// The dataflow id and the target machine live in the envelope
/// (`ExtensionMessage`), so they are not repeated here.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PeerMessage {
    /// Tensor data for a mirrored pool: the mirror daemon writes it into
    /// the mirror segment's data region under the seqlock, and the
    /// receiver reads its local segment zero-copy.
    Write {
        shared_memory_id: String,
        tensor_data: Vec<u8>,
        size: usize,
        /// Per-pool sequence assigned by the origin, echoed in
        /// [`PeerMessage::WriteAck`] so an ack for an earlier write can
        /// never resolve a newer pending reply.
        seq: u64,
    },
    /// Commit acknowledgement for a [`PeerMessage::Write`]. The origin's
    /// reply waits on it, so the output notification that follows a write
    /// can never overtake the tensor data.
    WriteAck {
        shared_memory_id: String,
        seq: u64,
        ok: bool,
        error: Option<String>,
    },
    /// Mirror this pool locally and answer with [`PeerMessage::RegisterAck`].
    Register {
        /// The machine that created the pool. The mirror records
        /// `{pool id -> origin}` so a later free can be targeted.
        origin_machine_id: String,
        shared_memory_id: String,
        shmem_name: String,
        size: usize,
        dtype: String,
        shape: Vec<i64>,
        device: String,
    },
    RegisterAck {
        shared_memory_id: String,
        ok: bool,
        /// The mirror daemon could open the sender's segment directly
        /// (same host): readers need no transfer at all.
        direct: bool,
        error: Option<String>,
        /// The mirror's direct-TCP data listener port, when it has one.
        /// `None` falls the origin back to the zenoh relay.
        data_port: Option<u16>,
        /// The mirror's explicitly advertised dialable address
        /// (`DORA_MEMORY_POOL_DATA_ADDR`). Overrides the
        /// coordinator-derived address, which is only the WS source
        /// address and so is the wrong dial target under NAT, on a
        /// multi-homed host, or when the coordinator is local.
        data_addr: Option<std::net::SocketAddr>,
    },
    /// Drop the mirror tracking entry and unlink the mirror segment.
    Free { shared_memory_id: String },
}
