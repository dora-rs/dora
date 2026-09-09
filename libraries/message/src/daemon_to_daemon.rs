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
    /// An opaque message between the daemons of one dataflow, sent on
    /// behalf of an out-of-tree extension. dora never interprets
    /// `namespace` or `payload` — it moves the bytes and nothing else.
    ///
    /// This is the inter-daemon half of the extension seam (the node-facing
    /// halves are [`crate::node_to_daemon::DaemonRequest::ExtensionRequest`]
    /// and the extension table). Naming the variants after any one
    /// transport would freeze that transport's architecture into the 1.0
    /// protocol, which is exactly what `docs/extensions.md` exists to
    /// avoid; a second extension needs no change here at all.
    ExtensionMessage {
        dataflow_id: DataflowId,
        /// Extension that owns the payload. A daemon with no extension
        /// registered under this namespace drops the message.
        namespace: String,
        /// When set, only the daemon whose machine id matches acts on the
        /// message; the others drop it. `None` addresses every daemon in
        /// the dataflow.
        target_machine: Option<String>,
        #[serde(with = "crate::bulk_bytes::vec")]
        payload: Vec<u8>,
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
            // Opaque to dora, but it can carry a full tensor frame.
            Self::ExtensionMessage { payload, .. } => payload.len(),
        }
    }
}
