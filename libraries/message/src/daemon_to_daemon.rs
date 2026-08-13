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
        }
    }
}
