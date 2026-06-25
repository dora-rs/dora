use std::{ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use dora_arrow_convert::IntoArrow;

use crate::arrow_utils::decode_arrow_ipc_zero_copy;

pub enum RawData {
    Empty,
    Vec(AVec<u8, ConstAlign<128>>),
}

impl RawData {
    pub fn into_arrow_array(self) -> eyre::Result<arrow::array::ArrayData> {
        match self {
            // A metadata-only message with no payload. Any real array is a
            // non-empty IPC stream, so an empty payload maps to the unit array.
            RawData::Empty => Ok(().into_arrow().into()),
            RawData::Vec(data) => {
                // Wrap the 128-byte-aligned `AVec` as an Arrow `Buffer` without
                // copying (the buffer aliases the allocation), then let the
                // zero-copy IPC decoder slice the array buffers out of it in
                // place (it realigns internally only if under-aligned).
                let ptr = NonNull::new(data.as_ptr() as *mut _).unwrap();
                let len = data.len();
                let raw_buffer = unsafe {
                    arrow::buffer::Buffer::from_custom_allocation(ptr, len, Arc::new(data))
                };
                decode_arrow_ipc_zero_copy(raw_buffer)
            }
        }
    }
}

impl std::fmt::Debug for RawData {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Data").finish_non_exhaustive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arrow_utils::ipc_encode;
    use arrow::array::{Array, Float32Array};
    use dora_message::node_to_daemon::DataMessage;

    /// The daemon/TCP fallback carries the IPC stream as a bincode-serialized
    /// `DataMessage::Vec`. A round-trip through that serialization must preserve
    /// both the payload and its 128-byte alignment, so the receiver still
    /// decodes zero-copy via `decode_arrow_ipc_zero_copy`.
    #[test]
    fn daemon_path_ipc_roundtrip_preserves_payload_and_alignment() {
        let data = Float32Array::from((0..1000).map(|i| i as f32).collect::<Vec<_>>()).into_data();

        // Encode the IPC stream into a 128-byte-aligned AVec, exactly as the
        // send path does for the daemon fallback.
        let len = ipc_encode::ipc_fast_path_len(&data).unwrap();
        let mut avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, len);
        ipc_encode::encode_ipc_into(&data, &mut avec).unwrap();
        let message = DataMessage::Vec(avec);

        // bincode round-trip = what the node->daemon->node TCP hops do.
        let bytes = bincode::serialize(&message).unwrap();
        let restored: DataMessage = bincode::deserialize(&bytes).unwrap();
        let DataMessage::Vec(avec) = restored;
        assert_eq!(
            avec.as_ptr() as usize % 128,
            0,
            "deserialized DataMessage::Vec must stay 128-byte aligned for zero-copy decode"
        );

        let decoded = RawData::Vec(avec).into_arrow_array().unwrap();
        assert_eq!(
            data, decoded,
            "daemon-path payload must decode to the input"
        );
    }

    /// An empty payload (metadata-only message) decodes to the unit array, never
    /// an error.
    #[test]
    fn empty_payload_decodes_to_unit_array() {
        let decoded = RawData::Empty.into_arrow_array().unwrap();
        assert_eq!(decoded, ().into_arrow().into());
    }
}
