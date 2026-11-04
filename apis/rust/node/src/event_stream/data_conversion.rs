use std::{ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use dora_arrow_convert::IntoArrow;

use crate::arrow_utils::decode_arrow_ipc_zero_copy_raw;

pub enum RawData {
    Empty,
    Vec(AVec<u8, ConstAlign<128>>),
}

impl RawData {
    pub fn into_arrow_array(self) -> eyre::Result<arrow::array::ArrayData> {
        match self {
            // A metadata-only message with no payload. Any real array is a
            // non-empty IPC stream, so an empty payload maps to the unit array.
            // An empty `Vec` (a zero-length sample sent over the daemon path)
            // is the same message, and the zenoh path already decodes an
            // empty payload this way, so it must not become `Event::Error`.
            RawData::Empty => Ok(unit_array()),
            RawData::Vec(data) if data.is_empty() => Ok(unit_array()),
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
                decode_arrow_ipc_zero_copy_raw(raw_buffer)
            }
        }
    }
}

/// The unit array an empty payload decodes to, on every receive path.
pub(crate) fn unit_array() -> arrow::array::ArrayData {
    dora_arrow_convert::internal::into_array_ref(().into_arrow()).to_data()
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

    /// The daemon/TCP fallback carries the IPC stream as a postcard-serialized
    /// `DataMessage::Vec`. A round-trip through that serialization must preserve
    /// both the payload and its 128-byte alignment, so the receiver still
    /// decodes zero-copy via `decode_arrow_ipc_zero_copy`.
    #[test]
    fn daemon_path_ipc_roundtrip_preserves_payload_and_alignment() {
        let data = Float32Array::from((0..1000).map(|i| i as f32).collect::<Vec<_>>()).into_data();

        // Encode the IPC stream into a 128-byte-aligned AVec, exactly as the
        // send path does for the daemon fallback.
        let len = ipc_encode::ipc_fast_path_len_data(&data).unwrap();
        let mut avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, len);
        ipc_encode::encode_ipc_into_data(&data, &mut avec).unwrap();
        let message = DataMessage::Vec(avec);

        // Same encode/decode entry points the node->daemon->node TCP hops use.
        let bytes = dora_message::encode(&message).unwrap();
        let restored: DataMessage = dora_message::decode(&bytes).unwrap();
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

    /// A zero-length sample reaches a daemon-path receiver as an empty
    /// `DataMessage::Vec`. It must decode to the unit array, as an empty
    /// zenoh payload does, rather than fail as an IPC stream with no batches.
    #[test]
    fn empty_vec_payload_decodes_like_no_payload() {
        let decoded = RawData::Vec(AVec::new(128)).into_arrow_array().unwrap();
        assert_eq!(decoded, RawData::Empty.into_arrow_array().unwrap());
    }

    /// An empty payload (metadata-only message) decodes to the unit array, never
    /// an error.
    #[test]
    fn empty_payload_decodes_to_unit_array() {
        let decoded = RawData::Empty.into_arrow_array().unwrap();
        assert_eq!(decoded, unit_array());
    }
}
