use std::{ptr::NonNull, sync::Arc};

use aligned_vec::{AVec, ConstAlign};
use arrow::alloc::Allocation;
use dora_arrow_convert::IntoArrow;
use dora_message::common::DataMessage;

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
                let data = Arc::new(data);
                decode_in_place(&data, data.clone())
            }
        }
    }
}

/// Decode a payload that is still shared with its event, without copying it.
///
/// Like [`RawData::into_arrow_array`], but the Arrow buffer keeps the `Arc`
/// alive instead of taking the `AVec` by value, so a caller that only borrows
/// the event (the recorder, the integration-testing output writer) need not
/// clone the whole payload first.
pub(crate) fn shared_data_to_arrow_array(
    data: Arc<DataMessage>,
) -> eyre::Result<arrow::array::ArrayData> {
    let DataMessage::Vec(v) = &*data;
    if v.is_empty() {
        return Ok(unit_array());
    }
    decode_in_place(v, data.clone())
}

/// Wrap the 128-byte-aligned `bytes` as an Arrow `Buffer` without copying (the
/// buffer aliases the allocation and keeps `owner` alive), then let the
/// zero-copy IPC decoder slice the array buffers out of it in place (it
/// realigns internally only if under-aligned).
///
/// `owner` must own `bytes` and never mutate them while the buffer exists.
fn decode_in_place(
    bytes: &AVec<u8, ConstAlign<128>>,
    owner: Arc<dyn Allocation>,
) -> eyre::Result<arrow::array::ArrayData> {
    let ptr = NonNull::from(bytes.as_slice()).cast::<u8>();
    // SAFETY: `ptr`/`len` describe memory owned by `owner`, which the buffer
    // keeps alive and which is never mutated behind the shared `Arc`.
    let raw_buffer =
        unsafe { arrow::buffer::Buffer::from_custom_allocation(ptr, bytes.len(), owner) };
    decode_arrow_ipc_zero_copy_raw(raw_buffer)
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

    /// Decoding a payload still shared with its event yields the same array as
    /// the owned path, and aliases the shared allocation rather than copying.
    #[test]
    fn shared_payload_decodes_in_place() {
        let data = Float32Array::from((0..1000).map(|i| i as f32).collect::<Vec<_>>()).into_data();
        let len = ipc_encode::ipc_fast_path_len_data(&data).unwrap();
        let mut avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, len);
        ipc_encode::encode_ipc_into_data(&data, &mut avec).unwrap();
        let shared = Arc::new(DataMessage::Vec(avec));
        let DataMessage::Vec(v) = &*shared;
        let payload = v.as_ptr_range();

        let decoded = shared_data_to_arrow_array(shared.clone()).unwrap();
        assert_eq!(data, decoded);
        let values = decoded.buffers()[0].as_ptr();
        assert!(
            payload.contains(&values),
            "the decoded values must alias the shared payload, not a copy of it"
        );
        assert!(
            Arc::strong_count(&shared) > 1,
            "the array keeps the payload alive"
        );

        let empty = shared_data_to_arrow_array(Arc::new(DataMessage::Vec(AVec::new(128)))).unwrap();
        assert_eq!(empty, unit_array());
    }

    /// An empty payload (metadata-only message) decodes to the unit array, never
    /// an error.
    #[test]
    fn empty_payload_decodes_to_unit_array() {
        let decoded = RawData::Empty.into_arrow_array().unwrap();
        assert_eq!(decoded, unit_array());
    }
}
