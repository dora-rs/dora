use std::sync::Arc;

use dora_arrow_convert::IntoArrow;
use dora_message::node_to_daemon::DataMessage;

use crate::arrow_utils::{IpcPayload, decode_arrow_ipc_zero_copy_raw};

/// Decode a received payload into an Arrow array without copying it.
///
/// The payload stays behind its `Arc`: the Arrow buffer aliases the
/// 128-byte-aligned allocation and keeps the `Arc` alive as its owner, and the
/// zero-copy IPC decoder slices the array buffers out of it in place (it
/// realigns internally only if under-aligned). Taking the `Arc` by reference
/// means a payload that is still shared — a recorded input, an
/// integration-test output — decodes in place instead of being deep-copied by
/// `Arc::unwrap_or_clone` just to hand ownership to the decoder.
pub(crate) fn shared_data_to_arrow_array(
    data: Option<&Arc<DataMessage>>,
) -> eyre::Result<arrow::array::ArrayRef> {
    let data = match data {
        // A metadata-only message with no payload. Any real array is a
        // non-empty IPC stream, so an empty payload maps to the unit array.
        // An empty `Vec` (a zero-length sample sent over the daemon path)
        // is the same message, and the zenoh path already decodes an
        // empty payload this way, so it must not become `Event::Error`.
        None => unit_array(),
        Some(message) if message.is_empty() => unit_array(),
        Some(message) => {
            decode_arrow_ipc_zero_copy_raw(IpcPayload::from_shared(message.clone()).into_arrow())?
        }
    };
    Ok(arrow::array::make_array(data))
}

/// The unit array an empty payload decodes to, on every receive path.
pub(crate) fn unit_array() -> arrow::array::ArrayData {
    dora_arrow_convert::internal::into_array_ref(().into_arrow()).to_data()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arrow_utils::ipc_encode;
    use aligned_vec::{AVec, ConstAlign};
    use arrow::array::{Array, Float32Array};

    fn decode(message: DataMessage) -> arrow::array::ArrayData {
        shared_data_to_arrow_array(Some(&Arc::new(message)))
            .unwrap()
            .to_data()
    }

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

        let decoded = decode(DataMessage::Vec(avec));
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
        let decoded = decode(DataMessage::Vec(AVec::new(128)));
        assert_eq!(decoded, unit_array());
    }

    /// A payload shared behind an `Arc` (a recorded input, an integration-test
    /// output) decodes in place: the decoded array aliases the shared
    /// allocation instead of a copy of it.
    #[test]
    fn shared_payload_decodes_without_copying() {
        let data = Float32Array::from((0..1000).map(|i| i as f32).collect::<Vec<_>>()).into_data();
        let len = ipc_encode::ipc_fast_path_len_data(&data).unwrap();
        let mut avec: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, len);
        ipc_encode::encode_ipc_into_data(&data, &mut avec).unwrap();
        let payload_range = avec.as_ptr_range();
        let shared = Arc::new(DataMessage::Vec(avec));

        let decoded = shared_data_to_arrow_array(Some(&shared)).unwrap().to_data();
        assert_eq!(data, decoded);
        let values = decoded.buffers()[0].as_ptr();
        assert!(
            payload_range.contains(&values),
            "decoded values must alias the shared payload, not a copy of it"
        );
        // The caller's handle is untouched and still usable.
        assert_eq!(
            Arc::strong_count(&shared),
            2,
            "decoded array keeps the payload alive"
        );
        drop(decoded);
        assert_eq!(Arc::strong_count(&shared), 1);

        let empty = Arc::new(DataMessage::Vec(AVec::new(128)));
        assert_eq!(
            shared_data_to_arrow_array(Some(&empty)).unwrap().to_data(),
            unit_array()
        );
    }

    /// An empty payload (metadata-only message) decodes to the unit array, never
    /// an error.
    #[test]
    fn empty_payload_decodes_to_unit_array() {
        let decoded = shared_data_to_arrow_array(None).unwrap().to_data();
        assert_eq!(decoded, unit_array());
    }
}
