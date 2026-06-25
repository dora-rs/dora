//! Utility functions for converting Arrow arrays to/from raw data.
//!
pub mod ipc_encode;

use arrow::array::ArrayData;
use eyre::Context;

/// Maximum Arrow IPC payload size (256 MB).
const MAX_IPC_BYTES: usize = 256 * 1024 * 1024;

/// Alignment guaranteed for every raw Arrow buffer inside Dora payloads.
///
/// Arrow kernels can issue SIMD loads from buffer bases. Some ARM platforms
/// fault on under-aligned SIMD loads, so every body buffer of an Arrow IPC
/// stream is placed at a 64-byte boundary relative to the payload base.
pub(crate) const ARROW_BUFFER_ALIGNMENT: usize = 64;
pub(crate) const ARROW_BUFFER_ALIGNMENT_EXPONENT: u8 =
    ARROW_BUFFER_ALIGNMENT.trailing_zeros() as u8;
const _: () = assert!(ARROW_BUFFER_ALIGNMENT.is_power_of_two());

/// Encode an Arrow [`ArrayData`] into an Arrow IPC stream byte buffer.
///
/// The resulting buffer contains a full IPC stream: schema message, one record
/// batch, and an end-of-stream marker. This is self-describing and can be
/// decoded without external type information.
pub fn encode_arrow_ipc(arrow_array: &ArrayData) -> eyre::Result<Vec<u8>> {
    use arrow::ipc::writer::StreamWriter;
    use arrow::record_batch::RecordBatch;
    use arrow_schema::{Field, Schema};
    use std::sync::Arc;

    let schema = Schema::new(vec![Field::new(
        "data",
        arrow_array.data_type().clone(),
        true,
    )]);
    let schema_ref = Arc::new(schema);

    let array_ref = arrow::array::make_array(arrow_array.clone());
    let batch = RecordBatch::try_new(schema_ref.clone(), vec![array_ref])
        .context("failed to create RecordBatch for IPC encoding")?;

    let mut buf = Vec::new();
    {
        let mut writer = StreamWriter::try_new(&mut buf, &schema_ref)
            .context("failed to create Arrow IPC StreamWriter")?;
        writer
            .write(&batch)
            .context("failed to write RecordBatch to IPC stream")?;
        writer
            .finish()
            .context("failed to finish Arrow IPC stream")?;
    }
    Ok(buf)
}

/// Decode an Arrow IPC stream byte buffer back into [`ArrayData`].
///
/// Expects the buffer to contain exactly one record batch with a single
/// column named `"data"`, as produced by [`encode_arrow_ipc`].
pub fn decode_arrow_ipc(ipc_buf: &[u8]) -> eyre::Result<ArrayData> {
    use arrow::ipc::reader::StreamReader;
    use std::io::Cursor;

    if ipc_buf.len() > MAX_IPC_BYTES {
        eyre::bail!(
            "Arrow IPC payload too large: {} bytes (max {MAX_IPC_BYTES})",
            ipc_buf.len()
        );
    }

    let cursor = Cursor::new(ipc_buf);
    let mut reader =
        StreamReader::try_new(cursor, None).context("failed to open Arrow IPC stream")?;

    let batch = reader
        .next()
        .ok_or_else(|| eyre::eyre!("Arrow IPC stream contained no record batches"))?
        .context("failed to read RecordBatch from IPC stream")?;

    if batch.num_columns() != 1 {
        eyre::bail!(
            "expected 1 column in IPC record batch, got {}",
            batch.num_columns()
        );
    }

    Ok(batch.column(0).to_data())
}

/// Decode an Arrow IPC stream from an Arrow [`Buffer`] **without copying** the
/// payload buffers when they are properly aligned.
///
/// Unlike [`decode_arrow_ipc`], which reads from a byte slice through
/// `StreamReader` (and therefore allocates a fresh buffer and copies every
/// array buffer out of the stream), this uses
/// [`arrow::ipc::reader::StreamDecoder`], which slices the array buffers
/// directly out of the provided [`Buffer`]. When the input buffer is suitably
/// aligned — as Dora's shared-memory payloads always are (128-byte `AVec` /
/// page-aligned Zenoh SHM) — the decoded array aliases the input and no payload
/// copy happens.
///
/// The decoder runs with the default `require_alignment = false`, so an
/// under-aligned input (e.g. an arbitrary heap `Vec`) is handled gracefully by
/// copying just the misaligned buffers rather than erroring. This keeps the
/// receive path robust while preserving zero-copy for the common SHM case.
pub fn decode_arrow_ipc_zero_copy(
    mut buffer: arrow::buffer::Buffer,
) -> eyre::Result<arrow::array::ArrayData> {
    use arrow::ipc::reader::StreamDecoder;

    if buffer.len() > MAX_IPC_BYTES {
        eyre::bail!(
            "Arrow IPC payload too large: {} bytes (max {MAX_IPC_BYTES})",
            buffer.len()
        );
    }

    let mut decoder = StreamDecoder::new();
    let mut batch = None;
    // `decode` is push-based: it may consume the schema message and return
    // `None` before yielding the record batch, so loop until we get a batch or
    // exhaust the input.
    while !buffer.is_empty() {
        if let Some(b) = decoder
            .decode(&mut buffer)
            .context("failed to decode Arrow IPC stream")?
        {
            batch = Some(b);
            break;
        }
    }

    let batch = batch.ok_or_else(|| eyre::eyre!("Arrow IPC stream contained no record batches"))?;

    if batch.num_columns() != 1 {
        eyre::bail!(
            "expected 1 column in IPC record batch, got {}",
            batch.num_columns()
        );
    }

    Ok(batch.column(0).to_data())
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{Array, StringArray, UInt64Array};

    #[test]
    fn ipc_roundtrip_primitive() {
        let array = UInt64Array::from(vec![1, 2, 3, 4, 5]);
        let data = array.into_data();
        let encoded = encode_arrow_ipc(&data).unwrap();
        let decoded = decode_arrow_ipc(&encoded).unwrap();
        assert_eq!(data, decoded);
    }

    /// Copy `bytes` into a 128-byte-aligned buffer, mirroring how Dora's
    /// receive path backs IPC payloads (an `AVec<u8, ConstAlign<128>>` for the
    /// daemon path, page-aligned Zenoh SHM for the zero-copy path). This is the
    /// precondition under which `decode_arrow_ipc_zero_copy` aliases the input.
    fn aligned_buffer_from(bytes: &[u8]) -> (arrow::buffer::Buffer, usize, usize) {
        use aligned_vec::{AVec, ConstAlign};
        use std::ptr::NonNull;

        let mut aligned: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, bytes.len());
        aligned.copy_from_slice(bytes);
        let base = aligned.as_ptr() as usize;
        let len = aligned.len();
        let ptr = NonNull::new(aligned.as_ptr() as *mut u8).unwrap();
        // SAFETY: `ptr`/`len` describe `aligned`'s allocation, which the Arc
        // keeps alive for the Buffer's lifetime.
        let buffer = unsafe {
            arrow::buffer::Buffer::from_custom_allocation(ptr, len, std::sync::Arc::new(aligned))
        };
        (buffer, base, len)
    }

    #[test]
    fn ipc_zero_copy_roundtrip_primitive() {
        let array = UInt64Array::from((0..1000u64).collect::<Vec<_>>());
        let data = array.into_data();
        let encoded = encode_arrow_ipc(&data).unwrap();
        let (buffer, _, _) = aligned_buffer_from(&encoded);
        let decoded = decode_arrow_ipc_zero_copy(buffer).unwrap();
        assert_eq!(data, decoded);
    }

    /// The headline claim: for an aligned input buffer the decoded array's data
    /// buffer points *into* the input allocation (no payload copy), and the
    /// strict `require_alignment(true)` decoder accepts it without falling back
    /// to a realigning copy.
    #[test]
    fn ipc_decode_is_zero_copy_for_aligned_buffer() {
        use arrow::ipc::reader::StreamDecoder;

        // A large primitive array so the data buffer dominates and any copy
        // would be unmistakable.
        let array = UInt64Array::from((0..100_000u64).collect::<Vec<_>>());
        let data = array.into_data();
        let encoded = encode_arrow_ipc(&data).unwrap();

        // 1) Proof via the strict decoder: require_alignment(true) errors if any
        //    buffer would need realigning. A clean decode proves the body
        //    buffers are used in place.
        {
            let (mut buffer, _, _) = aligned_buffer_from(&encoded);
            let mut decoder = StreamDecoder::new().with_require_alignment(true);
            let mut got = None;
            while !buffer.is_empty() {
                if let Some(b) = decoder
                    .decode(&mut buffer)
                    .expect("aligned IPC buffer must decode without realignment")
                {
                    got = Some(b);
                    break;
                }
            }
            assert_eq!(got.unwrap().column(0).to_data(), data);
        }

        // 2) Proof via pointer aliasing: the decoded data buffer lies within the
        //    input allocation's address range.
        {
            let (buffer, base, len) = aligned_buffer_from(&encoded);
            let decoded = decode_arrow_ipc_zero_copy(buffer).unwrap();
            let data_ptr = decoded.buffers()[0].as_ptr() as usize;
            assert!(
                data_ptr >= base && data_ptr < base + len,
                "decoded data buffer at {data_ptr:#x} is outside input \
                 [{base:#x}, {:#x}) — a copy happened (not zero-copy)",
                base + len
            );
        }
    }

    /// Production safety: an *under-aligned* input must still decode correctly.
    /// The default decoder (`require_alignment = false`) falls back to copying
    /// only the misaligned buffers rather than erroring.
    #[test]
    fn ipc_zero_copy_decoder_handles_misaligned_input() {
        let array = UInt64Array::from(vec![1, 2, 3, 4, 5, 6, 7, 8]);
        let data = array.into_data();
        let encoded = encode_arrow_ipc(&data).unwrap();

        // Force a 1-byte-offset (deliberately misaligned) backing buffer.
        let mut shifted = Vec::with_capacity(encoded.len() + 1);
        shifted.push(0u8);
        shifted.extend_from_slice(&encoded);
        let buffer = arrow::buffer::Buffer::from_vec(shifted).slice(1);

        let decoded = decode_arrow_ipc_zero_copy(buffer).unwrap();
        assert_eq!(data, decoded);
    }

    #[test]
    fn ipc_roundtrip_string() {
        let array = StringArray::from(vec!["hello", "world"]);
        let data = array.into_data();
        let encoded = encode_arrow_ipc(&data).unwrap();
        let decoded = decode_arrow_ipc(&encoded).unwrap();
        assert_eq!(data, decoded);
    }

    #[test]
    fn ipc_roundtrip_empty_array() {
        let array = UInt64Array::from(Vec::<u64>::new());
        let data = array.into_data();
        let encoded = encode_arrow_ipc(&data).unwrap();
        let decoded = decode_arrow_ipc(&encoded).unwrap();
        assert_eq!(data.len(), decoded.len());
    }

    #[test]
    fn ipc_roundtrip_with_nulls() {
        let array = UInt64Array::from(vec![Some(1), None, Some(3)]);
        let data = array.into_data();
        let encoded = encode_arrow_ipc(&data).unwrap();
        let decoded = decode_arrow_ipc(&encoded).unwrap();
        assert_eq!(data, decoded);
    }
}
