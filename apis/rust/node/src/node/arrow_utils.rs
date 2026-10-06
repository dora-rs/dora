//! Utility functions for converting Arrow arrays to/from raw data.
//!
pub mod ipc_encode;

use aligned_vec::{AVec, ConstAlign};
use arrow::array::ArrayData;
use dora_arrow_convert::{
    DoraArray,
    internal::{array_ref, from_array_data},
};
use eyre::Context;

/// A byte buffer holding an Arrow IPC stream, ready to decode.
///
/// A dora-owned wrapper: the receive path needs to hand the decoder a buffer
/// whose backing allocation it does not copy, but the Arrow buffer type that
/// makes that possible must not appear in dora's frozen public API (it would
/// pin 1.x to one Arrow major — see
/// `docs/plan-arrow-version-decoupling.md`). Construct one from the payload
/// you have; decoding is zero-copy when the payload is 64-byte aligned, which
/// dora's own 128-byte-aligned and page-aligned shared-memory payloads always
/// are.
#[derive(Debug, Clone)]
pub struct IpcPayload(arrow::buffer::Buffer);

impl IpcPayload {
    /// Wrap a 128-byte-aligned payload buffer without copying it.
    pub fn from_aligned_vec(data: AVec<u8, ConstAlign<128>>) -> Self {
        let ptr = std::ptr::NonNull::new(data.as_ptr() as *mut u8)
            .expect("AVec allocation pointer is never null");
        let len = data.len();
        // SAFETY: `ptr`/`len` describe `data`'s allocation, and `data` itself is
        // moved into the `Arc` that owns the buffer, so the allocation outlives
        // every reference the `Buffer` hands out.
        Self(unsafe {
            arrow::buffer::Buffer::from_custom_allocation(ptr, len, std::sync::Arc::new(data))
        })
    }

    /// Take ownership of a `Vec` payload without copying it.
    ///
    /// A plain `Vec` carries no alignment guarantee, so the decoder may have to
    /// realign individual buffers; use [`from_aligned_vec`](Self::from_aligned_vec)
    /// on the hot path.
    pub fn from_vec(data: Vec<u8>) -> Self {
        Self(arrow::buffer::Buffer::from_vec(data))
    }

    /// Copy a payload out of a slice.
    pub fn from_slice(data: &[u8]) -> Self {
        Self(arrow::buffer::Buffer::from_slice_ref(data))
    }

    /// The payload length in bytes.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether the payload is empty.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    /// The payload bytes.
    pub fn as_slice(&self) -> &[u8] {
        self.0.as_slice()
    }

    pub(crate) fn into_arrow(self) -> arrow::buffer::Buffer {
        self.0
    }
}

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
///
/// # Errors
///
/// Returns an error if the encoded IPC stream would exceed 256 MB — the limit
/// every decode path (`decode_arrow_ipc`, `decode_arrow_ipc_zero_copy`, and the
/// streaming input decoder) also enforces. Encoding fails loudly here rather
/// than emitting a payload that every receiver would reject; split the array
/// into smaller batches.
///
/// # Example
///
/// ```
/// # fn main() -> eyre::Result<()> {
/// use dora_node_api::IntoArrow;
/// use dora_node_api::arrow_utils::{decode_arrow_ipc, encode_arrow_ipc};
///
/// let ipc = encode_arrow_ipc(&vec![1u64, 2, 3].into_arrow())?;
///
/// // The stream is self-describing: decoding recovers the original data
/// // without any external type information.
/// let decoded = decode_arrow_ipc(&ipc)?;
/// let values: Vec<u64> = (&decoded).try_into()?;
/// assert_eq!(values, vec![1, 2, 3]);
/// # Ok(())
/// # }
/// ```
pub fn encode_arrow_ipc(array: &DoraArray) -> eyre::Result<Vec<u8>> {
    encode_arrow_ipc_data(&array_ref(array).to_data())
}

/// Same, for dora-internal callers that already hold an [`ArrayData`].
pub(crate) fn encode_arrow_ipc_data(arrow_array: &ArrayData) -> eyre::Result<Vec<u8>> {
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

    // Fail loudly at the producer instead of emitting a stream that every
    // receive path will unconditionally reject. `decode_arrow_ipc`,
    // `decode_arrow_ipc_zero_copy`, and the streaming `InputDecoder` all bail
    // on payloads over `MAX_IPC_BYTES`, and the fast-path encoder refuses
    // oversized arrays too (routing them here). Without this check an
    // oversized array would encode successfully, get sent, and then be
    // silently dropped as undecodable on the consumer with no error on the
    // sending side — see the matching guard in `uint8_layout`.
    if buf.len() > MAX_IPC_BYTES {
        eyre::bail!(
            "Arrow IPC payload too large: {} bytes (max {MAX_IPC_BYTES}); \
             split the output into smaller batches",
            buf.len()
        );
    }
    Ok(buf)
}

/// Decode an Arrow IPC stream byte buffer back into [`ArrayData`].
///
/// Expects the stream to contain exactly one record batch with a single
/// column, as produced by [`encode_arrow_ipc`] (which writes a single batch
/// whose one column is named `"data"`). The array is taken from that column by
/// position; the column *name* is not inspected. Returns an error for an empty,
/// truncated, multi-batch, or otherwise malformed stream, for a stream with
/// trailing bytes after its end-of-stream marker, for a batch whose column
/// count is not exactly one, and for any payload larger than 256 MB.
///
/// The trailing-byte rejection matches [`decode_arrow_ipc_zero_copy`], so both
/// decoders accept and reject exactly the same streams.
///
/// # Example
///
/// ```
/// # fn main() -> eyre::Result<()> {
/// use dora_node_api::IntoArrow;
/// use dora_node_api::arrow_utils::{decode_arrow_ipc, encode_arrow_ipc};
///
/// let ipc = encode_arrow_ipc(&"hello".to_string().into_arrow())?;
/// let decoded = decode_arrow_ipc(&ipc)?;
/// let text: String = (&decoded).try_into()?;
/// assert_eq!(text, "hello");
/// # Ok(())
/// # }
/// ```
pub fn decode_arrow_ipc(ipc_buf: &[u8]) -> eyre::Result<DoraArray> {
    decode_arrow_ipc_data(ipc_buf).map(from_array_data)
}

/// Error for a stream that carries more than one record batch. The encoder
/// always writes exactly one, so extras are malformed (or crafted): both
/// decoders reject them rather than silently returning only the first batch's
/// data, which would drop payload with no error.
const MULTI_BATCH_ERROR: &str =
    "expected exactly one record batch in IPC stream, but found more than one";

/// Error for a stream that carries extra bytes after the single record batch and
/// its end-of-stream marker. The encoder sizes every stream exactly, so trailing
/// bytes are malformed (or crafted). Both decoders reject them, so the daemon
/// (`decode_arrow_ipc_data`) and inter-node (`decode_arrow_ipc_zero_copy_raw`)
/// paths agree on the same input rather than one accepting what the other
/// rejects.
const TRAILING_BYTES_ERROR: &str = "unexpected trailing bytes after the record batch in IPC stream";

/// Extract the single column's [`ArrayData`] from a decoded batch, rejecting a
/// batch whose column count is not exactly one. Shared by both decoders so the
/// column-count check and the extraction cannot drift apart.
fn array_from_single_column_batch(
    batch: arrow::record_batch::RecordBatch,
) -> eyre::Result<ArrayData> {
    if batch.num_columns() != 1 {
        eyre::bail!(
            "expected 1 column in IPC record batch, got {}",
            batch.num_columns()
        );
    }
    Ok(batch.column(0).to_data())
}

/// Same, for dora-internal callers that want the raw [`ArrayData`].
pub(crate) fn decode_arrow_ipc_data(ipc_buf: &[u8]) -> eyre::Result<ArrayData> {
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

    // Inspect what follows the first batch. A second decoded batch is a
    // multi-batch stream; a decode *error* on the next message must be reported
    // as such rather than mislabeled as "more than one batch"; `None` is the
    // end-of-stream marker.
    match reader.next() {
        Some(Ok(_)) => eyre::bail!(MULTI_BATCH_ERROR),
        Some(Err(e)) => {
            return Err(e).context("failed to read RecordBatch from IPC stream");
        }
        None => {}
    }

    // Reject bytes after the end-of-stream marker so this path agrees with the
    // zero-copy decoder (see `TRAILING_BYTES_ERROR`). `StreamReader` stops at the
    // marker and would otherwise silently ignore anything after it; the inner
    // `Cursor`'s position is how many bytes it consumed reaching that point.
    let consumed = reader.get_ref().position() as usize;
    if consumed != ipc_buf.len() {
        eyre::bail!(TRAILING_BYTES_ERROR);
    }

    array_from_single_column_batch(batch)
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
///
/// # Errors
///
/// Like [`decode_arrow_ipc`], returns an error for an empty, truncated,
/// multi-batch, or otherwise malformed stream, for a stream with trailing bytes
/// after its end-of-stream marker, for a batch whose column count is not exactly
/// one, and for any payload larger than 256 MB — it never panics on bad input.
/// It accepts and rejects exactly the same streams as [`decode_arrow_ipc`].
///
/// # Example
///
/// ```
/// # fn main() -> eyre::Result<()> {
/// use dora_node_api::IntoArrow;
/// use dora_node_api::arrow_utils::{IpcPayload, decode_arrow_ipc_zero_copy, encode_arrow_ipc};
///
/// let ipc = encode_arrow_ipc(&vec![1u64, 2, 3].into_arrow())?;
/// let decoded = decode_arrow_ipc_zero_copy(IpcPayload::from_vec(ipc))?;
/// let values: Vec<u64> = (&decoded).try_into()?;
/// assert_eq!(values, vec![1, 2, 3]);
///
/// // A malformed stream is rejected with an error rather than panicking.
/// assert!(decode_arrow_ipc_zero_copy(IpcPayload::from_vec(vec![0u8; 8])).is_err());
/// # Ok(())
/// # }
/// ```
pub fn decode_arrow_ipc_zero_copy(payload: IpcPayload) -> eyre::Result<DoraArray> {
    decode_arrow_ipc_zero_copy_raw(payload.into_arrow()).map(from_array_data)
}

/// Same, for dora-internal callers that already hold an Arrow buffer.
pub(crate) fn decode_arrow_ipc_zero_copy_raw(
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
    // exhaust the input. Keep draining after the first batch so a second one is
    // detected and rejected rather than silently dropped (the encoder always
    // writes exactly one batch; a stream with more is malformed or crafted).
    while !buffer.is_empty() {
        let before = buffer.len();
        let decoded = match decoder.decode(&mut buffer) {
            Ok(decoded) => decoded,
            Err(e) => {
                // `decode` consumes the 8-byte end-of-stream marker and enters
                // its `Finished` state, then errors on any byte after it. So
                // once we already hold the batch, a decode error means the
                // stream had extra bytes after a complete single-batch stream:
                // report that as trailing garbage (matching the `StreamReader`
                // path) rather than a generic decode failure. Before the batch,
                // it is a genuinely malformed/truncated stream.
                let context = if batch.is_some() {
                    TRAILING_BYTES_ERROR
                } else {
                    "failed to decode Arrow IPC stream"
                };
                return Err::<arrow::array::ArrayData, _>(e).context(context);
            }
        };
        match decoded {
            Some(b) => {
                // `replace` both stores the batch and tells us whether one was
                // already present, so a second batch is detected and rejected
                // rather than silently dropped.
                if batch.replace(b).is_some() {
                    eyre::bail!(MULTI_BATCH_ERROR);
                }
            }
            None if buffer.len() == before => {
                // `decode` yielded no batch and consumed nothing. Before the
                // first batch this is a crafted/truncated payload that would
                // otherwise spin this loop forever on the zenoh IO worker
                // (surfaced below as "no record batches"), so stop. `decode`
                // otherwise only returns `None` once it has drained the buffer,
                // so trailing bytes after the marker never reach here — they hit
                // the `Err` arm above.
                break;
            }
            // Progress without a batch: the schema message before the batch, or
            // the end-of-stream marker after it. Keep going; the loop ends when
            // the buffer is drained.
            None => {}
        }
    }

    let batch = batch.ok_or_else(|| eyre::eyre!("Arrow IPC stream contained no record batches"))?;

    array_from_single_column_batch(batch)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{Array, StringArray, UInt64Array};

    #[test]
    fn ipc_roundtrip_primitive() {
        let array = UInt64Array::from(vec![1, 2, 3, 4, 5]);
        let data = array.into_data();
        let encoded = encode_arrow_ipc_data(&data).unwrap();
        let decoded = decode_arrow_ipc_data(&encoded).unwrap();
        assert_eq!(data, decoded);
    }

    /// Build an IPC stream that (invalidly) carries the same single-column
    /// batch twice, to exercise the multi-batch rejection on both decoders.
    fn two_batch_ipc_stream() -> Vec<u8> {
        use arrow::array::ArrayRef;
        use arrow::ipc::writer::StreamWriter;
        use arrow::record_batch::RecordBatch;
        use arrow_schema::{Field, Schema};
        use std::sync::Arc;

        let array = UInt64Array::from(vec![1, 2, 3]);
        let schema = Arc::new(Schema::new(vec![Field::new(
            "data",
            array.data_type().clone(),
            true,
        )]));
        let batch =
            RecordBatch::try_new(schema.clone(), vec![Arc::new(array) as ArrayRef]).unwrap();

        let mut buf = Vec::new();
        {
            let mut writer = StreamWriter::try_new(&mut buf, &schema).unwrap();
            writer.write(&batch).unwrap();
            writer.write(&batch).unwrap();
            writer.finish().unwrap();
        }
        buf
    }

    /// A stream carrying more than one record batch must be rejected rather
    /// than silently decoded down to its first batch, which would drop the rest
    /// of the payload with no error. `decode_arrow_ipc` documents "exactly one
    /// record batch".
    #[test]
    fn ipc_decode_rejects_multiple_batches() {
        let buf = two_batch_ipc_stream();
        let err =
            decode_arrow_ipc_data(&buf).expect_err("a multi-batch IPC stream must be rejected");
        assert!(
            err.to_string().contains("more than one"),
            "unexpected error: {err}"
        );
    }

    /// The zero-copy decoder backs the actual inter-node receive path, so it
    /// must reject a multi-batch stream too rather than silently returning only
    /// the first batch.
    #[test]
    fn ipc_zero_copy_decode_rejects_multiple_batches() {
        let buf = two_batch_ipc_stream();
        let err = decode_arrow_ipc_zero_copy_raw(arrow::buffer::Buffer::from_vec(buf))
            .expect_err("a multi-batch IPC stream must be rejected");
        assert!(
            err.to_string().contains("more than one"),
            "unexpected error: {err}"
        );
    }

    /// A valid single-batch stream with extra bytes appended after its
    /// end-of-stream marker must be rejected by *both* decoders. The arrow-ipc
    /// `StreamDecoder` consumes the 8-byte end-of-stream marker and then errors
    /// on any further byte, and `StreamReader` stops at the marker; without an
    /// explicit check the two paths would disagree (one rejecting, one silently
    /// ignoring the tail). No in-tree sender produces trailing bytes, so
    /// rejecting them keeps the two decoders in lockstep on the same input.
    #[test]
    fn ipc_decode_rejects_trailing_bytes() {
        let array = UInt64Array::from(vec![1, 2, 3]);
        let mut encoded = encode_arrow_ipc_data(&array.into_data()).unwrap();
        // Distinct from the `0xFF..` end-of-stream continuation marker, so these
        // are unambiguously bytes *after* a complete stream.
        encoded.extend_from_slice(&[0u8; 16]);

        let err = decode_arrow_ipc_data(&encoded)
            .expect_err("a stream with trailing bytes must be rejected (StreamReader)");
        assert!(
            err.to_string().contains("trailing bytes"),
            "unexpected error: {err}"
        );

        let err = decode_arrow_ipc_zero_copy_raw(arrow::buffer::Buffer::from_vec(encoded))
            .expect_err("a stream with trailing bytes must be rejected (zero-copy)");
        assert!(
            err.to_string().contains("trailing bytes"),
            "unexpected error: {err}"
        );
    }

    /// An array whose IPC stream exceeds `MAX_IPC_BYTES` must fail at encode
    /// time. Otherwise the sender would emit a stream that every receive path
    /// rejects, silently dropping the message with no producer-side error.
    #[test]
    fn ipc_encode_rejects_oversized_payload() {
        use arrow::array::UInt8Array;

        // Body just over the 256 MB cap; the framing pushes the stream over too.
        let array = UInt8Array::from(vec![0u8; MAX_IPC_BYTES + 1]);
        let data = array.into_data();
        let err = encode_arrow_ipc_data(&data)
            .expect_err("oversized payload must be rejected by the encoder");
        assert!(
            err.to_string().contains("too large"),
            "unexpected error: {err}"
        );
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
        let encoded = encode_arrow_ipc_data(&data).unwrap();
        let (buffer, _, _) = aligned_buffer_from(&encoded);
        let decoded = decode_arrow_ipc_zero_copy_raw(buffer).unwrap();
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
        let encoded = encode_arrow_ipc_data(&data).unwrap();

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
            let decoded = decode_arrow_ipc_zero_copy_raw(buffer).unwrap();
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
        let encoded = encode_arrow_ipc_data(&data).unwrap();

        // Force a 1-byte-offset (deliberately misaligned) backing buffer.
        let mut shifted = Vec::with_capacity(encoded.len() + 1);
        shifted.push(0u8);
        shifted.extend_from_slice(&encoded);
        let buffer = arrow::buffer::Buffer::from_vec(shifted).slice(1);

        let decoded = decode_arrow_ipc_zero_copy_raw(buffer).unwrap();
        assert_eq!(data, decoded);
    }

    #[test]
    fn ipc_roundtrip_string() {
        let array = StringArray::from(vec!["hello", "world"]);
        let data = array.into_data();
        let encoded = encode_arrow_ipc_data(&data).unwrap();
        let decoded = decode_arrow_ipc_data(&encoded).unwrap();
        assert_eq!(data, decoded);
    }

    #[test]
    fn ipc_roundtrip_empty_array() {
        let array = UInt64Array::from(Vec::<u64>::new());
        let data = array.into_data();
        let encoded = encode_arrow_ipc_data(&data).unwrap();
        let decoded = decode_arrow_ipc_data(&encoded).unwrap();
        assert_eq!(data.len(), decoded.len());
    }

    /// A zero-length *typed* array must encode to a self-describing stream that
    /// decodes back to the SAME type, not `Null`. record/replay relies on this:
    /// `record-node` IPC-encodes empty typed arrays (rather than dropping them
    /// to an absent payload) so replay preserves the type instead of collapsing
    /// to `NullArray::new(0)` (#2027/#2083).
    #[test]
    fn ipc_roundtrip_empty_typed_array_preserves_type() {
        use arrow::array::Float32Array;
        let data = Float32Array::from(Vec::<f32>::new()).into_data();
        let encoded = encode_arrow_ipc_data(&data).unwrap();
        let decoded = decode_arrow_ipc_data(&encoded).unwrap();
        assert_eq!(decoded.data_type(), &arrow_schema::DataType::Float32);
        assert_eq!(decoded.len(), 0);
    }

    #[test]
    fn ipc_roundtrip_with_nulls() {
        let array = UInt64Array::from(vec![Some(1), None, Some(3)]);
        let data = array.into_data();
        let encoded = encode_arrow_ipc_data(&data).unwrap();
        let decoded = decode_arrow_ipc_data(&encoded).unwrap();
        assert_eq!(data, decoded);
    }
}
