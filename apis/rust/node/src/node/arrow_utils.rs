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

    // A received array arrow accepted can still make its writer panic (a
    // corrupted run-end-encoded array), and nodes send received data on.
    let buf = catch_panic("failed to encode Arrow IPC stream: arrow panicked", || {
        let array_ref = arrow::array::make_array(arrow_array.clone());
        let batch = RecordBatch::try_new(schema_ref.clone(), vec![array_ref])
            .context("failed to create RecordBatch for IPC encoding")?;
        let mut buf = Vec::new();
        let mut writer = StreamWriter::try_new(&mut buf, &schema_ref)
            .context("failed to create Arrow IPC StreamWriter")?;
        writer
            .write(&batch)
            .context("failed to write RecordBatch to IPC stream")?;
        writer
            .finish()
            .context("failed to finish Arrow IPC stream")?;
        drop(writer);
        Ok(buf)
    })?;

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
    // Same for row counts: the receivers' own check, run on what was written
    // (a slice writes only the values it uses, not its parent's).
    check_row_counts(&buf).context("cannot send this array")?;
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
///
/// Copies the bytes into a [`Buffer`](arrow::buffer::Buffer) and runs the
/// zero-copy decoder on it, so both decoders accept and reject exactly the same
/// streams. arrow's `StreamReader` is not used: it allocates the size a message
/// claims before reading it, so a corrupt length aborts the process.
pub(crate) fn decode_arrow_ipc_data(ipc_buf: &[u8]) -> eyre::Result<ArrayData> {
    if ipc_buf.len() > MAX_IPC_BYTES {
        eyre::bail!(
            "Arrow IPC payload too large: {} bytes (max {MAX_IPC_BYTES})",
            ipc_buf.len()
        );
    }
    decode_arrow_ipc_zero_copy_raw(arrow::buffer::Buffer::from(ipc_buf))
}

/// Decode an Arrow IPC stream from an Arrow [`Buffer`] **without copying** the
/// payload buffers when they are properly aligned.
///
/// Unlike [`decode_arrow_ipc`], which first copies its byte slice into a new
/// buffer, this runs [`arrow::ipc::reader::StreamDecoder`] directly on the
/// provided [`Buffer`] and slices the array buffers out of it. When the input
/// buffer is suitably aligned — as Dora's shared-memory payloads always are
/// (128-byte `AVec` / page-aligned Zenoh SHM) — the decoded array aliases the
/// input and no payload copy happens.
///
/// An input that does not start on an 8-byte boundary (e.g. a borrowed zenoh
/// network payload) is first copied once into an aligned buffer; with an
/// 8-byte-aligned input, the decoder's default `require_alignment = false`
/// copies only a buffer that a type needs more aligned. This keeps the receive
/// path robust while preserving zero-copy for the common SHM case.
///
/// # Errors
///
/// Like [`decode_arrow_ipc`], returns an error for an empty, truncated,
/// multi-batch, or otherwise malformed stream, for a stream with trailing bytes
/// after its end-of-stream marker, for a batch whose column count is not exactly
/// one, and for any payload larger than 256 MB — it never panics on bad input
/// (a panic inside arrow is caught, so this needs the default `panic = "unwind"`).
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
    buffer: arrow::buffer::Buffer,
) -> eyre::Result<arrow::array::ArrayData> {
    if buffer.len() > MAX_IPC_BYTES {
        eyre::bail!(
            "Arrow IPC payload too large: {} bytes (max {MAX_IPC_BYTES})",
            buffer.len()
        );
    }
    let buffer = align_for_decode(buffer);
    check_row_counts(buffer.as_slice())?;
    catch_arrow_panic(|| decode_single_batch(buffer))
}

/// Most rows an array in an accepted payload can have: one per bit of the
/// largest accepted payload. Most arrays store at least one bit per row; the
/// compact ones (a `Null` array, a run-end-encoded array with long runs) can
/// claim more, so the limit is also enforced when encoding.
const MAX_IPC_ROWS: i64 = MAX_IPC_BYTES as i64 * 8;

/// Read an IPC message header, refusing one that grows to more than 4 times
/// its size when read. Flatbuffers lets one string or table be referenced many
/// times and arrow builds a copy for each reference, so a small header could
/// otherwise make arrow allocate far more than the payload; the headers dora
/// and arrow write grow less than 2 times (shared vtables).
pub(crate) fn read_message_header(metadata: &[u8]) -> eyre::Result<arrow::ipc::Message<'_>> {
    let options = flatbuffers::VerifierOptions {
        max_apparent_size: metadata.len().saturating_mul(4),
        ..Default::default()
    };
    arrow::ipc::root_as_message_with_opts(&options, metadata)
        .map_err(|e| eyre::eyre!("invalid Arrow IPC message header: {e}"))
}

/// Rows of all columns of one message together, saturating.
fn sum_of_rows(rows: impl Iterator<Item = i64>) -> i64 {
    rows.fold(0, i64::saturating_add)
}

/// Reject a record batch (or dictionary batch) in `stream` that claims a row
/// or null count below 0 or above [`MAX_IPC_ROWS`], overall, in any column or
/// in all columns together, whose buffers don't fit in its body, or that is
/// compressed, and reject a delta dictionary batch.
///
/// arrow validates some arrays by allocating per claimed row before it can see
/// that the data doesn't back the claim (a struct with a non-nullable `Null`
/// child allocates a bitmap for every row), and a failed allocation aborts the
/// process, which no panic catch can stop. So a crafted row count must be
/// rejected before arrow sees the message. Anything else malformed is left to
/// the decoder, which reports it.
pub(crate) fn check_row_counts(stream: &[u8]) -> eyre::Result<()> {
    let read_i32 = |at: usize| -> Option<i32> {
        Some(i32::from_le_bytes(stream.get(at..at + 4)?.try_into().ok()?))
    };
    let mut at = 0usize;
    loop {
        // A message is an optional 0xFFFFFFFF continuation marker, the metadata
        // length, the metadata (a flatbuffer) and the body.
        let Some(mut metadata_len) = read_i32(at) else {
            return Ok(());
        };
        if metadata_len == -1 {
            at += 4;
            let Some(len) = read_i32(at) else {
                return Ok(());
            };
            metadata_len = len;
        }
        let Ok(metadata_len) = usize::try_from(metadata_len) else {
            return Ok(());
        };
        if metadata_len == 0 {
            return Ok(()); // end-of-stream marker
        }
        let start = at + 4;
        let Some(metadata) = start
            .checked_add(metadata_len)
            .and_then(|end| stream.get(start..end))
        else {
            return Ok(());
        };
        // A header arrow would also refuse is an error either way.
        let message = read_message_header(metadata)?;
        let dictionary = message.header_as_dictionary_batch();
        if dictionary.is_some_and(|dictionary| dictionary.isDelta()) {
            // A delta adds to the dictionary a decoder keeps between messages,
            // so it could grow without limit; dora never sends one.
            eyre::bail!("Arrow IPC delta dictionary batches are not supported");
        }
        let batch = message
            .header_as_record_batch()
            .or_else(|| dictionary.and_then(|dictionary| dictionary.data()));
        if let Some(batch) = batch {
            if batch.compression().is_some() {
                // arrow reserves the size a compressed buffer claims before
                // reading it (when a crate enables arrow's compression), and
                // dora never compresses.
                eyre::bail!("compressed Arrow IPC messages are not supported");
            }
            // arrow casts these to `usize`, so a negative one is as bad as a
            // huge one.
            let columns = batch.nodes().into_iter().flatten();
            let counts = std::iter::once(batch.length())
                .chain(columns.flat_map(|node| [node.length(), node.null_count()]));
            for rows in counts {
                if !(0..=MAX_IPC_ROWS).contains(&rows) {
                    eyre::bail!(
                        "Arrow IPC message claims {rows} rows, outside the 0..={MAX_IPC_ROWS} \
                         rows a payload of at most {MAX_IPC_BYTES} bytes can hold"
                    );
                }
            }
            // arrow can build a bitmap per row for each column (a union keeps
            // one per child), so all columns together get the limit too.
            let rows = sum_of_rows(
                batch
                    .nodes()
                    .into_iter()
                    .flatten()
                    .map(|node| node.length()),
            );
            if rows > MAX_IPC_ROWS {
                eyre::bail!(
                    "Arrow IPC message claims {rows} rows across its columns, more than the \
                     {MAX_IPC_ROWS} a payload of at most {MAX_IPC_BYTES} bytes can hold"
                );
            }
            // Each buffer must lie inside the body, and together they may not be
            // longer than it: arrow copies an unaligned buffer once for every
            // column that points at it, so buffers sharing bytes would make a
            // small message allocate many times its size.
            let body = message.bodyLength();
            let mut total = 0i64;
            for buffer in batch.buffers().into_iter().flatten() {
                let (offset, length) = (buffer.offset(), buffer.length());
                let end = offset.checked_add(length);
                total = total.saturating_add(length);
                if offset < 0 || length < 0 || end.is_none_or(|end| end > body) || total > body {
                    eyre::bail!("Arrow IPC message buffers don't fit in its {body}-byte body");
                }
            }
        }
        let Ok(body_len) = usize::try_from(message.bodyLength()) else {
            return Ok(());
        };
        match (start + metadata_len).checked_add(body_len) {
            Some(next) if next <= stream.len() => at = next,
            _ => return Ok(()),
        }
    }
}

/// Copy `buffer` into a new aligned buffer if it does not start on an 8-byte
/// boundary; otherwise return it unchanged (no copy).
///
/// The IPC format pads every array buffer to 8 bytes from the start of the
/// message, so an 8-byte-aligned input keeps every buffer 8-byte aligned. Below
/// that, arrow realigns one array buffer at a time and panics doing it for a
/// dense union. A borrowed zenoh network payload can start at any address;
/// shared-memory and `AVec` payloads are at least 64-byte aligned and are never
/// copied.
pub(crate) fn align_for_decode(buffer: arrow::buffer::Buffer) -> arrow::buffer::Buffer {
    if buffer.as_ptr().align_offset(8) == 0 {
        buffer
    } else {
        arrow::buffer::Buffer::from(buffer.as_slice())
    }
}

/// Run an arrow-ipc decode, turning a panic into an error.
///
/// arrow 59 (arrow-ipc, arrow-buffer, arrow-data) has `unwrap`s and asserts that
/// panic on some malformed messages, and every decode here reads bytes from a
/// peer or a file, so a corrupt payload must be an error, not a crash. This
/// needs the default `panic = "unwind"`.
pub(crate) fn catch_arrow_panic<T>(decode: impl FnOnce() -> eyre::Result<T>) -> eyre::Result<T> {
    catch_panic(
        "failed to decode Arrow IPC stream: the decoder panicked",
        decode,
    )
}

/// Run `f`, turning a panic into the error `"{what}: {reason}"`.
fn catch_panic<T>(what: &str, f: impl FnOnce() -> eyre::Result<T>) -> eyre::Result<T> {
    std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)).unwrap_or_else(|panic| {
        let reason = panic
            .downcast_ref::<&str>()
            .copied()
            .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
            .unwrap_or("unknown reason");
        eyre::bail!("{what}: {reason}")
    })
}

fn decode_single_batch(mut buffer: arrow::buffer::Buffer) -> eyre::Result<ArrayData> {
    use arrow::ipc::reader::StreamDecoder;

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
                // report that as trailing garbage rather than a generic decode
                // failure. Before the batch, it is a genuinely
                // malformed/truncated stream.
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

    // A stream that ends inside a message (for example inside a record batch's
    // body, or halfway through the end-of-stream marker) is truncated.
    decoder
        .finish()
        .context("truncated Arrow IPC stream: it ends inside a message")?;
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

    /// A valid stream on a buffer that is not aligned (a borrowed zenoh network
    /// payload can start at any address) decodes like on an aligned one. arrow's
    /// own realignment panics on a dense union (#3863).
    #[test]
    fn misaligned_dense_union_decodes() {
        use arrow::array::{Int32Array, UnionArray};
        use arrow::buffer::ScalarBuffer;
        use arrow_schema::{DataType, Field, UnionFields};

        let fields: UnionFields = [
            (
                0i8,
                std::sync::Arc::new(Field::new("a", DataType::Int32, false)),
            ),
            (
                1i8,
                std::sync::Arc::new(Field::new("b", DataType::Utf8, false)),
            ),
        ]
        .into_iter()
        .collect();
        let union = UnionArray::try_new(
            fields,
            ScalarBuffer::from(vec![0i8, 1, 0]),
            Some(ScalarBuffer::from(vec![0i32, 0, 1])),
            vec![
                std::sync::Arc::new(Int32Array::from(vec![10, 20])),
                std::sync::Arc::new(StringArray::from(vec!["x"])),
            ],
        )
        .unwrap();
        let data = union.into_data();
        let encoded = encode_arrow_ipc_data(&data).unwrap();

        let mut shifted = vec![0u8; encoded.len() + 1];
        shifted[1..].copy_from_slice(&encoded);
        let misaligned = arrow::buffer::Buffer::from_vec(shifted).slice(1);
        assert_ne!(misaligned.as_ptr() as usize % 8, 0);

        assert_eq!(decode_arrow_ipc_zero_copy_raw(misaligned).unwrap(), data);
        assert_eq!(decode_arrow_ipc_data(&encoded).unwrap(), data);
    }

    /// A buffer that is 8-byte aligned (what the IPC format needs) is decoded in
    /// place, not copied, even when it is not aligned to arrow's 64/128-byte
    /// allocation alignment: dora's shared memory only guarantees 64 bytes.
    #[test]
    fn an_eight_byte_aligned_buffer_is_not_copied() {
        let data = UInt64Array::from(vec![1u64, 2, 3, 4]).into_data();
        let encoded = encode_arrow_ipc_data(&data).unwrap();
        let mut padded = vec![0u8; encoded.len() + 8];
        padded[8..].copy_from_slice(&encoded);
        let input = arrow::buffer::Buffer::from_vec(padded).slice(8);
        assert_eq!(input.as_ptr() as usize % 8, 0);
        assert_ne!(input.as_ptr() as usize % 64, 0);
        let range = input.as_ptr() as usize..input.as_ptr() as usize + input.len();

        let decoded = decode_arrow_ipc_zero_copy_raw(input).unwrap();
        assert_eq!(decoded, data);
        let values = decoded.buffers()[0].as_ptr() as usize;
        assert!(range.contains(&values), "the values were copied");
    }

    /// A valid single-batch stream with extra bytes appended after its
    /// end-of-stream marker must be rejected by *both* decoders. The arrow-ipc
    /// `StreamDecoder` both use consumes the 8-byte end-of-stream marker and
    /// then errors on any further byte. No in-tree sender produces trailing
    /// bytes.
    #[test]
    fn ipc_decode_rejects_trailing_bytes() {
        let array = UInt64Array::from(vec![1, 2, 3]);
        let mut encoded = encode_arrow_ipc_data(&array.into_data()).unwrap();
        // Distinct from the `0xFF..` end-of-stream continuation marker, so these
        // are unambiguously bytes *after* a complete stream.
        encoded.extend_from_slice(&[0u8; 16]);

        let err = decode_arrow_ipc_data(&encoded)
            .expect_err("a stream with trailing bytes must be rejected");
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

    /// Production safety: an *under-aligned* input must still decode correctly
    /// (it is copied into an aligned buffer first) rather than erroring.
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
