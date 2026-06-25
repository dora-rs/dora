//! Hand-rolled, 1-copy Arrow IPC stream encoder.
//!
//! Arrow's official [`StreamWriter`](arrow::ipc::writer::StreamWriter) always
//! stages the record-batch body in an internal `Vec` before writing it out, so
//! encoding a message through it copies the payload at least twice. This module
//! provides a *fast path* that writes the IPC flatbuffer headers and copies each
//! array buffer **directly into a caller-provided `&mut [u8]`** — exactly one
//! copy of the payload, straight into the (shared-memory) sample. For types the
//! fast path does not handle it falls back to the official writer.
//!
//! The fast-path output is a normal Arrow IPC stream and decodes through the
//! official [`StreamDecoder`](arrow::ipc::reader::StreamDecoder) — including the
//! zero-copy [`decode_arrow_ipc_zero_copy`](super::decode_arrow_ipc_zero_copy)
//! receive path — because every body buffer is placed at a 64-byte-aligned
//! offset, matching what the official writer produces with its default
//! `alignment = 64`.
//!
//! ## How the fast path stays correct without slice-truncation logic
//!
//! Arrow's writer contains a lot of per-type code to *truncate* buffers for
//! sliced arrays. We sidestep all of it with two rules:
//!  * **Require `offset() == 0` on every node** — so logical element `i` lives at
//!    physical position `i`. Any array (or child) with a non-zero offset routes
//!    to the fallback.
//!  * **Copy each data buffer in full.** Arrow tolerates buffers that are larger
//!    than strictly required for `len` elements, so copying the whole buffer (a
//!    freshly built array's buffers are exactly sized anyway) always decodes to
//!    a logically-equal array. The only generated buffer is the all-ones
//!    validity bitmap for a node with no nulls, exactly as arrow emits.
//!
//! `FixedSizeList` is the one type whose child length is implied rather than
//! offset-encoded, so its child is sliced to `len * value_size` before
//! recursion.

use arrow::array::ArrayData;
use arrow::buffer::Buffer as ArrowBuffer;
use arrow::ipc::writer::{DictionaryTracker, IpcDataGenerator, IpcWriteOptions};
use arrow::ipc::{Buffer as IpcBuffer, FieldNode, MessageHeader, MetadataVersion};
use arrow_schema::{DataType, Field, Schema};
use eyre::{Context, bail, eyre};

use super::ARROW_BUFFER_ALIGNMENT as ALIGN;

/// IPC stream continuation marker (precedes every message length prefix).
const CONTINUATION_MARKER: [u8; 4] = [0xff, 0xff, 0xff, 0xff];
/// Bytes of the message length/continuation prefix (continuation + i32 length).
const PREFIX_LEN: usize = 8;

#[inline]
fn round_up(n: usize, align: usize) -> usize {
    debug_assert!(align.is_power_of_two());
    (n + align - 1) & !(align - 1)
}

/// Whether the fast path handles this `DataType` directly (per node — children
/// are validated by recursion). Everything else uses the official-writer
/// fallback: dictionary, any `*View`, `Union`, `Map`, run-end-encoded, etc.
fn is_fast_path_type(data_type: &DataType) -> bool {
    use DataType::*;
    matches!(
        data_type,
        Null | Boolean
            | Int8
            | Int16
            | Int32
            | Int64
            | UInt8
            | UInt16
            | UInt32
            | UInt64
            | Float16
            | Float32
            | Float64
            | Timestamp(_, _)
            | Date32
            | Date64
            | Time32(_)
            | Time64(_)
            | Duration(_)
            | Interval(_)
            | Decimal128(_, _)
            | Decimal256(_, _)
            | FixedSizeBinary(_)
            | Binary
            | LargeBinary
            | Utf8
            | LargeUtf8
            | List(_)
            | LargeList(_)
            | FixedSizeList(_, _)
            | Struct(_)
    )
}

/// One body buffer to emit: a 64-aligned body offset, its byte length, and where
/// the bytes come from.
struct Desc {
    offset: usize,
    len: usize,
    src: BufferSrc,
}

enum BufferSrc {
    /// Copy `len` bytes from this buffer starting at the given byte offset.
    Bytes(ArrowBuffer, usize),
    /// Fill `len` bytes with `0xff` (an all-valid validity bitmap).
    AllOnes,
}

#[derive(Default)]
struct Layout {
    nodes: Vec<FieldNode>,
    ipc_buffers: Vec<IpcBuffer>,
    descs: Vec<Desc>,
    body_len: usize,
}

impl Layout {
    fn push_buffer(&mut self, off: &mut usize, len: usize, src: BufferSrc) {
        // `*off` is always 64-aligned here (the invariant `push_buffer` keeps).
        self.ipc_buffers
            .push(IpcBuffer::new(*off as i64, len as i64));
        self.descs.push(Desc {
            offset: *off,
            len,
            src,
        });
        *off += round_up(len, ALIGN);
    }
}

/// Walk `array` building the IPC field-node list, buffer descriptors, and body
/// length. Returns `None` if any node is not fast-path eligible (unsupported
/// type or non-zero offset), so the caller falls back to the official writer.
fn build_layout(array: &ArrayData) -> Option<Layout> {
    let mut layout = Layout::default();
    let mut off = 0usize;
    build_layout_rec(array, &mut layout, &mut off)?;
    layout.body_len = off;
    Some(layout)
}

fn build_layout_rec(array: &ArrayData, layout: &mut Layout, off: &mut usize) -> Option<()> {
    let data_type = array.data_type();
    if !is_fast_path_type(data_type) || array.offset() != 0 {
        return None;
    }

    let len = array.len();
    // NullArray reports `null_count == 0` on its `ArrayData`, but the IPC field
    // node records every element as null (matching arrow's writer).
    let null_count = if matches!(data_type, DataType::Null) {
        len
    } else {
        array.null_count()
    };
    layout
        .nodes
        .push(FieldNode::new(len as i64, null_count as i64));

    // Validity bitmap (every type except `Null` carries one in IPC V5; the
    // fast-path set excludes the other no-validity types — Union/RunEndEncoded).
    if !matches!(data_type, DataType::Null) {
        match array.nulls() {
            Some(nulls) => {
                if nulls.inner().offset() != 0 {
                    return None;
                }
                let sliced = nulls.inner().sliced();
                let bytes = sliced.len();
                layout.push_buffer(off, bytes, BufferSrc::Bytes(sliced, 0));
            }
            None => {
                let bytes = len.div_ceil(8);
                layout.push_buffer(off, bytes, BufferSrc::AllOnes);
            }
        }
    }

    // Data buffers, copied in full (empty for Struct/FixedSizeList).
    for buffer in array.buffers() {
        layout.push_buffer(off, buffer.len(), BufferSrc::Bytes(buffer.clone(), 0));
    }

    // Children.
    match data_type {
        DataType::FixedSizeList(_, value_size) => {
            // The child length is implied (`len * value_size`), not carried by
            // an offsets buffer, so slice it to exactly that before recursing.
            let n = len.checked_mul(*value_size as usize)?;
            let child = array.child_data().first()?;
            if child.len() < n {
                return None;
            }
            build_layout_rec(&child.slice(0, n), layout, off)?;
        }
        _ => {
            for child in array.child_data() {
                build_layout_rec(child, layout, off)?;
            }
        }
    }

    Some(())
}

/// Everything needed to write the stream, plus the exact total length.
struct Prepared {
    layout: Layout,
    schema_message: Vec<u8>,
    record_batch_message: Vec<u8>,
    schema_block: usize,
    record_batch_block: usize,
    total: usize,
}

fn ipc_write_options() -> eyre::Result<IpcWriteOptions> {
    IpcWriteOptions::try_new(ALIGN, false, MetadataVersion::V5)
        .map_err(|e| eyre!("failed to build Arrow IPC write options: {e}"))
}

/// Build the schema IPC message flatbuffer (one nullable field named `data`),
/// matching what `encode_arrow_ipc` / the official writer emit.
fn build_schema_message(data_type: &DataType) -> eyre::Result<Vec<u8>> {
    let schema = Schema::new(vec![Field::new("data", data_type.clone(), true)]);
    let options = ipc_write_options()?;
    let mut tracker = DictionaryTracker::new(false);
    let encoded = IpcDataGenerator {}.schema_to_bytes_with_dictionary_tracker(
        &schema,
        &mut tracker,
        &options,
    );
    Ok(encoded.ipc_message)
}

/// Hand-build the RecordBatch IPC message flatbuffer (header only — no body),
/// mirroring arrow's `record_batch_to_bytes`.
fn build_record_batch_message(
    num_rows: usize,
    nodes: &[FieldNode],
    buffers: &[IpcBuffer],
    body_len: usize,
) -> Vec<u8> {
    use flatbuffers::FlatBufferBuilder;

    let mut fbb = FlatBufferBuilder::new();
    let buffers_fb = fbb.create_vector(buffers);
    let nodes_fb = fbb.create_vector(nodes);

    let record_batch = {
        let mut builder = arrow::ipc::RecordBatchBuilder::new(&mut fbb);
        builder.add_length(num_rows as i64);
        builder.add_nodes(nodes_fb);
        builder.add_buffers(buffers_fb);
        builder.finish()
    };

    let message = {
        let mut builder = arrow::ipc::MessageBuilder::new(&mut fbb);
        builder.add_version(MetadataVersion::V5);
        builder.add_header_type(MessageHeader::RecordBatch);
        builder.add_bodyLength(body_len as i64);
        builder.add_header(record_batch.as_union_value());
        builder.finish()
    };

    fbb.finish(message, None);
    fbb.finished_data().to_vec()
}

fn prepare(array: &ArrayData) -> Option<Prepared> {
    let layout = build_layout(array)?;
    let schema_message = build_schema_message(array.data_type()).ok()?;
    let record_batch_message = build_record_batch_message(
        array.len(),
        &layout.nodes,
        &layout.ipc_buffers,
        layout.body_len,
    );
    let schema_block = round_up(PREFIX_LEN + schema_message.len(), ALIGN);
    let record_batch_block = round_up(PREFIX_LEN + record_batch_message.len(), ALIGN);
    // schema block + record-batch header block + body + end-of-stream (8 bytes).
    let total = schema_block + record_batch_block + layout.body_len + PREFIX_LEN;
    Some(Prepared {
        layout,
        schema_message,
        record_batch_message,
        schema_block,
        record_batch_block,
        total,
    })
}

/// Exact byte length of the IPC stream [`encode_ipc_into`] would write, or
/// `None` if `array` is not fast-path eligible (use [`encode_ipc_to_vec`]).
///
/// Lets a caller size the (shared-memory) sample before encoding into it.
pub fn ipc_fast_path_len(array: &ArrayData) -> Option<usize> {
    prepare(array).map(|p| p.total)
}

/// Frame one IPC message into `dst[at..]`: continuation marker, i32-LE metadata
/// length, the flatbuffer, then zero padding so the block is 64-aligned (which
/// makes the following body — or next message — start 64-aligned). Returns the
/// block size written.
fn write_framed_message(dst: &mut [u8], at: usize, flatbuffer: &[u8]) -> usize {
    let block = round_up(PREFIX_LEN + flatbuffer.len(), ALIGN);
    let metadata_len = (block - PREFIX_LEN) as i32;
    dst[at..at + 4].copy_from_slice(&CONTINUATION_MARKER);
    dst[at + 4..at + 8].copy_from_slice(&metadata_len.to_le_bytes());
    dst[at + 8..at + 8 + flatbuffer.len()].copy_from_slice(flatbuffer);
    // Zero the trailing padding (the sample buffer may be uninitialized SHM).
    dst[at + 8 + flatbuffer.len()..at + block].fill(0);
    block
}

/// Encode `array` as a complete Arrow IPC stream directly into `dst`, copying
/// each array buffer exactly once.
///
/// `dst.len()` must equal [`ipc_fast_path_len(array)`](ipc_fast_path_len);
/// `array` must be fast-path eligible (it is when `ipc_fast_path_len` returned
/// `Some`).
pub fn encode_ipc_into(array: &ArrayData, dst: &mut [u8]) -> eyre::Result<()> {
    let prepared =
        prepare(array).ok_or_else(|| eyre!("array is not Arrow IPC fast-path eligible"))?;
    if dst.len() != prepared.total {
        bail!(
            "destination size {} does not match required IPC length {}",
            dst.len(),
            prepared.total
        );
    }

    let mut at = 0;
    at += write_framed_message(dst, at, &prepared.schema_message);
    debug_assert_eq!(at, prepared.schema_block);
    at += write_framed_message(dst, at, &prepared.record_batch_message);
    debug_assert_eq!(at, prepared.schema_block + prepared.record_batch_block);

    let body_start = at;
    for desc in &prepared.layout.descs {
        let start = body_start + desc.offset;
        let end = start + desc.len;
        match &desc.src {
            BufferSrc::Bytes(buffer, src_off) => {
                dst[start..end].copy_from_slice(&buffer.as_slice()[*src_off..*src_off + desc.len]);
            }
            BufferSrc::AllOnes => dst[start..end].fill(0xff),
        }
        // Zero this buffer's trailing alignment padding.
        let padded_end = body_start + desc.offset + round_up(desc.len, ALIGN);
        dst[end..padded_end].fill(0);
    }
    at = body_start + prepared.layout.body_len;

    // End-of-stream: continuation marker + zero length.
    dst[at..at + 4].copy_from_slice(&CONTINUATION_MARKER);
    dst[at + 4..at + 8].copy_from_slice(&0i32.to_le_bytes());
    debug_assert_eq!(at + PREFIX_LEN, prepared.total);

    Ok(())
}

/// Fallback encoder for any array (including non-fast-path types): produce a
/// full Arrow IPC stream `Vec` via the official writer. The caller copies this
/// into the sample, so this path costs two payload copies.
pub fn encode_ipc_to_vec(array: &ArrayData) -> eyre::Result<Vec<u8>> {
    super::encode_arrow_ipc(array).context("Arrow IPC fallback encode")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arrow_utils::decode_arrow_ipc_zero_copy;
    use arrow::array::{
        Array, ArrayRef, BooleanArray, FixedSizeBinaryArray, Float32Array, Int32Array,
        LargeStringArray, ListArray, NullArray, StringArray, StructArray, UInt8Array, UInt64Array,
    };
    use arrow::buffer::Buffer;
    use arrow::ipc::reader::{StreamDecoder, StreamReader};
    use arrow_schema::{DataType, Field};
    use std::io::Cursor;
    use std::sync::Arc;

    /// Encode via the fast path into a fresh `Vec` sized by `ipc_fast_path_len`.
    fn fast_encode(array: &ArrayData) -> Vec<u8> {
        let len = ipc_fast_path_len(array).expect("array should be fast-path eligible");
        let mut buf = vec![0u8; len];
        encode_ipc_into(array, &mut buf).expect("fast-path encode");
        buf
    }

    /// Decode a complete IPC stream with the OFFICIAL arrow `StreamReader` (the
    /// oracle for wire correctness — independent of our decoder).
    fn read_official(bytes: &[u8]) -> ArrayData {
        let mut reader = StreamReader::try_new(Cursor::new(bytes), None).expect("open IPC stream");
        let batch = reader
            .next()
            .expect("one batch")
            .expect("batch decodes via official reader");
        assert_eq!(batch.num_columns(), 1);
        batch.column(0).to_data()
    }

    /// Copy `bytes` into a 128-byte-aligned Arrow `Buffer`, mirroring how the
    /// receive path backs SHM payloads — the precondition for zero-copy decode.
    fn aligned_buffer(bytes: &[u8]) -> (Buffer, usize, usize) {
        use aligned_vec::{AVec, ConstAlign};
        use std::ptr::NonNull;
        let mut aligned: AVec<u8, ConstAlign<128>> = AVec::__from_elem(128, 0, bytes.len());
        aligned.copy_from_slice(bytes);
        let base = aligned.as_ptr() as usize;
        let len = aligned.len();
        let ptr = NonNull::new(aligned.as_ptr() as *mut u8).unwrap();
        // SAFETY: ptr/len describe `aligned`; the Arc keeps it alive.
        let buffer =
            unsafe { Buffer::from_custom_allocation(ptr, len, std::sync::Arc::new(aligned)) };
        (buffer, base, len)
    }

    /// The core assertion: the fast-path stream (1) decodes via the official
    /// reader to an equal array, and (2) `ipc_fast_path_len` matches the bytes
    /// written and the stream the reader fully consumes.
    fn assert_fast_roundtrip(array: &ArrayData) {
        let encoded = fast_encode(array);
        let decoded = read_official(&encoded);
        assert_eq!(array, &decoded, "fast-path stream must decode to the input");
        // Our own zero-copy decoder must agree too.
        let (buffer, _, _) = aligned_buffer(&encoded);
        let zc = decode_arrow_ipc_zero_copy(buffer).expect("zero-copy decode");
        assert_eq!(array, &zc, "zero-copy decode must equal the input");
    }

    #[test]
    fn roundtrip_primitive_no_nulls() {
        let array = Float32Array::from((0..1000).map(|i| i as f32).collect::<Vec<_>>()).into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_primitive_with_nulls() {
        let array = UInt64Array::from(vec![Some(1), None, Some(3), None, Some(5)]).into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_empty_primitive() {
        let array = Int32Array::from(Vec::<i32>::new()).into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_boolean() {
        let array =
            BooleanArray::from(vec![true, false, true, true, false, false, true]).into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_utf8() {
        let array =
            StringArray::from(vec![Some("hello"), None, Some(""), Some("world!")]).into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_large_utf8_64bit_offsets() {
        let array = LargeStringArray::from(vec!["a", "bb", "ccc"]).into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_fixed_size_binary() {
        let values = vec![vec![1u8, 2, 3], vec![4, 5, 6], vec![7, 8, 9]];
        let array = FixedSizeBinaryArray::try_from_iter(values.into_iter())
            .unwrap()
            .into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_struct_with_multilevel_nulls() {
        let array = StructArray::from(vec![
            (
                Arc::new(Field::new("a", DataType::UInt64, true)),
                Arc::new(UInt64Array::from(vec![Some(1), None, Some(3)])) as ArrayRef,
            ),
            (
                Arc::new(Field::new("b", DataType::Utf8, true)),
                Arc::new(StringArray::from(vec![Some("x"), Some("yy"), None])) as ArrayRef,
            ),
        ])
        .into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_list_of_primitive() {
        let data = vec![
            Some(vec![Some(0), Some(1), Some(2)]),
            None,
            Some(vec![Some(3), None, Some(5)]),
            Some(vec![]),
        ];
        let array =
            ListArray::from_iter_primitive::<arrow::datatypes::Int32Type, _, _>(data).into_data();
        assert_fast_roundtrip(&array);
    }

    #[test]
    fn roundtrip_nullarray_zero_and_n() {
        assert_fast_roundtrip(&NullArray::new(0).into_data());
        assert_fast_roundtrip(&NullArray::new(7).into_data());
    }

    /// Headline zero-copy proof: the fast-path body is 64-aligned, so the strict
    /// decoder accepts it without a realigning copy AND the decoded data buffer
    /// aliases the input allocation.
    #[test]
    fn fast_path_decodes_zero_copy() {
        let array = UInt64Array::from((0..50_000u64).collect::<Vec<_>>()).into_data();
        let encoded = fast_encode(&array);

        // (1) strict decoder: errors if any buffer needs realigning.
        {
            let (mut buffer, _, _) = aligned_buffer(&encoded);
            let mut decoder = StreamDecoder::new().with_require_alignment(true);
            let mut got = None;
            while !buffer.is_empty() {
                if let Some(b) = decoder
                    .decode(&mut buffer)
                    .expect("aligned fast-path stream must decode without realignment")
                {
                    got = Some(b);
                    break;
                }
            }
            assert_eq!(got.unwrap().column(0).to_data(), array);
        }

        // (2) pointer aliasing: the decoded data buffer lies inside the input.
        {
            let (buffer, base, len) = aligned_buffer(&encoded);
            let decoded = decode_arrow_ipc_zero_copy(buffer).unwrap();
            let ptr = decoded.buffers()[0].as_ptr() as usize;
            assert!(
                ptr >= base && ptr < base + len,
                "decoded data buffer at {ptr:#x} is outside input [{base:#x}, {:#x}) — a copy happened",
                base + len
            );
        }
    }

    #[test]
    fn fast_path_len_matches_official_decode() {
        // The whole stream is consumed by the official reader (no trailing junk,
        // no truncation): `ipc_fast_path_len` is exact.
        let array = Float32Array::from(vec![1.0, 2.0, 3.0, 4.0]).into_data();
        let encoded = fast_encode(&array);
        let mut reader = StreamReader::try_new(Cursor::new(&encoded[..]), None).unwrap();
        let _ = reader.next().unwrap().unwrap();
        assert!(reader.next().is_none(), "exactly one batch, fully consumed");
    }

    /// Sliced (non-zero offset) arrays are routed to the fallback, which must
    /// still round-trip the *logical* slice (not the parent).
    #[test]
    fn sliced_array_routes_to_fallback_and_roundtrips() {
        let array = UInt64Array::from(vec![10, 20, 30, 40, 50])
            .into_data()
            .slice(2, 2); // offset 2 -> not fast-path
        assert_eq!(array.offset(), 2);
        assert!(ipc_fast_path_len(&array).is_none());

        let encoded = encode_ipc_to_vec(&array).unwrap();
        let decoded = read_official(&encoded);
        assert_eq!(array.len(), decoded.len());
        let dec = arrow::array::make_array(decoded);
        let dec = dec.as_any().downcast_ref::<UInt64Array>().unwrap();
        assert_eq!(dec.values(), &[30, 40]);
    }

    /// A `*View` type must route to the fallback (validate the classifier).
    #[test]
    fn view_type_routes_to_fallback() {
        use arrow::array::StringViewArray;
        let array = StringViewArray::from(vec!["a", "bb", "ccc"]).into_data();
        assert!(
            ipc_fast_path_len(&array).is_none(),
            "Utf8View is not fast-path eligible"
        );
        let encoded = encode_ipc_to_vec(&array).unwrap();
        let decoded = read_official(&encoded);
        assert_eq!(array, decoded);
    }

    /// Deterministic fuzz: many shapes through the bidirectional assertion.
    #[test]
    fn fuzz_roundtrip_many_shapes() {
        // Simple LCG so the matrix is varied but reproducible (no rng/time).
        let mut state: u64 = 0x1234_5678_9abc_def0;
        let mut next = || {
            state = state
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            state
        };

        for _ in 0..200 {
            let len = (next() % 64) as usize;
            let kind = next() % 6;
            let array: ArrayData = match kind {
                0 => UInt8Array::from(
                    (0..len)
                        .map(|i| {
                            if next().is_multiple_of(4) {
                                None
                            } else {
                                Some((i as u8).wrapping_add(1))
                            }
                        })
                        .collect::<Vec<_>>(),
                )
                .into_data(),
                1 => Float32Array::from((0..len).map(|i| i as f32 * 0.5).collect::<Vec<_>>())
                    .into_data(),
                2 => BooleanArray::from(
                    (0..len)
                        .map(|i| (i + next() as usize).is_multiple_of(2))
                        .collect::<Vec<_>>(),
                )
                .into_data(),
                3 => StringArray::from(
                    (0..len)
                        .map(|i| {
                            if next().is_multiple_of(5) {
                                None
                            } else {
                                Some("x".repeat(i % 7))
                            }
                        })
                        .collect::<Vec<_>>(),
                )
                .into_data(),
                4 => Int32Array::from(
                    (0..len)
                        .map(|i| {
                            if next().is_multiple_of(3) {
                                None
                            } else {
                                Some(i as i32 - 10)
                            }
                        })
                        .collect::<Vec<_>>(),
                )
                .into_data(),
                _ => StructArray::from(vec![(
                    Arc::new(Field::new("v", DataType::Int32, true)),
                    Arc::new(Int32Array::from(
                        (0..len).map(|i| Some(i as i32)).collect::<Vec<_>>(),
                    )) as ArrayRef,
                )])
                .into_data(),
            };
            assert_fast_roundtrip(&array);
        }
    }
}
