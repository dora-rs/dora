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
//! Two types need their children sliced before recursion because the child's
//! IPC length is the parent's rather than the child's own: `Struct` (each field
//! to the struct's `len`) and `FixedSizeList` (its child to `len * value_size`).
//! `List`/`LargeList` children are bounded by an offsets buffer and recursed at
//! full length.

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
    /// Append a buffer descriptor at the current (64-aligned) offset and advance
    /// past it (padded to 64). Returns `None` if the running body size would
    /// overflow `usize` or exceed [`super::MAX_IPC_BYTES`] — such arrays route to
    /// the fallback rather than wrapping to a too-small offset.
    fn push_buffer(&mut self, off: &mut usize, len: usize, src: BufferSrc) -> Option<()> {
        // `*off` is always 64-aligned here (the invariant `push_buffer` keeps).
        let padded = len.checked_add(ALIGN - 1).map(|n| n & !(ALIGN - 1))?;
        let next = off.checked_add(padded)?;
        if next > super::MAX_IPC_BYTES {
            return None;
        }
        self.ipc_buffers
            .push(IpcBuffer::new(*off as i64, len as i64));
        self.descs.push(Desc {
            offset: *off,
            len,
            src,
        });
        *off = next;
        Some(())
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
                layout.push_buffer(off, bytes, BufferSrc::Bytes(sliced, 0))?;
            }
            None => {
                let bytes = len.div_ceil(8);
                layout.push_buffer(off, bytes, BufferSrc::AllOnes)?;
            }
        }
    }

    // Data buffers, copied in full (empty for Struct/FixedSizeList).
    for buffer in array.buffers() {
        layout.push_buffer(off, buffer.len(), BufferSrc::Bytes(buffer.clone(), 0))?;
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
        DataType::Struct(_) => {
            // Every struct field's IPC field node must report the struct's row
            // count. A child `ArrayData` may legally be *longer* than the struct
            // (e.g. built via the low-level builder), so slice each child to the
            // struct's `len` before recursing — otherwise the child node would
            // declare a different length and the stream would be un-decodable.
            for child in array.child_data() {
                if child.len() < len {
                    return None;
                }
                build_layout_rec(&child.slice(0, len), layout, off)?;
            }
        }
        _ => {
            // List/LargeList: the single child is the values array, bounded by
            // the offsets buffer (its length is independent of the parent's), so
            // it is recursed at full length.
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
    write_body(dst, body_start, &prepared.layout);
    at = body_start + prepared.layout.body_len;

    // End-of-stream: continuation marker + zero length.
    dst[at..at + 4].copy_from_slice(&CONTINUATION_MARKER);
    dst[at + 4..at + 8].copy_from_slice(&0i32.to_le_bytes());
    debug_assert_eq!(at + PREFIX_LEN, prepared.total);

    Ok(())
}

/// Copy each body buffer to its 64-aligned offset within `dst[body_start..]`,
/// zeroing per-buffer alignment padding (the destination may be uninitialized
/// SHM). Shared by the full-stream and schema-less-batch encoders.
fn write_body(dst: &mut [u8], body_start: usize, layout: &Layout) {
    for desc in &layout.descs {
        let start = body_start + desc.offset;
        let end = start + desc.len;
        match &desc.src {
            BufferSrc::Bytes(buffer, src_off) => {
                dst[start..end].copy_from_slice(&buffer.as_slice()[*src_off..*src_off + desc.len]);
            }
            BufferSrc::AllOnes => dst[start..end].fill(0xff),
        }
        let padded_end = body_start + desc.offset + round_up(desc.len, ALIGN);
        dst[end..padded_end].fill(0);
    }
}

/// Encode the IPC **schema message** for `data_type` as a framed,
/// 64-byte-aligned block (one nullable field named `data`).
///
/// This is the schema prefix of a stream, sent once. A receiver feeds it to a
/// persistent [`StreamDecoder`](arrow::ipc::reader::StreamDecoder) to prime it,
/// then decodes a sequence of schema-less batch messages from
/// [`encode_batch_into`] against the same decoder (the W3 schema-once path).
pub fn encode_schema_message(data_type: &DataType) -> eyre::Result<Vec<u8>> {
    let schema_message = build_schema_message(data_type)?;
    let block = round_up(PREFIX_LEN + schema_message.len(), ALIGN);
    let mut dst = vec![0u8; block];
    write_framed_message(&mut dst, 0, &schema_message);
    Ok(dst)
}

/// Exact byte length of the schema-less batch message [`encode_batch_into`]
/// would write (record-batch header block + body + 8-byte end-of-stream marker,
/// **no** schema prefix), or `None` if `array` is not fast-path eligible.
pub fn batch_fast_path_len(array: &ArrayData) -> Option<usize> {
    let prepared = prepare(array)?;
    Some(prepared.record_batch_block + prepared.layout.body_len + PREFIX_LEN)
}

/// Encode `array` as a schema-less Arrow IPC **batch message** into `dst`:
/// the record-batch header block, the body, and a trailing 8-byte end-of-stream
/// marker (no schema prefix). Decoded by a [`StreamDecoder`] already primed with
/// the matching [`encode_schema_message`]. The trailing marker lets the decoder
/// flush a 0-row batch (empty body); a non-empty batch never reads it. `dst.len()`
/// must equal [`batch_fast_path_len(array)`](batch_fast_path_len).
pub fn encode_batch_into(array: &ArrayData, dst: &mut [u8]) -> eyre::Result<()> {
    let prepared =
        prepare(array).ok_or_else(|| eyre!("array is not Arrow IPC fast-path eligible"))?;
    let expected = prepared.record_batch_block + prepared.layout.body_len + PREFIX_LEN;
    if dst.len() != expected {
        bail!(
            "destination size {} does not match required batch length {expected}",
            dst.len(),
        );
    }
    let at = write_framed_message(dst, 0, &prepared.record_batch_message);
    debug_assert_eq!(at, prepared.record_batch_block);
    write_body(dst, at, &prepared.layout);
    // End-of-stream marker (continuation + zero length), matching the tail of a
    // full stream so an empty batch flushes through the persistent decoder.
    let body_end = at + prepared.layout.body_len;
    dst[body_end..body_end + 4].copy_from_slice(&CONTINUATION_MARKER);
    dst[body_end + 4..body_end + PREFIX_LEN].copy_from_slice(&0i32.to_le_bytes());
    Ok(())
}

/// Fallback encoder for any array (including non-fast-path types): produce a
/// full Arrow IPC stream `Vec` via the official writer. The caller copies this
/// into the sample, so this path costs two payload copies.
pub fn encode_ipc_to_vec(array: &ArrayData) -> eyre::Result<Vec<u8>> {
    super::encode_arrow_ipc(array).context("Arrow IPC fallback encode")
}

/// IPC layout of a `UInt8` array of `data_len` elements (no nulls): the byte
/// offsets and message blocks, computed directly without materializing the
/// data buffer (so it is cheap for a large image/tensor).
struct Uint8Layout {
    schema_message: Vec<u8>,
    record_batch_message: Vec<u8>,
    validity_len: usize,
    validity_padded: usize,
    body_len: usize,
    total: usize,
    data_offset: usize,
}

fn uint8_layout(data_len: usize) -> eyre::Result<Uint8Layout> {
    if data_len > super::MAX_IPC_BYTES {
        bail!(
            "UInt8 payload too large: {data_len} bytes (max {})",
            super::MAX_IPC_BYTES
        );
    }
    let validity_len = data_len.div_ceil(8);
    let validity_padded = round_up(validity_len, ALIGN);
    let body_len = validity_padded + round_up(data_len, ALIGN);
    // Matches what `encode_ipc_into` emits for a no-null UInt8Array: one field
    // node, then [validity all-ones | data].
    let nodes = [FieldNode::new(data_len as i64, 0)];
    let buffers = [
        IpcBuffer::new(0, validity_len as i64),
        IpcBuffer::new(validity_padded as i64, data_len as i64),
    ];
    let record_batch_message = build_record_batch_message(data_len, &nodes, &buffers, body_len);
    let schema_message = build_schema_message(&DataType::UInt8)?;
    let schema_block = round_up(PREFIX_LEN + schema_message.len(), ALIGN);
    let record_batch_block = round_up(PREFIX_LEN + record_batch_message.len(), ALIGN);
    let total = schema_block + record_batch_block + body_len + PREFIX_LEN;
    let data_offset = schema_block + record_batch_block + validity_padded;
    Ok(Uint8Layout {
        schema_message,
        record_batch_message,
        validity_len,
        validity_padded,
        body_len,
        total,
        data_offset,
    })
}

/// Total IPC stream length for a no-null `UInt8` array of `data_len` elements.
/// Lets a caller size the sample before constructing the message in place via
/// [`encode_uint8_ipc_header`].
pub fn uint8_ipc_len(data_len: usize) -> eyre::Result<usize> {
    Ok(uint8_layout(data_len)?.total)
}

/// Write a complete no-null `UInt8` IPC stream into `dst` **except the data
/// region**, and return the byte offset at which the caller must write the
/// `data_len` data bytes (the buffer-protocol "construct in place" path).
///
/// `dst.len()` must equal [`uint8_ipc_len(data_len)`](uint8_ipc_len). After the
/// caller fills `dst[offset..offset + data_len]`, `dst` is a valid IPC stream
/// that decodes to the user's bytes as a `UInt8Array` — with zero payload
/// copies.
pub fn encode_uint8_ipc_header(dst: &mut [u8], data_len: usize) -> eyre::Result<usize> {
    let layout = uint8_layout(data_len)?;
    if dst.len() != layout.total {
        bail!(
            "destination size {} does not match required UInt8 IPC length {}",
            dst.len(),
            layout.total
        );
    }
    let mut at = 0;
    at += write_framed_message(dst, at, &layout.schema_message);
    at += write_framed_message(dst, at, &layout.record_batch_message);
    let body_start = at;

    // Validity: all-ones bitmap, then padding to the data buffer.
    dst[body_start..body_start + layout.validity_len].fill(0xff);
    dst[body_start + layout.validity_len..body_start + layout.validity_padded].fill(0);

    // The data region [data_offset .. data_offset + data_len] is left for the
    // caller. Zero only its trailing alignment padding.
    let data_end = layout.data_offset + data_len;
    let body_end = body_start + layout.body_len;
    dst[data_end..body_end].fill(0);

    // End-of-stream marker.
    dst[body_end..body_end + 4].copy_from_slice(&CONTINUATION_MARKER);
    dst[body_end + 4..body_end + 8].copy_from_slice(&0i32.to_le_bytes());
    debug_assert_eq!(body_end + PREFIX_LEN, layout.total);

    Ok(layout.data_offset)
}

/// Length of the leading schema-message block of a full IPC stream produced by
/// [`encode_ipc_into`] (so a receiver can split it into the schema prefix and
/// the record-batch+body), or `None` if `stream` is not a framed IPC message.
pub fn schema_block_len(stream: &[u8]) -> Option<usize> {
    if stream.len() < PREFIX_LEN || stream[0..4] != CONTINUATION_MARKER {
        return None;
    }
    let metadata_len = i32::from_le_bytes(stream[4..8].try_into().ok()?);
    let block = PREFIX_LEN.checked_add(usize::try_from(metadata_len).ok()?)?;
    (block <= stream.len()).then_some(block)
}

/// Schema identity of a full IPC stream: the FNV-1a hash of its leading schema
/// block, plus the block itself. This pairing IS the schema-once wire contract
/// — the producer (`publish_schema_once`), the node receive path (in-band
/// priming), and the daemon's `dora topic` rebuild all derive the hash from
/// exactly these bytes via this function; independent re-derivations could
/// drift and silently break batch↔schema matching.
pub fn schema_block_and_hash(stream: &[u8]) -> Option<(u64, &[u8])> {
    let block = schema_block_len(stream)?;
    let schema = stream.get(..block)?;
    Some((dora_message::metadata::fnv1a(schema), schema))
}

/// Given a full IPC stream, return the schema-less record-batch slice —
/// everything after the schema block, **including** the trailing 8-byte
/// end-of-stream marker. The marker is what lets the receiver's persistent
/// decoder flush a 0-row batch (whose body is empty); a non-empty batch never
/// reads it. `None` if `stream` is malformed. See [`decode_one_batch`].
pub fn batch_slice(stream: &[u8]) -> Option<&[u8]> {
    let block = schema_block_len(stream)?;
    // The stream is [schema block][record-batch block][body][EOS(8)]; keep
    // everything from the record-batch block onward.
    (stream.len() >= block + PREFIX_LEN).then(|| &stream[block..])
}

/// Maximum number of distinct schemas an [`InputDecoder`] retains for local
/// re-priming. Bounds the memory a misbehaving producer (rotating through many
/// schemas on one output) can pin in a receiver; beyond it the oldest schema is
/// evicted and its batches drop until it is re-installed.
const MAX_RETAINED_SCHEMAS: usize = 8;

/// Per-input receive state for the schema-once zenoh path: one persistent
/// [`StreamDecoder`](arrow::ipc::reader::StreamDecoder) primed from the schema
/// published on the output's `@schema` subtopic (or in-band, from the schema
/// block of a full self-describing stream on the data topic), then reused to
/// decode the schema-less batch messages that flow on the data topic.
pub struct InputDecoder {
    /// Live decoder, primed with the schema for [`schema_hash`](Self::schema_hash).
    decoder: arrow::ipc::reader::StreamDecoder,
    /// Hash of the schema the live `decoder` is currently primed with.
    schema_hash: Option<u64>,
    /// The framed schema messages installed via [`set_schema`](Self::set_schema),
    /// keyed by hash (most-recent last, bounded by [`MAX_RETAINED_SCHEMAS`]).
    /// Retained so the decoder can be re-primed locally (no network round-trip)
    /// after a failed batch decode soft-resets the live decoder, or when batches
    /// reference a schema seen earlier (e.g. after in-band priming from a
    /// different schema's full stream re-primed the live decoder in between).
    schemas: Vec<(u64, ArrowBuffer)>,
}

impl Default for InputDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl InputDecoder {
    /// Create an unprimed decoder. It decodes nothing until a schema is
    /// installed via [`set_schema`](Self::set_schema) (delivered from the output's
    /// `@schema` subtopic).
    pub fn new() -> Self {
        Self {
            decoder: arrow::ipc::reader::StreamDecoder::new(),
            schema_hash: None,
            schemas: Vec::new(),
        }
    }

    /// Forget all state — the primed decoder AND the retained schemas. The next
    /// schema message must re-establish priming. Used on producer restart and
    /// for lock-poison recovery.
    pub fn reset(&mut self) {
        self.decoder = arrow::ipc::reader::StreamDecoder::new();
        self.schema_hash = None;
        self.schemas.clear();
    }

    /// Whether a schema with this hash is already available — live or retained.
    /// The in-band priming path skips the schema-block copy and the eager
    /// re-prime for known schemas; [`decode_batch`](Self::decode_batch)
    /// re-primes lazily from the retained set when a batch actually needs one,
    /// so eagerly re-priming a retained schema would only churn the live
    /// decoder (e.g. a large full-stream message of schema B clobbering the
    /// live prime of schema A between A's schema-less batches).
    pub fn knows_schema(&self, hash: u64) -> bool {
        self.schema_hash == Some(hash) || self.schemas.iter().any(|(h, _)| *h == hash)
    }

    /// Install the schema for `hash` from a framed IPC **schema message** (the
    /// payload published on the `@schema` subtopic, or the schema block of a
    /// full stream received in-band): prime a fresh decoder with it and retain
    /// the bytes for later local re-priming. A no-op when the live decoder is
    /// already primed with `hash`.
    pub fn set_schema(&mut self, hash: u64, schema: ArrowBuffer) -> eyre::Result<()> {
        check_ipc_size(schema.len())?;
        if self.schema_hash == Some(hash) {
            return Ok(());
        }
        self.prime(hash, schema)
    }

    /// Prime a fresh `StreamDecoder` with `schema` and record it as the current
    /// and retained schema for `hash`.
    fn prime(&mut self, hash: u64, schema: ArrowBuffer) -> eyre::Result<()> {
        // Create a fresh decoder before priming. In Arrow 59+, even consuming a
        // schema message can leave the decoder close to terminal state when the
        // EOS marker is encountered. A fresh decoder ensures we start clean.
        let mut decoder = arrow::ipc::reader::StreamDecoder::new();
        prime_with_schema(&mut decoder, schema.clone())?;
        self.decoder = decoder;
        self.schema_hash = Some(hash);
        self.schemas.retain(|(h, _)| *h != hash);
        self.schemas.push((hash, schema));
        if self.schemas.len() > MAX_RETAINED_SCHEMAS {
            self.schemas.remove(0);
        }
        Ok(())
    }

    /// Decode a schema-less batch message against the decoder primed for `hash`.
    ///
    /// If the live decoder is not primed for `hash` but a matching schema was
    /// previously installed (e.g. after a soft reset), it re-primes from the
    /// retained bytes first. Returns `Ok(None)` when no schema for `hash` is
    /// known yet — the caller drops the message, which is fine on the lossy
    /// `CongestionControl::Drop` data plane (the `@schema` history query, the
    /// next schema publish, or the producer's periodic full-stream refresh —
    /// which primes in-band — will prime it).
    pub fn decode_batch(
        &mut self,
        buffer: ArrowBuffer,
        hash: u64,
    ) -> eyre::Result<Option<arrow::array::ArrayData>> {
        check_ipc_size(buffer.len())?;
        if self.schema_hash != Some(hash) {
            match self.schemas.iter().find(|(h, _)| *h == hash) {
                Some((_, schema)) => {
                    let schema = schema.clone();
                    self.prime(hash, schema)?;
                }
                // No schema for this hash known yet — drop (lossy plane).
                None => return Ok(None),
            }
        }
        match decode_one_batch(&mut self.decoder, buffer) {
            Ok(array) => {
                // Arrow 59+ reaches a terminal state after decoding each message's
                // EOS marker, rejecting further input. Reset the decoder so the next
                // batch (another independent IPC stream) can be decoded against the
                // same schema. Clearing schema_hash forces the next batch to re-prime
                // from the retained schemas (instant lookup, no network round-trip).
                self.decoder = arrow::ipc::reader::StreamDecoder::new();
                self.schema_hash = None;
                Ok(Some(array))
            }
            Err(e) => {
                // A failed batch decode (truncated/corrupt payload — zenoh
                // tail-loss or a malicious peer) leaves the persistent decoder
                // mid-message. Soft-reset the LIVE decoder so the next batch is
                // not fed into that poisoned state (which would misinterpret it
                // against this message's stale buffers and deliver corrupt data).
                // The retained schema is kept, so the next batch re-primes
                // locally — instant recovery, with no wait for a schema refresh.
                self.decoder = arrow::ipc::reader::StreamDecoder::new();
                self.schema_hash = None;
                Err(e)
            }
        }
    }
}

/// Reject an IPC payload larger than [`super::MAX_IPC_BYTES`] before decoding.
/// Defense-in-depth against an oversized peer-controlled zenoh payload, mirroring
/// the guard in [`decode_arrow_ipc_zero_copy`](super::decode_arrow_ipc_zero_copy)
/// (the persistent-decoder paths receive the same untrusted bytes).
fn check_ipc_size(len: usize) -> eyre::Result<()> {
    if len > super::MAX_IPC_BYTES {
        bail!(
            "Arrow IPC payload too large: {len} bytes (max {})",
            super::MAX_IPC_BYTES
        );
    }
    Ok(())
}

/// Feed a schema message to `decoder`; it must yield no batch.
fn prime_with_schema(
    decoder: &mut arrow::ipc::reader::StreamDecoder,
    mut buffer: ArrowBuffer,
) -> eyre::Result<()> {
    while !buffer.is_empty() {
        let before = buffer.len();
        if decoder
            .decode(&mut buffer)
            .map_err(|e| eyre!("failed to decode IPC schema message: {e}"))?
            .is_some()
        {
            bail!("expected a schema message but got a record batch");
        }
        // Guard against a crafted/truncated payload that decodes to no batch
        // without consuming bytes — otherwise this loop spins forever.
        if buffer.len() == before {
            bail!("IPC schema decoder made no progress on a partial/corrupt message");
        }
    }
    Ok(())
}

/// Feed a record-batch message to `decoder` and return the single decoded array.
///
/// The schema-less batch is terminated by an 8-byte end-of-stream marker (see
/// [`encode_batch_into`]/[`batch_slice`]). For a non-empty batch the body bytes
/// flush it on their own; the marker matters for a **0-row** batch, whose
/// zero-length body arrow's `StreamDecoder` only emits when polled with more
/// input — without the trailing marker an empty array is silently dropped
/// (PR #2366). The decoder yields the pending batch *before* consuming the
/// marker's zero length, so it never reaches its terminal state and stays
/// usable for the next batch.
fn decode_one_batch(
    decoder: &mut arrow::ipc::reader::StreamDecoder,
    mut buffer: ArrowBuffer,
) -> eyre::Result<arrow::array::ArrayData> {
    while !buffer.is_empty() {
        let before = buffer.len();
        if let Some(batch) = decoder
            .decode(&mut buffer)
            .map_err(|e| eyre!("failed to decode IPC record batch: {e}"))?
        {
            if batch.num_columns() != 1 {
                bail!(
                    "expected 1 column in IPC record batch, got {}",
                    batch.num_columns()
                );
            }
            return Ok(batch.column(0).to_data());
        }
        // Guard against a crafted/truncated payload that decodes to no batch
        // without consuming bytes — otherwise this loop spins forever.
        if buffer.len() == before {
            bail!("IPC batch decoder made no progress on a partial/corrupt message");
        }
    }
    bail!("IPC batch message yielded no record batch")
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

    /// Encode `array` as a schema-less batch message (the fast path's
    /// `batch_slice` equivalent), as shipped on the schema-once data plane.
    fn batch_bytes(array: &ArrayData) -> Vec<u8> {
        let len = batch_fast_path_len(array).unwrap();
        let mut buf = vec![0u8; len];
        encode_batch_into(array, &mut buf).unwrap();
        buf
    }

    fn batch_buf(array: &ArrayData) -> Buffer {
        Buffer::from_vec(batch_bytes(array))
    }

    /// The schema-once receive contract: an `InputDecoder` is primed by a schema
    /// message (as delivered from the `@schema` subtopic), then decodes the
    /// schema-less batches that follow, and drops a batch whose hash it isn't
    /// primed for.
    #[test]
    fn input_decoder_schema_then_batches() {
        let f32_schema = || Buffer::from_vec(encode_schema_message(&DataType::Float32).unwrap());

        let mut dec = InputDecoder::new();

        // A batch arriving before any schema is dropped (not primed).
        let early = Float32Array::from(vec![9.0]).into_data();
        assert!(dec.decode_batch(batch_buf(&early), 7).unwrap().is_none());

        // Installing the schema primes the decoder; following batches decode.
        dec.set_schema(7, f32_schema()).unwrap();
        for vals in [vec![1.0f32, 2.0, 3.0], vec![4.0], vec![5.0, 6.0, 7.0]] {
            let array = Float32Array::from(vals).into_data();
            assert_eq!(
                dec.decode_batch(batch_buf(&array), 7).unwrap().unwrap(),
                array
            );
        }

        // A batch tagged with a hash the decoder isn't primed for is dropped.
        let other = Float32Array::from(vec![8.0]).into_data();
        assert!(dec.decode_batch(batch_buf(&other), 99).unwrap().is_none());

        // Installing a schema under the new hash primes it; its batches decode.
        dec.set_schema(99, f32_schema()).unwrap();
        let after = Float32Array::from(vec![10.0, 11.0]).into_data();
        assert_eq!(
            dec.decode_batch(batch_buf(&after), 99).unwrap().unwrap(),
            after
        );
    }

    /// Priming a new schema must not forget previously seen ones: batches for a
    /// schema installed earlier still decode after the live decoder was re-primed
    /// with a different schema in between (re-primed locally from the retained
    /// set). Without this, in-band priming from a full stream (e.g. a large
    /// message or a schema-change message on the same output) would clobber the
    /// schema that later schema-less batches reference, silently dropping them.
    #[test]
    fn input_decoder_retains_multiple_schemas() {
        let schema_msg = |dt: &DataType| Buffer::from_vec(encode_schema_message(dt).unwrap());

        let mut dec = InputDecoder::new();
        dec.set_schema(1, schema_msg(&DataType::Float32)).unwrap();
        dec.set_schema(2, schema_msg(&DataType::Int32)).unwrap();

        // Live decoder is primed for hash 2 …
        let ints = Int32Array::from(vec![1, 2, 3]).into_data();
        assert_eq!(
            dec.decode_batch(batch_buf(&ints), 2).unwrap().unwrap(),
            ints
        );

        // … but a batch for hash 1 must still decode (retained schema).
        let floats = Float32Array::from(vec![4.0, 5.0]).into_data();
        assert_eq!(
            dec.decode_batch(batch_buf(&floats), 1).unwrap().unwrap(),
            floats,
            "a schema installed earlier must be retained across later primes"
        );

        // And switching back again also works.
        let more_ints = Int32Array::from(vec![6]).into_data();
        assert_eq!(
            dec.decode_batch(batch_buf(&more_ints), 2).unwrap().unwrap(),
            more_ints
        );
    }

    /// The retained-schema set is bounded: schemas beyond the cap evict the
    /// oldest, whose batches then drop until it is re-installed.
    /// Test that schema-less batches can be decoded repeatedly after the
    /// decoder is soft-reset between batches (Arrow 59+ terminal state fix).
    /// This is the regression test for PR #2445/#2366 interaction with Arrow 59.
    #[test]
    fn input_decoder_handles_sequential_batches_arrow_59_terminal_state() {
        let f32_schema = || Buffer::from_vec(encode_schema_message(&DataType::Float32).unwrap());

        let mut dec = InputDecoder::new();
        // Prime with a schema.
        dec.set_schema(7, f32_schema()).unwrap();

        // Decode 5 schema-less batches sequentially. Each one should decode
        // correctly despite the decoder being reset between batches (soft-reset
        // for Arrow 59 terminal state handling).
        let batches = [
            Float32Array::from(vec![1.0, 2.0]).into_data(),
            Float32Array::from(vec![3.0]).into_data(),
            Float32Array::from(vec![4.0, 5.0, 6.0]).into_data(),
            Float32Array::from(vec![7.0, 8.0]).into_data(),
            Float32Array::from(vec![9.0, 10.0, 11.0, 12.0]).into_data(),
        ];

        for (i, batch) in batches.iter().enumerate() {
            let result = dec.decode_batch(batch_buf(batch), 7);
            assert!(
                result.is_ok(),
                "batch {} decode failed: {:?}",
                i,
                result.err()
            );
            let decoded = result.unwrap().unwrap();
            assert_eq!(
                &decoded, batch,
                "batch {} mismatch: expected {:?}, got {:?}",
                i, batch, decoded
            );
        }
    }

    #[test]
    fn input_decoder_evicts_oldest_schema_beyond_cap() {
        let f32_schema = || Buffer::from_vec(encode_schema_message(&DataType::Float32).unwrap());

        let mut dec = InputDecoder::new();
        // Install cap + 1 distinct hashes (same schema bytes — only the hash
        // keys retention); hash 0 must be evicted, the rest retained.
        for hash in 0..=(MAX_RETAINED_SCHEMAS as u64) {
            dec.set_schema(hash, f32_schema()).unwrap();
        }
        let array = Float32Array::from(vec![1.0]).into_data();
        assert!(
            dec.decode_batch(batch_buf(&array), 0).unwrap().is_none(),
            "the oldest schema must be evicted beyond the cap"
        );
        assert_eq!(
            dec.decode_batch(batch_buf(&array), 1).unwrap().unwrap(),
            array,
            "schemas within the cap must be retained"
        );
    }

    /// A failed (truncated) batch decode must soft-reset the persistent decoder
    /// so the next valid batch is not misinterpreted against the truncated message's
    /// stale state — and, because the schema is retained, that next batch
    /// re-primes locally and decodes (instant recovery, not drop-until-refresh).
    #[test]
    fn decode_batch_resets_on_error_then_reprimes() {
        let mut dec = InputDecoder::new();
        dec.set_schema(
            7,
            Buffer::from_vec(encode_schema_message(&DataType::Float32).unwrap()),
        )
        .unwrap();

        // A valid batch decodes.
        let good = Float32Array::from(vec![4.0, 5.0]).into_data();
        assert_eq!(
            dec.decode_batch(batch_buf(&good), 7).unwrap().unwrap(),
            good
        );

        // A truncated batch errors and soft-resets the live decoder. Cut the
        // buffer in half so the record-batch header survives but its declared
        // body is incomplete (a tail loss) — dropping only the trailing 8-byte
        // EOS marker would leave a still-valid batch.
        let dropped = Float32Array::from(vec![6.0, 7.0, 8.0, 9.0]).into_data();
        let mut truncated = batch_bytes(&dropped);
        truncated.truncate(truncated.len() / 2);
        assert!(dec.decode_batch(Buffer::from_vec(truncated), 7).is_err());

        // The next valid batch (same hash) re-primes from the retained schema and
        // decodes correctly — not dropped, not corrupted.
        let after = Float32Array::from(vec![10.0, 11.0]).into_data();
        assert_eq!(
            dec.decode_batch(batch_buf(&after), 7).unwrap().unwrap(),
            after,
            "after a failed batch the decoder must re-prime from the retained schema"
        );

        // `reset()` forgets the schema too; batches then drop until re-installed.
        dec.reset();
        let final_batch = Float32Array::from(vec![12.0]).into_data();
        assert!(
            dec.decode_batch(batch_buf(&final_batch), 7)
                .unwrap()
                .is_none(),
            "after a full reset the decoder must drop until a schema is re-installed"
        );
    }

    /// Schema-once batch path for a *fallback* (dictionary) type. Dictionary
    /// arrays are not fast-path eligible, so the full stream comes from the
    /// official writer as `[schema][dictionary batch][record batch][EOS]`. The
    /// schema-once optimization still applies to small messages (node/mod.rs),
    /// shipping `batch_slice` = `[dictionary batch][record batch]` against a
    /// decoder primed from the schema subtopic — i.e. a *replacement* dictionary
    /// on every message. The `InputDecoder` tests otherwise use only `Float32`; a
    /// failure here is silent intermittent input loss, so it needs an explicit
    /// oracle.
    #[test]
    fn input_decoder_dictionary_fallback_batch_sequence() {
        use arrow::array::DictionaryArray;
        use arrow::datatypes::Int32Type;

        // Build a Dictionary<Int32, Utf8> from words, distinct values in
        // first-seen order (deterministic, no iterator-trait ambiguity).
        fn dict(words: &[&str]) -> ArrayData {
            let mut values: Vec<&str> = Vec::new();
            let mut keys: Vec<i32> = Vec::new();
            for w in words {
                let idx = values.iter().position(|v| v == w).unwrap_or_else(|| {
                    values.push(*w);
                    values.len() - 1
                });
                keys.push(idx as i32);
            }
            DictionaryArray::<Int32Type>::try_new(
                Int32Array::from(keys),
                Arc::new(StringArray::from(values)),
            )
            .unwrap()
            .into_data()
        }

        let first = dict(&["a", "b", "a", "c", "b"]);
        // Confirm this really exercises the fallback (not the fast path).
        assert!(
            ipc_fast_path_len(&first).is_none(),
            "dictionary must route to the official-writer fallback"
        );

        // Prime from the schema block of the dictionary stream — exactly the
        // bytes the producer publishes on the `@schema` subtopic.
        let mut dec = InputDecoder::new();
        let full0 = encode_ipc_to_vec(&first).unwrap();
        let block = schema_block_len(&full0).unwrap();
        dec.set_schema(1, Buffer::from(&full0[..block])).unwrap();

        // Every message (including the first) ships only the schema-less batch
        // slice (which for a dictionary type carries a replacement dictionary
        // batch + record batch) against the primed decoder. Each must decode to
        // its own input.
        let slice0 = batch_slice(&full0).expect("fallback stream is a valid IPC stream");
        assert_eq!(
            dec.decode_batch(Buffer::from(slice0), 1)
                .unwrap()
                .expect("first batch decodes against the primed decoder"),
            first
        );
        for words in [
            ["x", "y", "x", "z"].as_slice(),
            ["b", "b"].as_slice(),
            ["new", "values", "entirely"].as_slice(),
        ] {
            let arr = dict(words);
            let full = encode_ipc_to_vec(&arr).unwrap();
            let slice = batch_slice(&full).expect("fallback stream is a valid IPC stream");
            let got = dec
                .decode_batch(Buffer::from(slice), 1)
                .unwrap()
                .expect("batch must decode against the primed decoder");
            assert_eq!(
                got, arr,
                "replacement-dictionary batch must decode correctly"
            );
        }
    }

    /// The W3 schema-once contract: prime one persistent `StreamDecoder` with a
    /// single schema message, then decode a *sequence* of schema-less batch
    /// messages against it. (A fresh full stream per message can't do this — its
    /// EOS terminates the decoder.)
    #[test]
    fn schema_primed_decoder_decodes_batch_sequence() {
        let schema = encode_schema_message(&DataType::Float32).unwrap();
        let mut decoder = StreamDecoder::new();

        // Prime: feeding the schema message yields no batch.
        let mut sbuf = Buffer::from_vec(schema);
        while !sbuf.is_empty() {
            assert!(
                decoder.decode(&mut sbuf).unwrap().is_none(),
                "schema message must not yield a batch"
            );
        }

        for vals in [vec![1.0f32, 2.0, 3.0], vec![4.0, 5.0], vec![6.0]] {
            let array = Float32Array::from(vals).into_data();
            let len = batch_fast_path_len(&array).unwrap();
            let mut buf = vec![0u8; len];
            encode_batch_into(&array, &mut buf).unwrap();

            let mut bbuf = Buffer::from_vec(buf);
            let mut got = None;
            while !bbuf.is_empty() {
                if let Some(b) = decoder.decode(&mut bbuf).unwrap() {
                    got = Some(b);
                    break;
                }
            }
            assert_eq!(
                got.expect("batch message must decode against the primed decoder")
                    .column(0)
                    .to_data(),
                array
            );
        }
    }

    /// `encode_uint8_ipc_header` + caller-filled data region produces a valid
    /// IPC stream identical to `encode_ipc_into` of the equivalent UInt8Array —
    /// the zero-copy "construct in place" path for `send_output_raw`/Python.
    #[test]
    fn uint8_ipc_header_constructs_in_place() {
        for data_len in [0usize, 1, 7, 8, 9, 1000] {
            let bytes: Vec<u8> = (0..data_len).map(|i| (i % 251) as u8).collect();
            let total = uint8_ipc_len(data_len).unwrap();
            let mut dst = vec![0u8; total];
            let offset = encode_uint8_ipc_header(&mut dst, data_len).unwrap();
            // The caller writes the data in place (no copy in real use).
            dst[offset..offset + data_len].copy_from_slice(&bytes);

            // Decodes to the original bytes as a no-null UInt8Array.
            let arr = arrow::array::make_array(read_official(&dst));
            let u8 = arr.as_any().downcast_ref::<UInt8Array>().unwrap();
            assert_eq!(u8.values(), bytes.as_slice(), "len {data_len}");
            assert_eq!(u8.null_count(), 0);

            // Byte-identical to encoding the equivalent array directly.
            let array = UInt8Array::from(bytes).into_data();
            assert_eq!(dst, fast_encode(&array), "len {data_len}");
        }
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

    /// A 0-row array sent through the `send_output_raw` UInt8-header path decodes
    /// back to a 0-row `UInt8` array via the full-stream zero-copy decoder.
    /// (The full stream keeps its end-of-stream marker, so this path was never
    /// broken — this pins it as a baseline alongside the schema-once regression
    /// below.)
    #[test]
    fn uint8_header_zero_len_full_stream_roundtrip() {
        let len = uint8_ipc_len(0).unwrap();
        let mut buf = vec![0u8; len];
        let off = encode_uint8_ipc_header(&mut buf, 0).unwrap();
        // Empty data region: it sits immediately before the 8-byte EOS marker.
        assert_eq!(off, len - PREFIX_LEN);
        assert_eq!(read_official(&buf).len(), 0);
        let (buffer, _, _) = aligned_buffer(&buf);
        let decoded = decode_arrow_ipc_zero_copy(buffer).unwrap();
        assert_eq!(decoded.data_type(), &DataType::UInt8);
        assert_eq!(decoded.len(), 0);
    }

    /// The schema-once receive path must decode a 0-row (empty) batch, not drop
    /// it. A schema-less batch carries no trailing end-of-stream marker, and
    /// arrow's `StreamDecoder` only emits a zero-length body on the poll *after*
    /// the header — so `decode_one_batch` must poll once more once its input
    /// drains. Regression guard for the empty-array drop reported on PR #2366.
    #[test]
    fn schema_once_zero_len_batch_roundtrip() {
        let schema = || Buffer::from_vec(encode_schema_message(&DataType::UInt8).unwrap());
        let batch = |vals: &[u8]| {
            let a = UInt8Array::from(vals.to_vec()).into_data();
            let len = batch_fast_path_len(&a).unwrap();
            let mut b = vec![0u8; len];
            encode_batch_into(&a, &mut b).unwrap();
            Buffer::from_vec(b)
        };

        let mut dec = InputDecoder::new();
        dec.set_schema(1, schema()).unwrap();

        // The empty batch decodes to a 0-row UInt8 array, not `Ok(None)`/error.
        let decoded = dec
            .decode_batch(batch(&[]), 1)
            .unwrap()
            .expect("0-row batch must decode, not drop");
        assert_eq!(decoded.data_type(), &DataType::UInt8);
        assert_eq!(decoded.len(), 0);

        // The persistent decoder stays usable across mixed empty/non-empty
        // batches (flushing the empty body must not wedge or terminate it).
        for vals in [vec![1u8, 2, 3], vec![], vec![9u8], vec![]] {
            let d = dec.decode_batch(batch(&vals), 1).unwrap().unwrap();
            assert_eq!(d.len(), vals.len());
            assert_eq!(d.data_type(), &DataType::UInt8);
        }
    }

    /// Same regression, but exercising the exact production producer path: the
    /// full stream is built by the UInt8 header encoder, the schema is extracted
    /// via [`schema_block_len`] and the schema-less batch via [`batch_slice`]
    /// (as `zenoh_publish` does), then decoded by the per-input [`InputDecoder`].
    #[test]
    fn schema_once_zero_len_via_batch_slice_roundtrip() {
        let total = uint8_ipc_len(0).unwrap();
        let mut full = vec![0u8; total];
        encode_uint8_ipc_header(&mut full, 0).unwrap();

        let sblock = schema_block_len(&full).unwrap();
        let schema = Buffer::from(&full[..sblock]);
        let batch = batch_slice(&full).expect("batch slice of a valid stream");

        let mut dec = InputDecoder::new();
        dec.set_schema(7, schema).unwrap();
        let decoded = dec
            .decode_batch(Buffer::from(batch), 7)
            .unwrap()
            .expect("0-row batch via batch_slice must decode, not drop");
        assert_eq!(decoded.data_type(), &DataType::UInt8);
        assert_eq!(decoded.len(), 0);
    }

    /// The daemon's `dora topic` debug path rebuilds a schema-once batch into a
    /// full self-describing stream by concatenating the cached `@schema` block
    /// with the schema-less batch. That only works because
    /// `schema_block ++ batch_slice` is byte-identical to the original stream —
    /// lock that invariant (and that the result still decodes) here.
    #[test]
    fn schema_block_plus_batch_slice_reconstructs_full_stream() {
        let array = Int32Array::from(vec![1, 2, 3]).into_data();
        let full = fast_encode(&array);
        let sblock = schema_block_len(&full).unwrap();
        let schema = &full[..sblock];
        let batch = batch_slice(&full).expect("batch slice of a valid stream");
        let rebuilt = [schema, batch].concat();
        assert_eq!(
            rebuilt, full,
            "schema_block ++ batch_slice must equal the original stream"
        );
        assert_eq!(read_official(&rebuilt), array);
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

    /// `FixedSizeList` exercises the unique fast-path child-slicing branch
    /// (`n = len * value_size`, then `child.slice(0, n)`). No other test covers
    /// it — `roundtrip_fixed_size_binary` is `FixedSizeBinary`, a leaf type.
    #[test]
    fn roundtrip_fixed_size_list() {
        use arrow::array::FixedSizeListArray;
        let values = Int32Array::from((0..12).collect::<Vec<_>>());
        let field = Arc::new(Field::new("item", DataType::Int32, true));
        let array = FixedSizeListArray::try_new(field, 3, Arc::new(values), None)
            .unwrap()
            .into_data();
        assert_fast_roundtrip(&array);
    }

    /// `FixedSizeList` with a list-level validity bitmap (null lists), so the
    /// child-slicing branch runs alongside the parent's null buffer.
    #[test]
    fn roundtrip_fixed_size_list_with_nulls() {
        use arrow::array::FixedSizeListArray;
        use arrow::buffer::NullBuffer;
        let values = Int32Array::from((0..12).collect::<Vec<_>>());
        let field = Arc::new(Field::new("item", DataType::Int32, true));
        let nulls = NullBuffer::from(vec![true, false, true, true]);
        let array = FixedSizeListArray::try_new(field, 3, Arc::new(values), Some(nulls))
            .unwrap()
            .into_data();
        assert_fast_roundtrip(&array);
    }

    /// `Decimal128` is fast-path (buffer copied verbatim) but otherwise untested.
    #[test]
    fn roundtrip_decimal128() {
        use arrow::array::Decimal128Array;
        let array = Decimal128Array::from(vec![Some(12_345i128), None, Some(-9_876), Some(0)])
            .with_precision_and_scale(20, 4)
            .unwrap()
            .into_data();
        assert_fast_roundtrip(&array);
    }

    /// A temporal type (`Timestamp`) — also fast-path-verbatim, also untested.
    #[test]
    fn roundtrip_timestamp_temporal() {
        use arrow::array::TimestampMicrosecondArray;
        let array =
            TimestampMicrosecondArray::from(vec![Some(1_000_000i64), None, Some(2_500_000)])
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

    /// Regression: a Struct whose child `ArrayData` is *longer* than the struct
    /// (constructible via the low-level builder) must be sliced to the struct's
    /// len, otherwise the child field node declares the wrong row count and the
    /// stream is un-decodable.
    #[test]
    fn roundtrip_struct_with_oversized_child() {
        use arrow_schema::Fields;
        let child = Int32Array::from(vec![10, 20, 30]).into_data(); // len 3
        let fields: Fields = vec![Field::new("v", DataType::Int32, false)].into();
        let struct_data = ArrayData::builder(DataType::Struct(fields))
            .len(2) // shorter than the child
            .add_child_data(child)
            .build()
            .unwrap();

        assert!(
            ipc_fast_path_len(&struct_data).is_some(),
            "struct with an oversized child should stay on the fast path"
        );
        let decoded = read_official(&fast_encode(&struct_data));
        assert_eq!(decoded.len(), 2);
        let arr = arrow::array::make_array(decoded);
        let sa = arr.as_any().downcast_ref::<StructArray>().unwrap();
        let col = sa.column(0).as_any().downcast_ref::<Int32Array>().unwrap();
        assert_eq!(
            col.values(),
            &[10, 20],
            "child must be truncated to the struct's len"
        );
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
