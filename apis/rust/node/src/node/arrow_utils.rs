//! Utility functions for converting Arrow arrays to/from raw data.
//!
use arrow::array::{ArrayData, BufferSpec};
use arrow_schema::DataType;
use dora_message::metadata::{ArrowTypeInfo, BufferOffset};
use eyre::Context;

/// Maximum Arrow IPC payload size (256 MB).
const MAX_IPC_BYTES: usize = 256 * 1024 * 1024;

/// Maximum recursion depth for nested Arrow types.
const MAX_TYPE_DEPTH: usize = 32;

/// Maximum number of buffer offsets or child arrays per level.
const MAX_ENTRIES: usize = 256;

/// Alignment guaranteed for every raw Arrow buffer inside Dora payloads.
///
/// Arrow kernels can issue SIMD loads from buffer bases. Some ARM platforms
/// fault on under-aligned SIMD loads, so the raw data-plane format keeps every
/// buffer start at a 64-byte boundary relative to the payload base.
pub(crate) const ARROW_BUFFER_ALIGNMENT: usize = 64;
pub(crate) const ARROW_BUFFER_ALIGNMENT_EXPONENT: u8 =
    ARROW_BUFFER_ALIGNMENT.trailing_zeros() as u8;
const _: () = assert!(ARROW_BUFFER_ALIGNMENT.is_power_of_two());

fn payload_buffer_alignment(spec: &BufferSpec) -> usize {
    let arrow_alignment = match *spec {
        BufferSpec::FixedWidth { alignment, .. } => alignment,
        BufferSpec::VariableWidth | BufferSpec::BitMap | BufferSpec::AlwaysNull => 1,
    };
    arrow_alignment.max(ARROW_BUFFER_ALIGNMENT)
}

/// Compute a hash of an Arrow [`DataType`] for fast type matching.
///
/// Uses the `Debug` representation which is stable for the same type within
/// a given Arrow version. Receivers can compare this O(1) value before
/// performing a full structural type comparison.
///
/// Uses FNV-1a with a fixed seed for cross-process determinism (unlike
/// `DefaultHasher` which may be randomized per process).
pub fn compute_schema_hash(data_type: &DataType) -> u64 {
    use std::fmt::Write;

    // FNV-1a accumulator that hashes the `Debug` formatting of the type as it
    // is written, without materializing the intermediate `String`. This is
    // called once per array node (recursively over children) on every
    // `send_output`, so avoiding the per-message heap allocation matters on the
    // latency-critical path. The byte stream — and therefore the resulting
    // hash — is identical to hashing `format!("{data_type:?}")` directly.
    struct FnvWriter(u64);
    impl Write for FnvWriter {
        fn write_str(&mut self, s: &str) -> std::fmt::Result {
            for byte in s.bytes() {
                self.0 ^= byte as u64;
                self.0 = self.0.wrapping_mul(0x100000001b3); // FNV prime
            }
            Ok(())
        }
    }

    let mut writer = FnvWriter(0xcbf29ce484222325); // FNV offset basis
    // `Debug` for `DataType` and our `write_str` are both infallible.
    let _ = write!(writer, "{data_type:?}");
    writer.0
}

/// Calculates the data size in bytes required for storing a continuous copy of the given Arrow
/// array.
pub fn required_data_size(array: &ArrayData) -> usize {
    let mut next_offset = 0;
    required_data_size_inner(array, &mut next_offset);
    next_offset
}
fn required_data_size_inner(array: &ArrayData, next_offset: &mut usize) {
    let layout = arrow::array::layout(array.data_type());
    for (buffer, spec) in array.buffers().iter().zip(&layout.buffers) {
        let alignment = payload_buffer_alignment(spec);
        *next_offset = (*next_offset).div_ceil(alignment) * alignment;
        *next_offset += buffer.len();
    }
    for child in array.child_data() {
        required_data_size_inner(child, next_offset);
    }
}

/// Copy the given Arrow array into the provided buffer.
///
/// If the Arrow array consists of multiple buffers, they are placed continuously in the target
/// buffer (there might be some padding for alignment)
///
/// Panics if the buffer is not large enough.
pub fn copy_array_into_sample(target_buffer: &mut [u8], arrow_array: &ArrayData) -> ArrowTypeInfo {
    let mut next_offset = 0;
    copy_array_into_sample_inner(target_buffer, &mut next_offset, arrow_array)
}

fn copy_array_into_sample_inner(
    target_buffer: &mut [u8],
    next_offset: &mut usize,
    arrow_array: &ArrayData,
) -> ArrowTypeInfo {
    let mut buffer_offsets = Vec::new();
    let layout = arrow::array::layout(arrow_array.data_type());
    for (buffer, spec) in arrow_array.buffers().iter().zip(&layout.buffers) {
        let len = buffer.len();
        let alignment = payload_buffer_alignment(spec);
        *next_offset = (*next_offset).div_ceil(alignment) * alignment;

        assert!(
            target_buffer[*next_offset..].len() >= len,
            "target buffer too small (total_len: {}, offset: {}, required_len: {len})",
            target_buffer.len(),
            *next_offset,
        );

        target_buffer[*next_offset..][..len].copy_from_slice(buffer.as_slice());
        buffer_offsets.push(BufferOffset {
            offset: *next_offset,
            len,
        });
        *next_offset += len;
    }

    let mut child_data = Vec::new();
    for child in arrow_array.child_data() {
        let child_type_info = copy_array_into_sample_inner(target_buffer, next_offset, child);
        child_data.push(child_type_info);
    }

    ArrowTypeInfo {
        data_type: arrow_array.data_type().clone(),
        len: arrow_array.len(),
        null_count: arrow_array.null_count(),
        validity: arrow_array.nulls().map(|b| b.validity().to_owned()),
        offset: arrow_array.offset(),
        buffer_offsets,
        child_data,
        field_names: None,
        schema_hash: Some(compute_schema_hash(arrow_array.data_type())),
    }
}

/// Tries to convert the given raw Arrow buffer into an Arrow array.
///
/// The `type_info` is required for decoding the `raw_buffer` correctly.
pub fn buffer_into_arrow_array(
    raw_buffer: &arrow::buffer::Buffer,
    type_info: &ArrowTypeInfo,
) -> eyre::Result<arrow::array::ArrayData> {
    buffer_into_arrow_array_inner(raw_buffer, type_info, 0)
}

/// A zero-length Arrow buffer that is aligned for any fixed-width type.
///
/// `MutableBuffer` allocates with Arrow's 64-byte alignment, so the resulting
/// (empty) buffer satisfies the alignment checks `ArrayData::try_new` performs
/// even for length-0 buffers — unlike a buffer sliced out of an empty payload.
fn aligned_empty_buffer() -> arrow::buffer::Buffer {
    arrow::buffer::MutableBuffer::new(0).into()
}

fn buffer_into_arrow_array_inner(
    raw_buffer: &arrow::buffer::Buffer,
    type_info: &ArrowTypeInfo,
    depth: usize,
) -> eyre::Result<arrow::array::ArrayData> {
    if depth > MAX_TYPE_DEPTH {
        eyre::bail!("Arrow type nesting depth exceeds maximum ({MAX_TYPE_DEPTH})");
    }

    // NOTE: do not special-case an empty `raw_buffer` here. A zero-footprint
    // array (e.g. `NullArray::new(n)`, a struct whose only fields are
    // `Null`-typed, or a zero-length typed array) serializes to an empty
    // payload while still carrying a meaningful `type_info.len`. Falling back
    // to `ArrayData::new_empty` would discard that length (and `offset`,
    // `validity`, `child_data`), silently truncating the array to length 0
    // (dora-rs/dora#2083). The general path below honors all of them by
    // rebuilding through `ArrayData::try_new`, which keeps the declared length.

    if type_info.buffer_offsets.len() > MAX_ENTRIES {
        eyre::bail!(
            "too many buffer offsets: {} (max {MAX_ENTRIES})",
            type_info.buffer_offsets.len()
        );
    }
    if type_info.child_data.len() > MAX_ENTRIES {
        eyre::bail!(
            "too many child arrays: {} (max {MAX_ENTRIES})",
            type_info.child_data.len()
        );
    }

    let mut buffers = Vec::new();
    for BufferOffset { offset, len } in &type_info.buffer_offsets {
        if *len == 0 {
            // A zero-length buffer carries no data; slicing it out of the
            // payload risks producing an under-aligned pointer (the payload
            // base may be empty/dangling), which `ArrayData::try_new` rejects
            // for SIMD-aligned types. Substitute a freshly allocated, properly
            // aligned empty buffer instead. This is what lets a zero-footprint
            // array round-trip with its true length (dora-rs/dora#2083).
            buffers.push(aligned_empty_buffer());
            continue;
        }
        // Use checked_add to guard against malicious/buggy peers sending
        // `offset` or `len` values that would overflow `usize` on addition
        // and bypass the bounds check below.
        let end = offset
            .checked_add(*len)
            .ok_or_else(|| eyre::eyre!("buffer offset overflow: offset={offset}, len={len}"))?;
        if end > raw_buffer.len() {
            eyre::bail!(
                "buffer offset out of bounds: offset={offset}, len={len}, buffer_len={}",
                raw_buffer.len()
            );
        }
        buffers.push(raw_buffer.slice_with_length(*offset, *len));
    }

    let mut child_data = Vec::new();
    for child_type_info in &type_info.child_data {
        child_data.push(buffer_into_arrow_array_inner(
            raw_buffer,
            child_type_info,
            depth + 1,
        )?)
    }

    arrow::array::ArrayData::try_new(
        type_info.data_type.clone(),
        type_info.len,
        type_info
            .validity
            .clone()
            .map(arrow::buffer::Buffer::from_vec),
        type_info.offset,
        buffers,
        child_data,
    )
    .context("Error creating Arrow array")
}

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

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{Array, ArrayRef, StringArray, StructArray, UInt64Array};
    use arrow::datatypes::Field;
    use std::sync::Arc;

    fn assert_buffer_offsets_are_aligned(type_info: &ArrowTypeInfo) {
        for offset in &type_info.buffer_offsets {
            assert_eq!(
                offset.offset % ARROW_BUFFER_ALIGNMENT,
                0,
                "buffer offset {offset:?} is not {ARROW_BUFFER_ALIGNMENT}-byte aligned"
            );
        }
        for child in &type_info.child_data {
            assert_buffer_offsets_are_aligned(child);
        }
    }

    #[test]
    fn ipc_roundtrip_primitive() {
        let array = UInt64Array::from(vec![1, 2, 3, 4, 5]);
        let data = array.into_data();
        let encoded = encode_arrow_ipc(&data).unwrap();
        let decoded = decode_arrow_ipc(&encoded).unwrap();
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
    fn schema_hash_is_deterministic() {
        let dt = DataType::UInt64;
        let h1 = compute_schema_hash(&dt);
        let h2 = compute_schema_hash(&dt);
        assert_eq!(h1, h2);
    }

    #[test]
    fn schema_hash_matches_debug_fnv_for_nested_types() {
        // Guards the allocation-free `fmt::Write` accumulator against the
        // straightforward `format!("{dt:?}")` + FNV-1a reference. The two must
        // produce identical hashes for arbitrary (including deeply nested)
        // types so the value stays stable across the optimization and across
        // processes running the same build.
        fn reference(data_type: &DataType) -> u64 {
            let s = format!("{data_type:?}");
            let mut hash: u64 = 0xcbf29ce484222325;
            for byte in s.bytes() {
                hash ^= byte as u64;
                hash = hash.wrapping_mul(0x100000001b3);
            }
            hash
        }

        let types = [
            DataType::Null,
            DataType::UInt8,
            DataType::Utf8,
            DataType::List(Arc::new(Field::new("item", DataType::Int32, true))),
            DataType::Struct(
                vec![
                    Field::new("a", DataType::Float64, false),
                    Field::new(
                        "b",
                        DataType::List(Arc::new(Field::new("item", DataType::UInt16, true))),
                        true,
                    ),
                ]
                .into(),
            ),
        ];
        for dt in &types {
            assert_eq!(
                compute_schema_hash(dt),
                reference(dt),
                "mismatch for {dt:?}"
            );
        }
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

    #[test]
    fn raw_payload_offsets_are_64_byte_aligned_recursively() {
        use arrow_schema::{DataType, Field};

        let array = StructArray::from(vec![
            (
                Arc::new(Field::new("values", DataType::UInt64, false)),
                Arc::new(UInt64Array::from(vec![1, 2, 3])) as ArrayRef,
            ),
            (
                Arc::new(Field::new("labels", DataType::Utf8, true)),
                Arc::new(StringArray::from(vec![Some("a"), None, Some("bbb")])) as ArrayRef,
            ),
        ]);
        let data = array.into_data();
        let mut sample = vec![0; required_data_size(&data)];
        let type_info = copy_array_into_sample(&mut sample, &data);

        assert_buffer_offsets_are_aligned(&type_info);
        let decoded =
            buffer_into_arrow_array(&arrow::buffer::Buffer::from(sample), &type_info).unwrap();
        assert_eq!(data, decoded);
    }

    #[test]
    fn buffer_into_arrow_array_rejects_overflow_offset() {
        // A peer-controlled BufferOffset with offset = usize::MAX would overflow
        // `offset + len` to a small value and bypass the bounds check, then panic
        // inside arrow's slice_with_length. We must reject it with an error.
        let info = ArrowTypeInfo {
            data_type: arrow_schema::DataType::UInt8,
            len: 0,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![BufferOffset {
                offset: usize::MAX,
                len: 1,
            }],
            child_data: vec![],
            field_names: None,
            schema_hash: None,
        };
        let buf = arrow::buffer::Buffer::from(vec![0u8; 1024]);
        let err = buffer_into_arrow_array(&buf, &info).unwrap_err();
        assert!(
            err.to_string().contains("overflow"),
            "expected overflow error, got: {err}"
        );
    }

    #[test]
    fn buffer_into_arrow_array_rejects_deep_nesting() {
        use std::sync::Arc;

        let mut info = ArrowTypeInfo {
            data_type: arrow_schema::DataType::UInt8,
            len: 0,
            null_count: 0,
            validity: None,
            offset: 0,
            buffer_offsets: vec![],
            child_data: vec![],
            field_names: None,
            schema_hash: None,
        };
        // Nest 40 levels deep (exceeds MAX_TYPE_DEPTH of 32)
        for _ in 0..40 {
            info = ArrowTypeInfo {
                data_type: arrow_schema::DataType::List(Arc::new(arrow_schema::Field::new(
                    "item",
                    arrow_schema::DataType::UInt8,
                    true,
                ))),
                len: 0,
                null_count: 0,
                validity: None,
                offset: 0,
                buffer_offsets: vec![],
                child_data: vec![info],
                field_names: None,
                schema_hash: None,
            };
        }
        let buf = arrow::buffer::Buffer::from(vec![0u8; 1024]);
        let result = buffer_into_arrow_array(&buf, &info);
        assert!(result.is_err());
        assert!(result.unwrap_err().to_string().contains("depth"));
    }

    // Raw-buffer round-trip beyond the offset-0 case (dora-rs/dora#2027 verified
    // test gap): a sliced (non-zero `offset`) array and the empty-buffer
    // early-return path.

    #[test]
    fn raw_roundtrip_preserves_nonzero_offset() {
        // Slice the `ArrayData` so it carries a non-zero logical `offset`.
        let data = UInt64Array::from(vec![10, 20, 30, 40, 50])
            .into_data()
            .slice(2, 2); // offset = 2, len = 2 -> [30, 40]
        assert_eq!(data.offset(), 2, "precondition: array is sliced");

        let mut sample = vec![0; required_data_size(&data)];
        let type_info = copy_array_into_sample(&mut sample, &data);
        let decoded =
            buffer_into_arrow_array(&arrow::buffer::Buffer::from(sample), &type_info).unwrap();
        assert_eq!(data, decoded);
    }

    #[test]
    fn raw_roundtrip_empty_primitive_uses_empty_buffer_path() {
        let data = UInt64Array::from(Vec::<u64>::new()).into_data();
        let mut sample = vec![0; required_data_size(&data)];
        let type_info = copy_array_into_sample(&mut sample, &data);
        // An empty array yields a zero-length payload, exercising the
        // `raw_buffer.is_empty()` early-return in `buffer_into_arrow_array`.
        let decoded =
            buffer_into_arrow_array(&arrow::buffer::Buffer::from(sample), &type_info).unwrap();
        assert_eq!(decoded.len(), 0);
        assert_eq!(decoded.data_type(), &DataType::UInt64);
    }

    #[test]
    fn raw_roundtrip_preserves_nullarray_length() {
        // dora-rs/dora#2083: a `NullArray::new(n>0)` has no data buffers and no
        // validity, so its serialized footprint is empty — but its logical
        // length must survive the raw round-trip instead of collapsing to 0.
        use arrow::array::NullArray;
        let data = NullArray::new(5).into_data();
        assert_eq!(data.len(), 5);
        let mut sample = vec![0; required_data_size(&data)];
        assert!(sample.is_empty(), "precondition: zero-footprint payload");
        let type_info = copy_array_into_sample(&mut sample, &data);
        assert_eq!(type_info.len, 5);
        let decoded =
            buffer_into_arrow_array(&arrow::buffer::Buffer::from(sample), &type_info).unwrap();
        assert_eq!(decoded.len(), 5, "NullArray length must be preserved");
        assert_eq!(decoded.data_type(), &DataType::Null);
    }

    #[test]
    fn raw_roundtrip_preserves_struct_of_nulls_length() {
        // dora-rs/dora#2083: a struct whose only field is `Null`-typed also has
        // a zero-byte footprint while carrying a non-zero length.
        use arrow::array::NullArray;
        use arrow_schema::Field;
        let child = Arc::new(NullArray::new(3)) as ArrayRef;
        let data = StructArray::from(vec![(
            Arc::new(Field::new("nulls", DataType::Null, true)),
            child,
        )])
        .into_data();
        assert_eq!(data.len(), 3);
        let mut sample = vec![0; required_data_size(&data)];
        assert!(sample.is_empty(), "precondition: zero-footprint payload");
        let type_info = copy_array_into_sample(&mut sample, &data);
        let decoded =
            buffer_into_arrow_array(&arrow::buffer::Buffer::from(sample), &type_info).unwrap();
        assert_eq!(decoded.len(), 3, "struct-of-nulls length must be preserved");
        assert_eq!(decoded.child_data()[0].len(), 3);
    }

    #[test]
    fn raw_roundtrip_empty_nested_struct() {
        use arrow_schema::Field;
        let empty_child = Arc::new(UInt64Array::from(Vec::<u64>::new())) as ArrayRef;
        let data = StructArray::from(vec![(
            Arc::new(Field::new("values", DataType::UInt64, false)),
            empty_child,
        )])
        .into_data();
        let mut sample = vec![0; required_data_size(&data)];
        let type_info = copy_array_into_sample(&mut sample, &data);
        let decoded =
            buffer_into_arrow_array(&arrow::buffer::Buffer::from(sample), &type_info).unwrap();
        assert_eq!(decoded.len(), 0);
        assert!(matches!(decoded.data_type(), DataType::Struct(_)));
    }
}
