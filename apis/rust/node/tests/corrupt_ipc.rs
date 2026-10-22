//! Corrupted Arrow IPC payloads must make both decoders return an error, never
//! panic or abort the process (#3863). The inputs were found by fuzzing.

use dora_node_api::IntoArrow;
use dora_node_api::arrow_utils::{
    IpcPayload, decode_arrow_ipc, decode_arrow_ipc_zero_copy, encode_arrow_ipc,
};

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

/// Both decoders reject `bytes` with an error.
fn assert_both_reject(bytes: &[u8]) {
    let copying = decode_arrow_ipc(bytes);
    assert!(
        copying.is_err(),
        "decode_arrow_ipc accepted a corrupt stream"
    );
    let zero_copy = decode_arrow_ipc_zero_copy(IpcPayload::from_vec(bytes.to_vec()));
    assert!(
        zero_copy.is_err(),
        "decode_arrow_ipc_zero_copy accepted a corrupt stream"
    );
}

/// 59 bytes that make arrow-ipc panic (`convert.rs:198`) while reading the
/// schema.
#[test]
fn a_stream_that_panics_arrow_is_an_error() {
    let bytes = hex(
        "ffffffff320000001000000000000a0000000000090004000a00000000000000000100000000000000000000000000000000000000000000000000",
    );
    assert_both_reject(&bytes);
    // The error says arrow panicked, and why.
    let err = decode_arrow_ipc_zero_copy(IpcPayload::from_vec(bytes))
        .unwrap_err()
        .to_string();
    assert!(err.contains("the decoder panicked: "), "got: {err}");
    assert!(!err.contains("unknown reason"), "got: {err}");
}

/// 320 bytes whose record batch claims a huge body: `StreamReader` used to
/// allocate that size up front and abort the process.
#[test]
fn a_fake_body_length_is_an_error_not_a_huge_allocation() {
    let bytes = hex(
        "ffffffff780000001000000000000a000c000a00090004000a00000010000000000104000800080000000400080000000400000001000000140000001000160010000e000f0004000000080010000000180000001c00000000000102180000000000060008000400060000000800000000000000040000006441746100000000ffffffffb8000000100000000c001a0018001700040008000c00000020000000800000000000400000000000000000000304000a0018000c00080004000a0000002c000000100000000300000000000000000000000100000003000000000000000000000000000000000000000200000000000000000000000100000000000000400000000000000003000000000000000000000000000000ffffff7f0000000000000000000000000000000000000000000000000000000000000000000000",
    );
    assert_both_reject(&bytes);
    // The body it claims is not there, so the stream is truncated.
    let err = decode_arrow_ipc(&bytes).unwrap_err().to_string();
    assert!(err.contains("truncated"), "got: {err}");
}

/// A valid stream cut inside its end-of-stream marker is truncated, so both
/// decoders reject it (the zero-copy one used to accept it).
#[test]
fn a_stream_cut_inside_the_end_marker_is_an_error() {
    let mut bytes = encode_arrow_ipc(&vec![1u64, 2, 3].into_arrow()).unwrap();
    bytes.truncate(bytes.len() - 4);
    assert_both_reject(&bytes);
}

/// A stream whose batch claims far more rows than any array in it could hold:
/// a struct with a non-nullable `Null` child and 2^57 rows. arrow validates it
/// by allocating a bitmap for every claimed row (2^54 bytes), which aborts the
/// process before arrow can reject the data (#3863, found by a Codex review).
fn stream_claiming_two_pow_57_rows() -> Vec<u8> {
    stream_claiming_rows((1u64 << 57).to_le_bytes())
}

/// The struct stream above with its row counts (batch length, both field-node
/// lengths and the child's null count) set to the given 8 bytes.
fn stream_claiming_rows(to: [u8; 8]) -> Vec<u8> {
    use arrow::array::{ArrayData, make_array};
    use arrow::datatypes::{DataType, Field, Schema};
    use std::sync::Arc;

    // Written with a small, easy-to-find row count, then patched: arrow's own
    // writer can't write 2^57 rows.
    let rows = 0x0102_0304_usize;
    let child = unsafe {
        ArrayData::new_unchecked(DataType::Null, rows, Some(rows), None, 0, vec![], vec![])
    };
    let ty = DataType::Struct(vec![Field::new("x", DataType::Null, false)].into());
    let parent = unsafe {
        ArrayData::new_unchecked(ty.clone(), rows, Some(0), None, 0, vec![], vec![child])
    };
    let schema = Arc::new(Schema::new(vec![Field::new("data", ty, true)]));
    let batch = unsafe {
        arrow::record_batch::RecordBatch::new_unchecked(
            schema.clone(),
            vec![make_array(parent)],
            rows,
        )
    };
    let mut bytes = Vec::new();
    let mut writer = arrow::ipc::writer::StreamWriter::try_new(&mut bytes, &schema).unwrap();
    writer.write(&batch).unwrap();
    writer.finish().unwrap();
    drop(writer);

    let from = (rows as u64).to_le_bytes();
    let mut patched = 0;
    let mut at = 0;
    while at + 8 <= bytes.len() {
        if bytes[at..at + 8] == from {
            bytes[at..at + 8].copy_from_slice(&to);
            patched += 1;
            at += 8;
        } else {
            at += 1;
        }
    }
    assert_eq!(
        patched, 4,
        "batch length + 2 field-node lengths + the child's null count"
    );
    bytes
}

#[test]
fn a_huge_row_count_is_an_error_not_an_abort() {
    let bytes = stream_claiming_two_pow_57_rows();
    assert_both_reject(&bytes);
    let err = decode_arrow_ipc(&bytes).unwrap_err().to_string();
    assert!(err.contains("rows"), "got: {err}");
}

/// The row limit doesn't reject a valid `Null` array (no per-row storage).
#[test]
fn a_null_array_still_decodes() {
    use arrow::array::{Array, NullArray};
    use arrow::datatypes::{DataType, Field, Schema};
    use std::sync::Arc;

    let schema = Arc::new(Schema::new(vec![Field::new("data", DataType::Null, true)]));
    let batch = arrow::record_batch::RecordBatch::try_new(
        schema.clone(),
        vec![Arc::new(NullArray::new(1_000_000))],
    )
    .unwrap();
    let mut bytes = Vec::new();
    let mut writer = arrow::ipc::writer::StreamWriter::try_new(&mut bytes, &schema).unwrap();
    writer.write(&batch).unwrap();
    writer.finish().unwrap();
    drop(writer);

    let decoded = decode_arrow_ipc(&bytes).unwrap();
    let decoded: arrow::array::ArrayRef = decoded.into();
    assert_eq!(decoded.len(), 1_000_000);
}

/// The same stream in arrow's older format, without the 0xFFFFFFFF marker
/// before each message, so every message header starts 4 bytes off an 8-byte
/// boundary. The row-count check must still see it.
#[test]
fn a_huge_row_count_in_the_legacy_format_is_an_error() {
    let modern = stream_claiming_two_pow_57_rows();
    let mut legacy = Vec::new();
    let mut at = 0;
    loop {
        assert_eq!(modern[at..at + 4], [0xff; 4]);
        let len = i32::from_le_bytes(modern[at + 4..at + 8].try_into().unwrap()) as usize;
        if len == 0 {
            legacy.extend_from_slice(&[0; 4]);
            break;
        }
        let message = arrow::ipc::root_as_message(&modern[at + 8..at + 8 + len]).unwrap();
        let end = at + 8 + len + message.bodyLength() as usize;
        legacy.extend_from_slice(&modern[at + 4..end]);
        at = end;
    }
    assert_both_reject(&legacy);
    let err = decode_arrow_ipc(&legacy).unwrap_err().to_string();
    assert!(err.contains("rows"), "got: {err}");
}

/// A negative row count is rejected too: arrow casts it to `usize`, which makes
/// it close to 2^64 and aborts the process the same way (#3863).
#[test]
fn a_negative_row_count_is_an_error_not_an_abort() {
    let bytes = stream_claiming_rows((-1i64).to_le_bytes());
    assert_both_reject(&bytes);
    let err = decode_arrow_ipc(&bytes).unwrap_err().to_string();
    assert!(err.contains("rows"), "got: {err}");
}

/// A delta dictionary adds to the dictionary a decoder keeps between
/// messages, so a peer could grow it without limit. dora never sends one.
#[test]
fn a_delta_dictionary_is_an_error() {
    let err = decode_arrow_ipc(&stream_with_a_delta_dictionary())
        .unwrap_err()
        .to_string();
    assert!(err.contains("delta"), "got: {err}");
}

/// Two batches of a dictionary column written with delta dictionaries: the
/// second dictionary message only carries the new value, marked `isDelta`.
pub fn stream_with_a_delta_dictionary() -> Vec<u8> {
    use arrow::array::{DictionaryArray, StringArray, UInt8Array};
    use arrow::datatypes::{DataType, Field, Schema};
    use arrow::ipc::writer::{DictionaryHandling, IpcWriteOptions, StreamWriter};
    use std::sync::Arc;

    let ty = DataType::Dictionary(Box::new(DataType::UInt8), Box::new(DataType::Utf8));
    let schema = Arc::new(Schema::new(vec![Field::new("data", ty, false)]));
    let first = DictionaryArray::new(
        UInt8Array::from(vec![0]),
        Arc::new(StringArray::from(vec!["a"])),
    );
    let second = DictionaryArray::new(
        UInt8Array::from(vec![1]),
        Arc::new(StringArray::from(vec!["a", "b"])),
    );
    let options = IpcWriteOptions::default().with_dictionary_handling(DictionaryHandling::Delta);
    let mut bytes = Vec::new();
    let mut writer = StreamWriter::try_new_with_options(&mut bytes, &schema, options).unwrap();
    for column in [first, second] {
        let batch =
            arrow::record_batch::RecordBatch::try_new(schema.clone(), vec![Arc::new(column)])
                .unwrap();
        writer.write(&batch).unwrap();
    }
    writer.finish().unwrap();
    drop(writer);
    let deltas = {
        let mut found = 0;
        let mut at = 0;
        while at + 8 <= bytes.len() {
            let len = i32::from_le_bytes(bytes[at + 4..at + 8].try_into().unwrap()) as usize;
            if len == 0 {
                break;
            }
            let message = arrow::ipc::root_as_message(&bytes[at + 8..at + 8 + len]).unwrap();
            if message
                .header_as_dictionary_batch()
                .is_some_and(|d| d.isDelta())
            {
                found += 1;
            }
            at += 8 + len + message.bodyLength() as usize;
        }
        found
    };
    assert_eq!(
        deltas, 1,
        "the writer must have written one delta dictionary"
    );
    bytes
}

/// Two columns pointing at the same unaligned bytes: arrow copies an unaligned
/// buffer once per column, so many columns sharing bytes make a small message
/// allocate many times its size. Every buffer must lie inside the message body
/// and together they may not be longer than it, which bounds that copying by the
/// payload size; no valid stream breaks this (#3863).
#[test]
fn buffers_that_do_not_fit_in_the_body_are_an_error() {
    let original = hex(
        "fffffffff80000001000000000000a000c000a00090004000a0000001000000000010400080008000000040008000000040000000100000004000000acffffff200000000c0000000000000d88000000020000004c0000000c0000000400040004000000d4ffffff10000000180000000000000214000000c4ffffff400000000000000100000000020000006331000010001400100000000f00040000000800100000001800000020000000000000021c00000008000c0004000b000800000040000000000000010000000002000000633000000400000064617461000000000000000000000000000000000000000000000000000000000000000000000000ffffffffd800000014000000000000000c001600140013000c0004000c0000001000000000000000140000000000000304000a0018000c00080004000a0000001400000068000000010000000000000000000000050000000000000000000000000000000000000000000000000000000000000000000000010000000000000008000000000000000000000000000000000000000000000001000000000000000800000000000000000000000300000001000000000000000000000000000000010000000000000000000000000000000100000000000000000000000000000007070707070707070707070707070707ffffffff00000000",
    );
    // Two 8-byte buffers at offset 1 of a 16-byte body: 16 bytes in total, not
    // more than the body, so it is allowed (it decodes or fails like before).
    let total_equals_body = decode_arrow_ipc(&original).map_err(|e| e.to_string());
    assert!(
        !matches!(&total_equals_body, Err(e) if e.contains("body")),
        "got: {total_equals_body:?}"
    );
    // 8 + 9 = 17 bytes of buffers in a 16-byte body.
    let longer = with_second_buffer(original.clone(), 1, 9);
    // A buffer ending at byte 17 of a 16-byte body.
    let outside = with_second_buffer(original, 9, 8);
    for bytes in [longer, outside] {
        assert_both_reject(&bytes);
        let err = decode_arrow_ipc(&bytes).unwrap_err().to_string();
        assert!(err.contains("body"), "got: {err}");
    }
}

/// `bytes` with the second buffer of its record batch changed: its
/// (offset, length) is the second place where the 16 bytes "offset 1, length 8"
/// appear in the message header.
fn with_second_buffer(mut bytes: Vec<u8>, offset: i64, length: i64) -> Vec<u8> {
    let mut needle = 1i64.to_le_bytes().to_vec();
    needle.extend_from_slice(&8i64.to_le_bytes());
    let at = (0..bytes.len() - 16)
        .filter(|&i| bytes[i..i + 16] == needle[..])
        .nth(1)
        .expect("two buffers at offset 1 with length 8");
    bytes[at..at + 8].copy_from_slice(&offset.to_le_bytes());
    bytes[at + 8..at + 16].copy_from_slice(&length.to_le_bytes());
    bytes
}
