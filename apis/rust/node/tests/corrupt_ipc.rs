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
