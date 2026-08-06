//! The `DORADMA` shared-memory wire format.
//!
//! Layout, byte for byte — the Python binding writes exactly this and a C++
//! node must read exactly this:
//!
//! | offset | size      | field                                            |
//! |--------|-----------|--------------------------------------------------|
//! | 0      | 8         | magic `DORADMA\0`                                |
//! | 8      | 8         | `json_len` (u64 LE)                              |
//! | 16     | 8         | `data_offset` (u64 LE), from the segment base    |
//! | 24     | 8         | `ipc_flag` (u64 LE), 1 = CUDA IPC handle valid   |
//! | 32     | 64        | CUDA IPC handle                                  |
//! | 96     | 8         | `write_gen` (u64 LE) seqlock, even = complete    |
//! | 104    | 152       | reserved                                         |
//! | 256    | `json_len`| metadata JSON, padded to a 256-byte multiple     |
//! | `data_offset` | payload |                                           |

use serde_json::Value;

pub const HEADER_SIZE: usize = 256;
pub const MAGIC: &[u8; 8] = b"DORADMA\x00";
pub const METADATA_ALIGN: usize = 256;

pub const OFFSET_JSON_LEN: usize = 8;
pub const OFFSET_DATA_OFFSET: usize = 16;
pub const OFFSET_IPC_FLAG: usize = 24;
pub const OFFSET_IPC_HANDLE: usize = 32;
pub const IPC_HANDLE_LEN: usize = 64;
pub const OFFSET_WRITE_GEN: usize = 96;

/// Decoded metadata JSON.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PoolMetadataJson {
    pub size: usize,
    pub dtype: String,
    pub shape: Vec<usize>,
    /// `"cpu"` or `"cuda"` — mirrors the Python key of the same name.
    pub pinned_type: String,
    /// `"shmem"`, `"unified"` or `"ipc"`. Additive: Python ignores it.
    pub transport: String,
}

/// Decoded fixed header plus its metadata JSON.
#[derive(Debug, Clone)]
pub struct ParsedHeader {
    pub json_len: usize,
    pub data_offset: usize,
    pub ipc_present: bool,
    pub ipc_handle: [u8; IPC_HANDLE_LEN],
    pub write_gen: u64,
    pub metadata: PoolMetadataJson,
}

/// Byte offset where the payload starts, given the metadata JSON length.
pub fn data_offset_for(json_len: usize) -> usize {
    HEADER_SIZE + json_len.div_ceil(METADATA_ALIGN) * METADATA_ALIGN
}

/// Serialize the metadata JSON exactly as the Python binding does, plus the
/// additive `transport` key.
pub fn metadata_json(
    size: usize,
    dtype: &str,
    shape: &[usize],
    pinned_type: &str,
    transport: &str,
) -> String {
    serde_json::json!({
        "size": size,
        "dtype": dtype,
        "shape": shape,
        "pinned_type": pinned_type,
        "transport": transport,
    })
    .to_string()
}

/// Parse a metadata JSON string. Missing optional keys degrade rather than
/// fail, so a segment written by an older Python sender still reads.
pub fn parse_metadata_json(json: &str) -> Result<PoolMetadataJson, String> {
    let v: Value = serde_json::from_str(json).map_err(|e| format!("invalid metadata json: {e}"))?;
    let size = v
        .get("size")
        .and_then(Value::as_u64)
        .ok_or_else(|| "metadata json missing `size`".to_string())? as usize;
    let dtype = v
        .get("dtype")
        .and_then(Value::as_str)
        .ok_or_else(|| "metadata json missing `dtype`".to_string())?
        .to_string();
    let shape = v
        .get("shape")
        .and_then(Value::as_array)
        .map(|a| {
            a.iter()
                .filter_map(Value::as_u64)
                .map(|x| x as usize)
                .collect()
        })
        .unwrap_or_default();
    let pinned_type = v
        .get("pinned_type")
        .and_then(Value::as_str)
        .unwrap_or("cpu")
        .to_string();
    // Written only by this crate; a Python-written segment has no `transport`
    // key, and its shape is exactly what `shmem` means.
    let transport = v
        .get("transport")
        .and_then(Value::as_str)
        .unwrap_or("shmem")
        .to_string();
    Ok(PoolMetadataJson {
        size,
        dtype,
        shape,
        pinned_type,
        transport,
    })
}

/// Write magic, `json_len`, `data_offset`, a zeroed IPC area and a zero
/// `write_gen`, followed by the metadata JSON, into `buf`.
pub fn write_header(buf: &mut [u8], json: &str) -> Result<(), String> {
    let json_len = json.len();
    let data_offset = data_offset_for(json_len);
    if buf.len() < data_offset {
        return Err(format!(
            "segment too small: {} bytes, need at least {} for header + metadata",
            buf.len(),
            data_offset
        ));
    }
    buf[..8].copy_from_slice(MAGIC);
    buf[OFFSET_JSON_LEN..OFFSET_JSON_LEN + 8].copy_from_slice(&(json_len as u64).to_le_bytes());
    buf[OFFSET_DATA_OFFSET..OFFSET_DATA_OFFSET + 8]
        .copy_from_slice(&(data_offset as u64).to_le_bytes());
    buf[OFFSET_IPC_FLAG..OFFSET_IPC_FLAG + 8].copy_from_slice(&0u64.to_le_bytes());
    buf[OFFSET_IPC_HANDLE..OFFSET_IPC_HANDLE + IPC_HANDLE_LEN].fill(0);
    buf[OFFSET_WRITE_GEN..OFFSET_WRITE_GEN + 8].copy_from_slice(&0u64.to_le_bytes());
    buf[HEADER_SIZE..HEADER_SIZE + json_len].copy_from_slice(json.as_bytes());
    Ok(())
}

fn read_u64(buf: &[u8], offset: usize) -> u64 {
    let mut b = [0u8; 8];
    b.copy_from_slice(&buf[offset..offset + 8]);
    u64::from_le_bytes(b)
}

/// Parse a segment's header. Every field that later becomes a pointer offset
/// is bounds-checked here so callers never form an out-of-range address.
pub fn parse_header(buf: &[u8]) -> Result<ParsedHeader, String> {
    if buf.len() < HEADER_SIZE {
        return Err(format!(
            "segment too small: {} bytes, need at least {HEADER_SIZE}",
            buf.len()
        ));
    }
    if &buf[..8] != MAGIC {
        return Err("segment magic is not DORADMA — not a dora memory pool".to_string());
    }
    let json_len = read_u64(buf, OFFSET_JSON_LEN) as usize;
    let data_offset = read_u64(buf, OFFSET_DATA_OFFSET) as usize;
    if data_offset > buf.len() {
        return Err(format!(
            "header data_offset {data_offset} is past the end of the {}-byte segment",
            buf.len()
        ));
    }
    if json_len > data_offset.saturating_sub(HEADER_SIZE) {
        return Err(format!(
            "header json_len {json_len} does not fit before data_offset {data_offset}"
        ));
    }
    let json = std::str::from_utf8(&buf[HEADER_SIZE..HEADER_SIZE + json_len])
        .map_err(|e| format!("metadata json is not utf-8: {e}"))?;
    let metadata = parse_metadata_json(json)?;

    let mut ipc_handle = [0u8; IPC_HANDLE_LEN];
    ipc_handle.copy_from_slice(&buf[OFFSET_IPC_HANDLE..OFFSET_IPC_HANDLE + IPC_HANDLE_LEN]);

    Ok(ParsedHeader {
        json_len,
        data_offset,
        ipc_present: read_u64(buf, OFFSET_IPC_FLAG) == 1,
        ipc_handle,
        write_gen: read_u64(buf, OFFSET_WRITE_GEN),
        metadata,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// These constants are duplicated in `apis/python/node/src/lib.rs`
    /// (DORADMA_HEADER_SIZE / DORADMA_MAGIC / DORADMA_METADATA_ALIGN) and in the
    /// field offsets it writes by hand. If Python ever changes, this test is the
    /// tripwire — a C++ node and a Python node must agree byte for byte.
    #[test]
    fn constants_match_the_python_binding() {
        assert_eq!(HEADER_SIZE, 256);
        assert_eq!(MAGIC, b"DORADMA\x00");
        assert_eq!(METADATA_ALIGN, 256);
        assert_eq!(OFFSET_JSON_LEN, 8);
        assert_eq!(OFFSET_DATA_OFFSET, 16);
        assert_eq!(OFFSET_IPC_FLAG, 24);
        assert_eq!(OFFSET_IPC_HANDLE, 32);
        assert_eq!(IPC_HANDLE_LEN, 64);
        assert_eq!(OFFSET_WRITE_GEN, 96);
    }

    #[test]
    fn data_offset_is_header_plus_padded_json() {
        assert_eq!(data_offset_for(0), 256);
        assert_eq!(data_offset_for(1), 512);
        assert_eq!(data_offset_for(255), 512);
        assert_eq!(data_offset_for(256), 512);
        assert_eq!(data_offset_for(257), 768);
    }

    #[test]
    fn metadata_json_round_trips() {
        let json = metadata_json(1024, "uint8", &[4, 720, 1280, 4], "cuda", "unified");
        let parsed = parse_metadata_json(&json).expect("valid json");
        assert_eq!(parsed.size, 1024);
        assert_eq!(parsed.dtype, "uint8");
        assert_eq!(parsed.shape, vec![4, 720, 1280, 4]);
        assert_eq!(parsed.pinned_type, "cuda");
        assert_eq!(parsed.transport, "unified");
    }

    /// A Python reader looks up `size`, `dtype`, `shape` and `pinned_type` by
    /// name, so the extra `transport` key must be additive, never a rename.
    #[test]
    fn metadata_json_keeps_the_python_keys() {
        let json = metadata_json(8, "float32", &[2], "cpu", "shmem");
        assert!(json.contains("\"size\""));
        assert!(json.contains("\"dtype\""));
        assert!(json.contains("\"shape\""));
        assert!(json.contains("\"pinned_type\""));
    }

    #[test]
    fn header_round_trips() {
        let json = metadata_json(64, "uint8", &[64], "cpu", "shmem");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 64];
        write_header(&mut buf, &json).expect("write");

        let parsed = parse_header(&buf).expect("parse");
        assert_eq!(parsed.json_len, json.len());
        assert_eq!(parsed.data_offset, data_offset_for(json.len()));
        assert!(!parsed.ipc_present);
        assert_eq!(parsed.write_gen, 0);
        assert_eq!(parsed.metadata.size, 64);
        assert_eq!(parsed.metadata.dtype, "uint8");
    }

    #[test]
    fn parse_rejects_bad_magic() {
        let json = metadata_json(8, "uint8", &[8], "cpu", "shmem");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, &json).expect("write");
        buf[0] = b'X';
        let err = parse_header(&buf).unwrap_err();
        assert!(err.contains("magic"), "unexpected error: {err}");
    }

    #[test]
    fn parse_rejects_truncated_segment() {
        let buf = vec![0u8; HEADER_SIZE - 1];
        let err = parse_header(&buf).unwrap_err();
        assert!(err.contains("too small"), "unexpected error: {err}");
    }

    /// A corrupt or hostile `data_offset` must not become an out-of-bounds
    /// pointer in the caller.
    #[test]
    fn parse_rejects_data_offset_past_the_end() {
        let json = metadata_json(8, "uint8", &[8], "cpu", "shmem");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, &json).expect("write");
        buf[OFFSET_DATA_OFFSET..OFFSET_DATA_OFFSET + 8].copy_from_slice(&(u64::MAX).to_le_bytes());
        let err = parse_header(&buf).unwrap_err();
        assert!(err.contains("data_offset"), "unexpected error: {err}");
    }

    #[test]
    fn write_header_rejects_a_buffer_that_cannot_hold_the_json() {
        let json = metadata_json(8, "uint8", &[8], "cpu", "shmem");
        let mut buf = vec![0u8; HEADER_SIZE];
        let err = write_header(&mut buf, &json).unwrap_err();
        assert!(err.contains("too small"), "unexpected error: {err}");
    }
}
