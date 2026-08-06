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
    /// `"shmem"`, `"unified"` or `"ipc"`. Additive: Python never writes this
    /// key. `parse_metadata_json` leaves it empty (`String::new()`) when
    /// absent — it has no way to know whether the segment is IPC-backed —
    /// and `parse_header` fills it in from the header's `ipc_flag` before
    /// returning, so callers only ever see the resolved value. If a value
    /// IS present, `parse_header` requires it to agree with `ipc_flag`
    /// (`transport == "ipc"` iff `ipc_flag == 1`) and rejects the header
    /// otherwise — the flag, not the JSON, is authoritative because it
    /// alone gates the payload-size bounds check.
    pub transport: String,
}

/// Decoded fixed header plus its metadata JSON.
#[derive(Debug, Clone)]
pub struct ParsedHeader {
    pub json_len: usize,
    pub data_offset: usize,
    pub ipc_present: bool,
    /// The 64-byte CUDA IPC handle. `Some` only when `ipc_present`; a
    /// non-IPC segment's IPC-handle bytes carry no meaning (they are
    /// zeroed by `write_header`, but a caller must not treat a zeroed
    /// handle as if it were a real one).
    pub ipc_handle: Option<[u8; IPC_HANDLE_LEN]>,
    pub write_gen: u64,
    pub metadata: PoolMetadataJson,
}

/// Byte offset where the payload starts, given the metadata JSON length.
///
/// Uses saturating arithmetic: a hostile or corrupted `json_len` must not
/// panic (debug) or wrap around to a small offset (release).
pub fn data_offset_for(json_len: usize) -> usize {
    let padded_json_len = json_len
        .div_ceil(METADATA_ALIGN)
        .saturating_mul(METADATA_ALIGN);
    HEADER_SIZE.saturating_add(padded_json_len)
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
/// fail, so a segment written by an older Python sender still reads. A key
/// that IS present but has the wrong JSON type is a malformed header, not a
/// missing-key degradation, and is rejected rather than silently defaulted.
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
    // `filter_map` would silently drop bad elements (`[2,"x",3]` -> `[2,3]`),
    // turning a 3-D tensor into a plausible-looking 2-D one. Reject instead.
    let shape = match v.get("shape") {
        None => Vec::new(),
        Some(val) => {
            let arr = val
                .as_array()
                .ok_or_else(|| format!("metadata json `shape` is not an array: {val}"))?;
            let mut shape = Vec::with_capacity(arr.len());
            for elem in arr {
                let n = elem.as_u64().ok_or_else(|| {
                    format!("metadata json `shape` contains a non-integer element: {elem}")
                })?;
                shape.push(n as usize);
            }
            shape
        }
    };
    // Absent means Python's own `effective_as_cuda` default: no
    // `pinned_type` key reads as CUDA on the Python side
    // (apis/python/node/src/lib.rs:3700), so this default must mirror
    // that polarity, not "cpu".
    let pinned_type = match v.get("pinned_type") {
        None => "cuda".to_string(),
        Some(val) => val
            .as_str()
            .ok_or_else(|| format!("metadata json `pinned_type` is not a string: {val}"))?
            .to_string(),
    };
    // Written only by this crate; a Python-written segment has no
    // `transport` key. Left empty here — resolving it needs the header's
    // `ipc_flag`, which this function does not see — and filled in by
    // `parse_header`.
    let transport = v
        .get("transport")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();
    Ok(PoolMetadataJson {
        size,
        dtype,
        shape,
        pinned_type,
        transport,
    })
}

/// Write magic, `json_len`, `data_offset`, a zeroed IPC area, a zero
/// `write_gen`, the zeroed reserved region and JSON padding, followed by the
/// metadata JSON, into `buf`.
///
/// `buf` is an arbitrary caller-supplied slice, not assumed to be
/// freshly-zeroed shared memory — every byte this function's format
/// documents is written explicitly, including the reserved and padding
/// ranges, so no stale data from a reused segment can leak into a field a
/// reader trusts.
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
    buf[OFFSET_WRITE_GEN + 8..HEADER_SIZE].fill(0);
    buf[HEADER_SIZE..HEADER_SIZE + json_len].copy_from_slice(json.as_bytes());
    buf[HEADER_SIZE + json_len..data_offset].fill(0);
    Ok(())
}

fn read_u64(buf: &[u8], offset: usize) -> u64 {
    let mut b = [0u8; 8];
    b.copy_from_slice(&buf[offset..offset + 8]);
    u64::from_le_bytes(b)
}

/// Parse a segment's header.
///
/// Every field that later becomes a pointer offset or a bounds is checked
/// here: `data_offset` (must land inside the segment and past the fixed
/// header) and `json_len` (must fit between the header and `data_offset`).
/// The metadata's `size` is also checked, EXCEPT when `ipc_present`: a
/// Python CUDA-receiver pool allocates a header-only segment
/// (`apis/python/node/src/lib.rs:2005`, `total_size = data_offset`) whose
/// JSON `size` is the byte count of the tensor living in the IPC-imported
/// GPU buffer, not of anything in this segment's (nonexistent) data region
/// — so `size` cannot be validated against `buf.len()` in that case.
///
/// The metadata JSON is decoded whole in this same pass rather than in a
/// separate step: the JSON region is written once, by `write_header`,
/// before the segment is registered with the daemon, and a consumer only
/// learns a pool exists through that registration — so a reader can never
/// observe a half-written JSON region. Only the payload and `write_gen`
/// change after registration. There is therefore no torn-read hazard here
/// that a fixed-header/metadata split would guard against.
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
    if data_offset < HEADER_SIZE {
        return Err(format!(
            "header data_offset {data_offset} is less than HEADER_SIZE {HEADER_SIZE} — would overlap the fixed header"
        ));
    }
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
    let mut metadata = parse_metadata_json(json)?;

    let ipc_present = read_u64(buf, OFFSET_IPC_FLAG) == 1;

    if !ipc_present {
        // Mirrors the Python read path (apis/python/node/src/lib.rs:3677),
        // which gates this same check on `ipc_present != 1` and uses
        // saturating arithmetic against a corrupted/hostile `data_offset`.
        let available = buf.len().saturating_sub(data_offset);
        if metadata.size > available {
            return Err(format!(
                "metadata size {} exceeds the {available}-byte data region available after data_offset {data_offset} in the {}-byte segment",
                metadata.size,
                buf.len()
            ));
        }
    }

    if metadata.transport.is_empty() {
        // Python never writes a `transport` key. Resolve it from the
        // header bit `parse_metadata_json` could not see: a Python CUDA
        // pool is ipc_flag=1 and header-only ("ipc"); a Python CPU pool is
        // ipc_flag=0 with payload in the shmem data region ("shmem").
        metadata.transport = if ipc_present { "ipc" } else { "shmem" }.to_string();
    } else if (metadata.transport == "ipc") != ipc_present {
        // `ipc_flag` and `transport` are both attacker-controlled bytes in
        // the same world-writable segment, and they must not be allowed to
        // disagree exactly where it matters: `ipc_flag` gates the size
        // check above, so `ipc_flag=1` with `transport="shmem"` would tell
        // a consumer the (nonexistent) data region holds an oversized
        // payload — the header-only allocation this function's own size
        // check exists to catch. The flag is authoritative (`"unified"`
        // legitimately pairs with `ipc_flag=0` and keeps its size check;
        // only a `transport` claiming "ipc" while the flag disagrees, or
        // vice versa, is a contradiction).
        return Err(format!(
            "metadata transport `{}` contradicts header ipc_flag {}",
            metadata.transport, ipc_present as u8
        ));
    }

    let ipc_handle = if ipc_present {
        let mut h = [0u8; IPC_HANDLE_LEN];
        h.copy_from_slice(&buf[OFFSET_IPC_HANDLE..OFFSET_IPC_HANDLE + IPC_HANDLE_LEN]);
        Some(h)
    } else {
        None
    };

    Ok(ParsedHeader {
        json_len,
        data_offset,
        ipc_present,
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

    /// Python never writes a `transport` key. When `ipc_flag = 1` (a
    /// GPU-DMA pool) the absent key must resolve to `"ipc"`.
    #[test]
    fn absent_transport_with_ipc_flag_resolves_to_ipc() {
        let json = r#"{"size":8,"dtype":"uint8","shape":[8],"pinned_type":"cuda"}"#;
        let mut buf = vec![0u8; data_offset_for(json.len())];
        write_header(&mut buf, json).expect("write");
        buf[OFFSET_IPC_FLAG..OFFSET_IPC_FLAG + 8].copy_from_slice(&1u64.to_le_bytes());

        let parsed = parse_header(&buf).expect("parse");
        assert_eq!(parsed.metadata.transport, "ipc");
    }

    /// Same absent key, but `ipc_flag = 0` (a plain CPU shmem pool): must
    /// resolve to `"shmem"`.
    #[test]
    fn absent_transport_without_ipc_flag_resolves_to_shmem() {
        let json = r#"{"size":8,"dtype":"uint8","shape":[8],"pinned_type":"cpu"}"#;
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, json).expect("write");

        let parsed = parse_header(&buf).expect("parse");
        assert_eq!(parsed.metadata.transport, "shmem");
    }

    /// Absent `pinned_type` must mirror Python's `effective_as_cuda`
    /// default (no key reads as CUDA), not "cpu".
    #[test]
    fn absent_pinned_type_defaults_to_cuda() {
        let json = r#"{"size":8,"dtype":"uint8","shape":[8]}"#;
        let parsed = parse_metadata_json(json).expect("valid json");
        assert_eq!(parsed.pinned_type, "cuda");
    }

    #[test]
    fn pinned_type_present_but_not_a_string_is_rejected() {
        let json = r#"{"size":8,"dtype":"uint8","shape":[8],"pinned_type":42}"#;
        let err = parse_metadata_json(json).unwrap_err();
        assert!(err.contains("pinned_type"), "unexpected error: {err}");
    }

    /// `filter_map` would silently turn `[2,"x",3]` into `[2,3]`, making a
    /// 3-D tensor look like a plausible 2-D one. Must error instead.
    #[test]
    fn shape_containing_a_non_integer_is_rejected() {
        let json = r#"{"size":8,"dtype":"uint8","shape":[2,"x",3],"pinned_type":"cpu"}"#;
        let err = parse_metadata_json(json).unwrap_err();
        assert!(err.contains("shape"), "unexpected error: {err}");
    }

    #[test]
    fn shape_present_but_not_an_array_is_rejected() {
        let json = r#"{"size":8,"dtype":"uint8","shape":8,"pinned_type":"cpu"}"#;
        let err = parse_metadata_json(json).unwrap_err();
        assert!(err.contains("shape"), "unexpected error: {err}");
    }

    #[test]
    fn parse_rejects_non_utf8_metadata_json() {
        let json = metadata_json(8, "uint8", &[8], "cpu", "shmem");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, &json).expect("write");
        buf[HEADER_SIZE] = 0xFF; // invalid UTF-8 lead byte, overwrites the leading `{`
        let err = parse_header(&buf).unwrap_err();
        assert!(err.contains("utf-8"), "unexpected error: {err}");
    }

    #[test]
    fn parse_rejects_data_offset_before_the_header_ends() {
        let json = metadata_json(8, "uint8", &[8], "cpu", "shmem");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, &json).expect("write");
        buf[OFFSET_DATA_OFFSET..OFFSET_DATA_OFFSET + 8].copy_from_slice(&0u64.to_le_bytes());
        let err = parse_header(&buf).unwrap_err();
        // `data_offset = 0` also fails the later "json_len does not fit"
        // check, whose message happens to contain "data_offset" too —
        // assert the specific cause this test targets, not a substring
        // both the fixed and unfixed code would produce.
        assert!(
            err.contains("less than HEADER_SIZE"),
            "unexpected error: {err}"
        );
    }

    /// `size` overrunning the segment is rejected for a plain shmem pool,
    /// but the exact same oversized `size` is the normal shape of a Python
    /// CUDA-receiver pool — header-only segment, `ipc_flag = 1`, `size`
    /// describing a tensor that actually lives in the IPC-imported GPU
    /// buffer — and must parse. No `transport` key, matching a real
    /// Python-written segment, so the flip is driven by `ipc_flag` alone
    /// and cannot trip the transport/ipc_flag contradiction check.
    #[test]
    fn oversized_size_is_rejected_for_shmem_but_allowed_for_ipc() {
        let json = r#"{"size":1000000,"dtype":"uint8","shape":[1000000],"pinned_type":"cpu"}"#;
        let data_offset = data_offset_for(json.len());
        let mut buf = vec![0u8; data_offset]; // header-only: no room for the payload
        write_header(&mut buf, json).expect("write");

        let err = parse_header(&buf).unwrap_err();
        assert!(err.contains("size"), "unexpected error: {err}");

        buf[OFFSET_IPC_FLAG..OFFSET_IPC_FLAG + 8].copy_from_slice(&1u64.to_le_bytes());
        let parsed = parse_header(&buf).expect("ipc-present header-only segment must parse");
        assert_eq!(parsed.metadata.size, 1_000_000);
        assert!(parsed.ipc_present);
    }

    /// The exploit shape from the code-review finding: `ipc_flag = 1`
    /// skips the size check, and a present `transport: "shmem"` must not
    /// then be allowed to tell a consumer the (nonexistent) data region
    /// holds the oversized payload.
    #[test]
    fn transport_shmem_contradicting_ipc_flag_is_rejected() {
        let json = metadata_json(83_886_080, "uint8", &[83_886_080], "cpu", "shmem");
        let mut buf = vec![0u8; data_offset_for(json.len())]; // header-only
        write_header(&mut buf, &json).expect("write");
        buf[OFFSET_IPC_FLAG..OFFSET_IPC_FLAG + 8].copy_from_slice(&1u64.to_le_bytes());

        let err = parse_header(&buf).unwrap_err();
        assert!(err.contains("contradicts"), "unexpected error: {err}");
    }

    /// The reverse direction: `transport: "ipc"` claims the size check is
    /// skippable, but `ipc_flag = 0` says the data region is real and must
    /// be validated. Also a contradiction.
    #[test]
    fn transport_ipc_contradicting_ipc_flag_is_rejected() {
        // Room for the 8-byte payload, so the size check (gated on
        // ipc_flag=0) passes and the transport/ipc_flag contradiction is
        // what actually fires.
        let json = metadata_json(8, "uint8", &[8], "cpu", "ipc");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8]; // ipc_flag left at 0
        write_header(&mut buf, &json).expect("write");

        let err = parse_header(&buf).unwrap_err();
        assert!(err.contains("contradicts"), "unexpected error: {err}");
    }

    /// `"unified"` legitimately pairs with `ipc_flag = 0` (it is not
    /// `"ipc"`, so it does not contradict the flag) and must keep its
    /// normal size check rather than being treated as exempt.
    #[test]
    fn transport_unified_with_no_ipc_flag_parses_and_keeps_its_size_check() {
        let json = metadata_json(8, "uint8", &[8], "cuda", "unified");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, &json).expect("write");
        let parsed = parse_header(&buf).expect("parse");
        assert_eq!(parsed.metadata.transport, "unified");
        assert!(!parsed.ipc_present);

        let oversized = metadata_json(1_000_000, "uint8", &[1_000_000], "cuda", "unified");
        let mut buf2 = vec![0u8; data_offset_for(oversized.len())]; // header-only
        write_header(&mut buf2, &oversized).expect("write");
        let err = parse_header(&buf2).unwrap_err();
        assert!(err.contains("size"), "unexpected error: {err}");
    }

    /// `constants_match_the_python_binding` only pins the offset constants
    /// to literals defined in this same file — an offset typo would
    /// round-trip through `write_header`/`parse_header` undetected. This
    /// asserts the actual bytes `write_header` produces, at literal offsets
    /// independent of the `OFFSET_*` constants it exercises.
    ///
    /// The buffer starts pre-poisoned (`0xEE`, not `0u8`) so a deleted
    /// `.fill(0)` call in `write_header` shows up as a non-zero byte here
    /// instead of being indistinguishable from an already-zeroed `Vec`.
    #[test]
    fn write_header_lays_out_bytes_at_the_documented_offsets() {
        let json = "{}";
        let data_offset = data_offset_for(json.len());
        let mut buf = vec![0xEEu8; data_offset];
        write_header(&mut buf, json).expect("write");

        assert_eq!(&buf[0..8], MAGIC);
        assert_eq!(&buf[8..16], (json.len() as u64).to_le_bytes());
        assert_eq!(&buf[16..24], (data_offset as u64).to_le_bytes());
        assert_eq!(&buf[24..32], 0u64.to_le_bytes(), "ipc_flag");
        assert!(
            buf[32..96].iter().all(|&b| b == 0),
            "ipc handle region not zeroed"
        );
        assert_eq!(&buf[96..104], 0u64.to_le_bytes(), "write_gen");
        assert!(
            buf[104..HEADER_SIZE].iter().all(|&b| b == 0),
            "reserved region not zeroed"
        );
        assert!(
            buf[HEADER_SIZE + json.len()..data_offset]
                .iter()
                .all(|&b| b == 0),
            "json padding not zeroed"
        );
    }
}
