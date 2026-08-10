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
    ///
    /// **No path in this workspace reads this value.** The transport, checked
    /// against the header's `ipc_flag`, is what every consumer here acts on.
    /// It is decoded anyway for two reasons. It is written by
    /// [`metadata_json`], and a parser that dropped a key its own writer emits
    /// would make this a partial definition of the format rather than the
    /// whole one. And decoding is what *type-checks* it: a segment whose
    /// `pinned_type` is not a string is rejected at [`parse_metadata_json`],
    /// on every open, before it can reach the peer that does branch on it —
    /// the Python binding computes `ipc_present == 1 || pinned_type !=
    /// Some("cpu")` and picks its whole read path from the answer.
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

/// Serialize the metadata JSON: the four keys the Python binding writes, plus
/// the additive `transport` key.
///
/// **Not byte-for-byte what Python emits, and it does not need to be.** Python
/// builds this with `json.dumps`, whose default separators are `", "` and
/// `": "`; `serde_json::to_string` is compact. So the same four keys with the
/// same values already produce a different `json_len` here than there — 73
/// bytes against 91 for a 16 KiB `uint8` pool, measured on a live segment of
/// each. That is invisible to every reader because `json_len` and
/// `data_offset` are fields *in the header*, not constants: [`parse_header`]
/// reads them, and [`data_offset_for`] rounds up to [`METADATA_ALIGN`], so
/// two producers only land on the same payload offset when their two lengths
/// round to the same multiple of 256. They do for the pools in
/// `examples/c++-memory-pool`; a longer dtype or a higher-rank shape would
/// separate them, and nothing would break when it did.
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

/// How many bytes of a segment a caller may safely reference before
/// [`parse_header`] has run: exactly the write-once header+metadata region,
/// `HEADER_SIZE + json_len`.
///
/// A `&[u8]` over the payload aliases bytes a peer process may be
/// overwriting, which is UB whether or not anything reads through it. So a
/// caller must bound its slice *before* it can parse anything — which means
/// bounding it against two fields it has not validated yet. Doing that by
/// hand does not work: clamping to the segment length lets a corrupt
/// `json_len` overshoot and clamp right back to the whole mapping, payload
/// included, and even a well-behaved-looking `json_len` (say 4096 against a
/// `data_offset` of 256 in an 8 KiB segment) passes a segment-length test
/// while spanning the payload.
///
/// The only correct cap is `data_offset`, the payload start. This validates
/// the chain `HEADER_SIZE + json_len <= data_offset <= segment_len` and
/// returns `HEADER_SIZE + json_len`, so the result provably cannot reach the
/// payload. Every well-formed segment satisfies that chain by construction —
/// `write_header` derives `data_offset` from `json_len` — so nothing
/// legitimate is rejected here that [`parse_header`] would have accepted.
/// `parse_header` re-checks all of it as the backstop.
pub fn header_region_len(fixed_header: &[u8], segment_len: usize) -> Result<usize, String> {
    if fixed_header.len() < HEADER_SIZE || segment_len < HEADER_SIZE {
        return Err(format!(
            "segment too small: {segment_len} bytes, need at least {HEADER_SIZE}"
        ));
    }
    let json_len = read_u64(fixed_header, OFFSET_JSON_LEN) as usize;
    let data_offset = read_u64(fixed_header, OFFSET_DATA_OFFSET) as usize;
    // Distinct wording from `parse_header`'s equivalent checks: these fire
    // first, on unvalidated bytes, and a test needs to be able to tell which
    // of the two rejected a segment.
    if data_offset > segment_len {
        return Err(format!(
            "cannot bound the header region: data_offset {data_offset} is past the end of the \
             {segment_len}-byte segment"
        ));
    }
    let end = HEADER_SIZE.checked_add(json_len).ok_or_else(|| {
        format!("cannot bound the header region: json_len {json_len} overflows the address space")
    })?;
    if end > data_offset {
        return Err(format!(
            "cannot bound the header region: json_len {json_len} does not fit before data_offset \
             {data_offset}"
        ));
    }
    Ok(end)
}

/// Parse a segment's header.
///
/// `buf` need only cover the write-once header+metadata region (size it with
/// [`header_region_len`]); it must **not** be extended over the payload,
/// which a peer may be writing concurrently. `segment_len` carries the total
/// mapping length separately,
/// because every bound below is against the whole segment while none of them
/// needs to read it.
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
pub fn parse_header(buf: &[u8], segment_len: usize) -> Result<ParsedHeader, String> {
    if segment_len < HEADER_SIZE || buf.len() < HEADER_SIZE {
        return Err(format!(
            "segment too small: {segment_len} bytes, need at least {HEADER_SIZE}"
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
    if data_offset > segment_len {
        return Err(format!(
            "header data_offset {data_offset} is past the end of the {segment_len}-byte segment"
        ));
    }
    if json_len > data_offset.saturating_sub(HEADER_SIZE) {
        return Err(format!(
            "header json_len {json_len} does not fit before data_offset {data_offset}"
        ));
    }
    // The caller sized `buf` from `json_len_field`, so this holds by
    // construction; check it anyway rather than let a wrongly sized slice
    // become a panicking index.
    if buf.len() < HEADER_SIZE + json_len {
        return Err(format!(
            "header slice is {} bytes, too short for the {json_len}-byte metadata json it declares",
            buf.len()
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
        let available = segment_len.saturating_sub(data_offset);
        if metadata.size > available {
            return Err(format!(
                "metadata size {} exceeds the {available}-byte data region available after data_offset {data_offset} in the {segment_len}-byte segment",
                metadata.size,
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

    /// This file's offsets, asserted against themselves. It fixes the layout
    /// against an accidental edit *here*; it says nothing about the Python
    /// binding, which is what [`constants_match_the_python_binding`] is for.
    #[test]
    fn the_header_layout_is_the_one_this_module_documents() {
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

    /// The `const NAME: ... = <value>;` initializer text from `source`, or a
    /// message naming what was searched for.
    ///
    /// Text, not a parsed value: the point is to compare what the other file
    /// literally says, so an edit there — to the number, or to the byte-string
    /// spelling of the magic — is visible here whatever form it takes.
    fn const_initializer(source: &str, name: &str) -> Result<String, String> {
        let needle = format!("const {name}");
        let line = source
            .lines()
            .find(|line| line.trim_start().starts_with(&needle))
            .ok_or_else(|| format!("no `{needle}` declaration"))?;
        let (_, rhs) = line
            .split_once('=')
            .ok_or_else(|| format!("`{needle}` line has no `=`: {line}"))?;
        Ok(rhs.trim().trim_end_matches(';').trim().to_string())
    }

    /// The byte-string literal a Rust source file would have to spell to mean
    /// [`MAGIC`], e.g. `b"DORADMA\x00"`. Derived from `MAGIC` rather than
    /// written out, so changing the constant changes what is demanded of the
    /// Python binding.
    fn magic_as_source_literal() -> String {
        let escaped: String = MAGIC
            .iter()
            .map(|&byte| {
                if byte.is_ascii_graphic() && byte != b'"' && byte != b'\\' {
                    (byte as char).to_string()
                } else {
                    format!("\\x{byte:02x}")
                }
            })
            .collect();
        format!("b\"{escaped}\"")
    }

    /// The Python binding declares this same wire format a second time, by
    /// hand (`DORADMA_HEADER_SIZE` / `DORADMA_MAGIC` /
    /// `DORADMA_METADATA_ALIGN` in `apis/python/node/src/lib.rs`), and this
    /// branch did not remove that copy. So this test reads Python's three
    /// constants out of its source and fails when they drift from ours — a
    /// C++ node and a Python node have to agree byte for byte, and nothing
    /// else notices if they stop.
    ///
    /// **What it does not cover:** the field offsets. Python writes them as
    /// bare literals inside `unsafe` blocks (`shmem_ptr.add(8)`, `.add(16)`,
    /// `.add(96)`) with no names to read, so an offset change there still
    /// passes here. Removing the duplicate definition is the real fix; this
    /// is the cheap guard until then.
    ///
    /// Reading a sibling crate's source makes this test workspace-only. That
    /// is deliberate: a version that skipped when the file was missing would
    /// promise a tripwire and silently not be one, which is the failure being
    /// corrected here. It fails loudly instead, naming the path.
    #[test]
    fn constants_match_the_python_binding() {
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../../apis/python/node/src/lib.rs");
        let source = std::fs::read_to_string(&path).unwrap_or_else(|err| {
            panic!(
                "cannot cross-check the DORADMA constants against the Python binding at {}: {err}",
                path.display()
            )
        });

        for (python_name, expected) in [
            ("DORADMA_HEADER_SIZE", HEADER_SIZE.to_string()),
            ("DORADMA_METADATA_ALIGN", METADATA_ALIGN.to_string()),
            ("DORADMA_MAGIC", magic_as_source_literal()),
        ] {
            let found = const_initializer(&source, python_name)
                .unwrap_or_else(|err| panic!("{}: {err}", path.display()));
            assert_eq!(
                found,
                expected,
                "`{python_name}` in {} no longer matches this crate's definition of the \
                 DORADMA format",
                path.display()
            );
        }
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

        let parsed = parse_header(&buf, buf.len()).expect("parse");
        assert_eq!(parsed.json_len, json.len());
        assert_eq!(parsed.data_offset, data_offset_for(json.len()));
        assert!(!parsed.ipc_present);
        assert_eq!(parsed.write_gen, 0);
        assert_eq!(parsed.metadata.size, 64);
        assert_eq!(parsed.metadata.dtype, "uint8");
    }

    /// `buf` covers only the write-once header+metadata region while
    /// `segment_len` describes the whole mapping. This is how `PoolSegment`
    /// calls it: forming a `&[u8]` over the payload would alias bytes a peer
    /// process is concurrently writing.
    #[test]
    fn parses_from_a_slice_that_stops_before_the_payload() {
        let json = metadata_json(64, "uint8", &[64], "cpu", "shmem");
        let data_offset = data_offset_for(json.len());
        let mut whole = vec![0u8; data_offset + 64];
        write_header(&mut whole, &json).expect("write");

        let region_len = header_region_len(&whole, data_offset + 64).expect("bound");
        let parsed = parse_header(&whole[..region_len], data_offset + 64).expect("parse");

        assert_eq!(parsed.metadata.size, 64);
        assert_eq!(parsed.data_offset, data_offset);
        assert!(
            region_len <= data_offset,
            "the slice must not reach the payload"
        );
    }

    /// The bound must be `data_offset`, not the segment length. Both cases
    /// here clamp harmlessly against `segment_len` and still span the payload.
    #[test]
    fn header_region_len_caps_at_data_offset_not_at_the_segment_length() {
        let json = metadata_json(64, "uint8", &[64], "cpu", "shmem");
        let data_offset = data_offset_for(json.len());
        let segment_len = data_offset + 8192;
        let mut buf = vec![0u8; HEADER_SIZE];
        write_header(&mut vec![0u8; data_offset + 64], &json).expect("scratch");
        // Rebuild just the fixed header so the two fields can be corrupted
        // independently of what `write_header` would derive.
        let mut whole = vec![0u8; data_offset + 8192];
        write_header(&mut whole, &json).expect("write");
        buf.copy_from_slice(&whole[..HEADER_SIZE]);

        // A json_len that is plausible against the segment but not against
        // data_offset: 256 + 4096 = 4352 <= segment_len, yet way past the
        // payload start.
        buf[OFFSET_JSON_LEN..OFFSET_JSON_LEN + 8].copy_from_slice(&4096u64.to_le_bytes());
        buf[OFFSET_DATA_OFFSET..OFFSET_DATA_OFFSET + 8]
            .copy_from_slice(&(HEADER_SIZE as u64).to_le_bytes());
        let err = header_region_len(&buf, segment_len).unwrap_err();
        assert!(
            err.contains("cannot bound the header region"),
            "unexpected error: {err}"
        );

        // A json_len large enough to overflow when added to HEADER_SIZE.
        buf[OFFSET_JSON_LEN..OFFSET_JSON_LEN + 8].copy_from_slice(&u64::MAX.to_le_bytes());
        let err = header_region_len(&buf, segment_len).unwrap_err();
        assert!(err.contains("overflows"), "unexpected error: {err}");

        // A data_offset past the end of the segment.
        buf[OFFSET_JSON_LEN..OFFSET_JSON_LEN + 8].copy_from_slice(&0u64.to_le_bytes());
        buf[OFFSET_DATA_OFFSET..OFFSET_DATA_OFFSET + 8].copy_from_slice(&u64::MAX.to_le_bytes());
        let err = header_region_len(&buf, segment_len).unwrap_err();
        assert!(err.contains("past the end"), "unexpected error: {err}");
    }

    /// The whole point of the bound: a well-formed segment must still parse,
    /// and the region it yields must stop at or before the payload.
    #[test]
    fn header_region_len_accepts_every_well_formed_segment() {
        for payload in [1usize, 64, 4096] {
            for shape_len in [0usize, 1, 40] {
                let shape: Vec<usize> = (0..shape_len).map(|_| 1).collect();
                let json = metadata_json(payload, "uint8", &shape, "cpu", "shmem");
                let data_offset = data_offset_for(json.len());
                let mut whole = vec![0u8; data_offset + payload];
                write_header(&mut whole, &json).expect("write");

                let region_len =
                    header_region_len(&whole, data_offset + payload).expect("well-formed");
                assert!(region_len <= data_offset);
                parse_header(&whole[..region_len], data_offset + payload).expect("parse");
            }
        }
    }

    /// The size check bounds against `segment_len`, not against the slice —
    /// otherwise every caller passing a header-sized slice would look like a
    /// header-only segment and reject its own payload.
    #[test]
    fn the_size_check_uses_segment_len_not_the_slice_length() {
        let json = metadata_json(1_000_000, "uint8", &[1_000_000], "cpu", "shmem");
        let data_offset = data_offset_for(json.len());
        let mut buf = vec![0u8; data_offset];
        write_header(&mut buf, &json).expect("write");

        // Slice stops at the header, but the real segment holds the payload.
        parse_header(&buf, data_offset + 1_000_000).expect("must accept");
        // Same slice, and now the segment really is header-only.
        let err = parse_header(&buf, data_offset).unwrap_err();
        assert!(err.contains("size"), "unexpected error: {err}");
    }

    #[test]
    fn parse_rejects_bad_magic() {
        let json = metadata_json(8, "uint8", &[8], "cpu", "shmem");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, &json).expect("write");
        buf[0] = b'X';
        let err = parse_header(&buf, buf.len()).unwrap_err();
        assert!(err.contains("magic"), "unexpected error: {err}");
    }

    #[test]
    fn parse_rejects_truncated_segment() {
        let buf = vec![0u8; HEADER_SIZE - 1];
        let err = parse_header(&buf, buf.len()).unwrap_err();
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
        let err = parse_header(&buf, buf.len()).unwrap_err();
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

        let parsed = parse_header(&buf, buf.len()).expect("parse");
        assert_eq!(parsed.metadata.transport, "ipc");
    }

    /// Same absent key, but `ipc_flag = 0` (a plain CPU shmem pool): must
    /// resolve to `"shmem"`.
    #[test]
    fn absent_transport_without_ipc_flag_resolves_to_shmem() {
        let json = r#"{"size":8,"dtype":"uint8","shape":[8],"pinned_type":"cpu"}"#;
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, json).expect("write");

        let parsed = parse_header(&buf, buf.len()).expect("parse");
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
        let err = parse_header(&buf, buf.len()).unwrap_err();
        assert!(err.contains("utf-8"), "unexpected error: {err}");
    }

    #[test]
    fn parse_rejects_data_offset_before_the_header_ends() {
        let json = metadata_json(8, "uint8", &[8], "cpu", "shmem");
        let mut buf = vec![0u8; data_offset_for(json.len()) + 8];
        write_header(&mut buf, &json).expect("write");
        buf[OFFSET_DATA_OFFSET..OFFSET_DATA_OFFSET + 8].copy_from_slice(&0u64.to_le_bytes());
        let err = parse_header(&buf, buf.len()).unwrap_err();
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

        let err = parse_header(&buf, buf.len()).unwrap_err();
        assert!(err.contains("size"), "unexpected error: {err}");

        buf[OFFSET_IPC_FLAG..OFFSET_IPC_FLAG + 8].copy_from_slice(&1u64.to_le_bytes());
        let parsed =
            parse_header(&buf, buf.len()).expect("ipc-present header-only segment must parse");
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

        let err = parse_header(&buf, buf.len()).unwrap_err();
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

        let err = parse_header(&buf, buf.len()).unwrap_err();
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
        let parsed = parse_header(&buf, buf.len()).expect("parse");
        assert_eq!(parsed.metadata.transport, "unified");
        assert!(!parsed.ipc_present);

        let oversized = metadata_json(1_000_000, "uint8", &[1_000_000], "cuda", "unified");
        let mut buf2 = vec![0u8; data_offset_for(oversized.len())]; // header-only
        write_header(&mut buf2, &oversized).expect("write");
        let err = parse_header(&buf2, buf2.len()).unwrap_err();
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
