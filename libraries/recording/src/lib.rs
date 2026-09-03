//! **Internal to dora — not a public API.**
//!
//! This crate is published to crates.io only because cargo requires every
//! dependency of a published crate to be published; `dora-node-api` and
//! `dora-cli` depend on it. It is not covered by dora's 1.0 stability
//! guarantee and may change in any release, including a patch.
//!
//! Depend on it directly at your own risk. See the "Stability scope at 1.0"
//! section of `docs/api-rust.md`.
//!
//! Binary `.drec` recording format for dora dataflow message capture and replay.
//!
//! A recording is a [`RecordingHeader`], a sequence of [`RecordEntry`] records,
//! and an optional [`RecordingFooter`]. Capture a stream with
//! [`RecordingWriter`] and read it back with [`RecordingReader`]. Both are
//! generic over any [`std::io::Write`] / [`std::io::Read`], so they work with
//! files as well as in-memory buffers.
//!
//! ```
//! use dora_recording::{FORMAT_VERSION, RecordEntry, RecordingHeader, RecordingReader, RecordingWriter};
//! use std::io::Cursor;
//!
//! # fn main() -> eyre::Result<()> {
//! let header = RecordingHeader {
//!     version: FORMAT_VERSION,
//!     start_nanos: 0,
//!     dataflow_id: uuid::Uuid::nil(),
//!     descriptor_yaml: b"nodes: []".to_vec(),
//! };
//!
//! // Write one entry into an in-memory buffer.
//! let mut buf: Vec<u8> = Vec::new();
//! {
//!     let mut writer = RecordingWriter::new(&mut buf, &header)?;
//!     writer.write_entry(&RecordEntry {
//!         node_id: "camera".to_string(),
//!         output_id: "image".to_string(),
//!         timestamp_offset_nanos: 0,
//!         event_bytes: vec![1, 2, 3],
//!     })?;
//!     let footer = writer.finish()?;
//!     assert_eq!(footer.total_messages, 1);
//! }
//!
//! // Read it back.
//! let mut reader = RecordingReader::open(Cursor::new(buf))?;
//! assert_eq!(reader.header().dataflow_id, uuid::Uuid::nil());
//! let entry = reader.next_entry()?.expect("one entry");
//! assert_eq!(entry.node_id, "camera");
//! assert_eq!(entry.event_bytes, vec![1, 2, 3]);
//! assert!(reader.next_entry()?.is_none());
//! # Ok(())
//! # }
//! ```

use std::io::{self, BufReader, BufWriter, Read, Write};

use eyre::Context;
use uuid::Uuid;

const MAGIC: &[u8; 8] = b"DORAREC\x00";
const FOOTER_MAGIC: &[u8; 8] = b"DORAEND\x00";
/// Version stamped into the header of newly written `.drec` files.
///
/// Bumped from 1 to 2 when `event_bytes` moved from bincode to postcard: the
/// container framing is unchanged, but every entry's payload is a
/// `Timestamped<InterDaemonEvent>` in the new encoding.
///
/// Writers must stamp [`RecordingHeader::version`] with this rather than a
/// literal, or they produce files their own reader rejects.
pub const FORMAT_VERSION: u16 = 2;
/// Oldest `.drec` version this build can read.
///
/// The entry payloads are opaque to the container, so a version that only the
/// *payload* encoding changed still has to be rejected here — otherwise the
/// header check passes and every entry fails to decode further downstream,
/// surfacing as "corrupt or format-drifted recording" instead of a clear
/// version error naming the dora release that wrote the file.
const MIN_SUPPORTED_FORMAT_VERSION: u16 = 2;
/// A bump that leaves the writers stamping a version below the reader's floor
/// would make every freshly written recording unreadable by the same build.
const _: () = assert!(FORMAT_VERSION >= MIN_SUPPORTED_FORMAT_VERSION);
/// Maximum size for a single record or YAML descriptor in a `.drec` file.
/// Guards against OOM from crafted files with `u32::MAX` length fields.
const MAX_RECORD_BYTES: usize = 64 * 1024 * 1024; // 64 MB

/// Header written at the start of a `.drec` file.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordingHeader {
    pub version: u16,
    pub start_nanos: u64,
    pub dataflow_id: Uuid,
    pub descriptor_yaml: Vec<u8>,
}

/// A single recorded message entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordEntry {
    pub node_id: String,
    pub output_id: String,
    pub timestamp_offset_nanos: u64,
    pub event_bytes: Vec<u8>,
}

impl RecordEntry {
    /// Assemble an owned entry from the borrowed fields produced by
    /// `parse_record`, copying the ids and payload out of the record buffer.
    fn owned(
        node_id: &str,
        output_id: &str,
        timestamp_offset_nanos: u64,
        event_bytes: &[u8],
    ) -> Self {
        Self {
            node_id: node_id.to_string(),
            output_id: output_id.to_string(),
            timestamp_offset_nanos,
            event_bytes: event_bytes.to_vec(),
        }
    }
}

/// A recorded entry's metadata without its event payload, produced by
/// [`RecordingReader::next_entry_header`] for callers that need the ids or
/// timing but not the (potentially large) payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RecordEntryHeader {
    pub node_id: String,
    pub output_id: String,
    pub timestamp_offset_nanos: u64,
}

/// Optional footer written at end of recording.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecordingFooter {
    pub total_messages: u64,
    pub total_bytes: u64,
}

/// Streams recording entries to disk.
pub struct RecordingWriter<W: Write> {
    writer: BufWriter<W>,
    total_messages: u64,
    total_bytes: u64,
}

impl<W: Write> RecordingWriter<W> {
    /// Create a writer over `inner`, immediately writing the recording header.
    pub fn new(inner: W, header: &RecordingHeader) -> eyre::Result<Self> {
        let mut writer = BufWriter::new(inner);
        write_header(&mut writer, header)?;
        Ok(Self {
            writer,
            total_messages: 0,
            total_bytes: 0,
        })
    }

    /// Append a single message entry to the recording.
    pub fn write_entry(&mut self, entry: &RecordEntry) -> eyre::Result<()> {
        let node_id_bytes = entry.node_id.as_bytes();
        let output_id_bytes = entry.output_id.as_bytes();
        // Compute as usize first to avoid u32 truncation before the cap check.
        let record_len_usize =
            2 + node_id_bytes.len() + 2 + output_id_bytes.len() + 8 + 4 + entry.event_bytes.len();
        if record_len_usize > MAX_RECORD_BYTES {
            eyre::bail!(
                "record too large to write: {record_len_usize} bytes (max {MAX_RECORD_BYTES})"
            );
        }
        let record_len = u32::try_from(record_len_usize)
            .map_err(|_| eyre::eyre!("record length {record_len_usize} exceeds u32::MAX"))?;

        let node_len = u16::try_from(node_id_bytes.len()).map_err(|_| {
            eyre::eyre!(
                "node_id too long: {} bytes (max {})",
                node_id_bytes.len(),
                u16::MAX
            )
        })?;
        let output_len = u16::try_from(output_id_bytes.len()).map_err(|_| {
            eyre::eyre!(
                "output_id too long: {} bytes (max {})",
                output_id_bytes.len(),
                u16::MAX
            )
        })?;

        self.writer.write_all(&record_len.to_le_bytes())?;
        self.writer.write_all(&node_len.to_le_bytes())?;
        self.writer.write_all(node_id_bytes)?;
        self.writer.write_all(&output_len.to_le_bytes())?;
        self.writer.write_all(output_id_bytes)?;
        self.writer
            .write_all(&entry.timestamp_offset_nanos.to_le_bytes())?;
        self.writer
            .write_all(&(entry.event_bytes.len() as u32).to_le_bytes())?;
        self.writer.write_all(&entry.event_bytes)?;

        self.total_messages += 1;
        self.total_bytes += record_len as u64 + 4; // +4 for record_len field itself
        Ok(())
    }

    /// Write the footer (message/byte totals), flush, and consume the writer.
    pub fn finish(mut self) -> eyre::Result<RecordingFooter> {
        let footer = RecordingFooter {
            total_messages: self.total_messages,
            total_bytes: self.total_bytes,
        };
        self.writer.write_all(FOOTER_MAGIC)?;
        self.writer
            .write_all(&footer.total_messages.to_le_bytes())?;
        self.writer.write_all(&footer.total_bytes.to_le_bytes())?;
        self.writer.flush()?;
        Ok(footer)
    }

    /// Flush buffered bytes to the underlying writer without finishing.
    pub fn flush(&mut self) -> eyre::Result<()> {
        self.writer.flush().wrap_err("flush failed")
    }
}

/// Reads recording entries from disk.
#[derive(Debug)]
pub struct RecordingReader<R: Read> {
    reader: BufReader<R>,
    header: RecordingHeader,
}

impl<R: Read> RecordingReader<R> {
    /// Open a reader over `inner`, immediately parsing and validating the header.
    pub fn open(inner: R) -> eyre::Result<Self> {
        let mut reader = BufReader::new(inner);
        let header = read_header(&mut reader)?;
        Ok(Self { reader, header })
    }

    /// The recording header parsed by [`open`](Self::open).
    pub fn header(&self) -> &RecordingHeader {
        &self.header
    }

    /// Read the next entry. Returns `None` at EOF or footer.
    pub fn next_entry(&mut self) -> eyre::Result<Option<RecordEntry>> {
        let Some(record_buf) = self.read_next_record()? else {
            return Ok(None);
        };
        let (node_id, output_id, timestamp_offset_nanos, event_bytes) = parse_record(&record_buf)?;
        Ok(Some(RecordEntry::owned(
            node_id,
            output_id,
            timestamp_offset_nanos,
            event_bytes,
        )))
    }

    /// Read the next entry's metadata, *without* copying its event payload out
    /// of the record buffer. Returns `None` at EOF or footer.
    ///
    /// Every length field is still bounds-checked and both ids are still
    /// validated as UTF-8, so a corrupt record is rejected exactly as by
    /// [`next_entry`](Self::next_entry); only the payload `.to_vec()` is
    /// skipped. Use this when the caller needs the entry's ids or timing but
    /// not the payload — e.g. counting how many messages each output recorded.
    pub fn next_entry_header(&mut self) -> eyre::Result<Option<RecordEntryHeader>> {
        let Some(record_buf) = self.read_next_record()? else {
            return Ok(None);
        };
        let (node_id, output_id, timestamp_offset_nanos, _event_bytes) = parse_record(&record_buf)?;
        Ok(Some(RecordEntryHeader {
            node_id: node_id.to_string(),
            output_id: output_id.to_string(),
            timestamp_offset_nanos,
        }))
    }

    /// Read the next entry produced by `node`, skipping entries from other
    /// nodes. Returns `None` at EOF or footer.
    ///
    /// Foreign entries are still fully read from the stream and validated (so
    /// corruption is rejected exactly as by [`next_entry`](Self::next_entry)),
    /// but nothing is copied out of the record buffer for them — neither the
    /// (potentially multi-megabyte) event payload nor the ids. `dora replay`
    /// spawns one replay process per node, each scanning the whole recording
    /// and keeping only its own node's entries; using
    /// [`next_entry`](Self::next_entry) there makes every process heap-copy
    /// every *other* node's payloads just to discard them.
    pub fn next_entry_for_node(&mut self, node: &str) -> eyre::Result<Option<RecordEntry>> {
        loop {
            let Some(record_buf) = self.read_next_record()? else {
                return Ok(None);
            };
            let (node_id, output_id, timestamp_offset_nanos, event_bytes) =
                parse_record(&record_buf)?;
            if node_id != node {
                // Validated above; skip without allocating the ids or copying
                // the payload (all borrowed from `record_buf`).
                continue;
            }
            return Ok(Some(RecordEntry::owned(
                node_id,
                output_id,
                timestamp_offset_nanos,
                event_bytes,
            )));
        }
    }

    /// Read the next record body from the stream, or `None` at EOF/footer.
    /// Shared framing for [`next_entry`](Self::next_entry),
    /// [`next_entry_header`](Self::next_entry_header), and
    /// [`next_entry_for_node`](Self::next_entry_for_node).
    fn read_next_record(&mut self) -> eyre::Result<Option<Vec<u8>>> {
        let mut len_buf = [0u8; 4];
        match self.reader.read_exact(&mut len_buf) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e).wrap_err("failed to read record length"),
        }

        // Check if this is a footer marker instead of a record
        if len_buf == FOOTER_MAGIC[..4] {
            let mut rest = [0u8; 4];
            match self.reader.read_exact(&mut rest) {
                Ok(()) if rest == FOOTER_MAGIC[4..] => return Ok(None),
                _ => {}
            }
            // Not actually footer magic, treat as corrupted -- stop reading
            return Ok(None);
        }

        let record_len = u32::from_le_bytes(len_buf) as usize;
        if record_len > MAX_RECORD_BYTES {
            eyre::bail!("record too large: {record_len} bytes (max {MAX_RECORD_BYTES})");
        }
        let mut record_buf = vec![0u8; record_len];
        match self.reader.read_exact(&mut record_buf) {
            Ok(()) => {}
            // A crash (SIGKILL / Ctrl-C) can flush a record's 4-byte length
            // prefix but only part of the record body that follows. Treat such
            // a torn trailing record the same way as a torn length prefix
            // (handled above): stop gracefully at EOF so every fully-written
            // record still replays, rather than failing the whole replay with
            // `truncated record`. Genuine intra-record corruption in a
            // *complete* record is still rejected by the bounds checks in
            // `read_array` / `read_slice` (see `parse_record`).
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => return Ok(None),
            Err(e) => return Err(e).wrap_err("failed to read record"),
        }

        Ok(Some(record_buf))
    }
}

/// Parse a record body into its `(node_id, output_id, timestamp_offset_nanos,
/// event_bytes)` fields, all **borrowed** from `record_buf`. Callers copy out
/// only the fields they keep — [`RecordingReader::next_entry_for_node`] compares
/// the borrowed node id and skips foreign records without allocating anything.
/// Every length field is bounds-checked via [`read_array`] / [`read_slice`], so
/// a corrupt or crafted record fails gracefully rather than panicking.
fn parse_record(record_buf: &[u8]) -> eyre::Result<(&str, &str, u64, &[u8])> {
    let mut pos = 0;

    let node_id_len = u16::from_le_bytes(read_array(record_buf, &mut pos)?) as usize;
    let node_id = std::str::from_utf8(read_slice(record_buf, &mut pos, node_id_len)?)
        .wrap_err("invalid node_id utf8")?;

    let output_id_len = u16::from_le_bytes(read_array(record_buf, &mut pos)?) as usize;
    let output_id = std::str::from_utf8(read_slice(record_buf, &mut pos, output_id_len)?)
        .wrap_err("invalid output_id utf8")?;

    let timestamp_offset_nanos = u64::from_le_bytes(read_array(record_buf, &mut pos)?);
    let event_bytes_len = u32::from_le_bytes(read_array(record_buf, &mut pos)?) as usize;
    let event_bytes = read_slice(record_buf, &mut pos, event_bytes_len)?;

    Ok((node_id, output_id, timestamp_offset_nanos, event_bytes))
}

fn read_array<const N: usize>(buf: &[u8], pos: &mut usize) -> eyre::Result<[u8; N]> {
    if *pos + N > buf.len() {
        eyre::bail!("buffer too short at offset {pos}");
    }
    let mut arr = [0u8; N];
    arr.copy_from_slice(&buf[*pos..*pos + N]);
    *pos += N;
    Ok(arr)
}

/// Borrow `len` bytes from `buf` at `*pos`, advancing `*pos`. Returns `Err`
/// (never panics) when a decoded length field claims more bytes than the
/// record actually holds -- corrupt/crafted records must fail gracefully, not
/// out-of-bounds slice (`checked_add` also guards against `pos + len` wrapping).
fn read_slice<'a>(buf: &'a [u8], pos: &mut usize, len: usize) -> eyre::Result<&'a [u8]> {
    let end = pos
        .checked_add(len)
        .ok_or_else(|| eyre::eyre!("length {len} at offset {pos} overflows"))?;
    if end > buf.len() {
        eyre::bail!(
            "buffer too short at offset {pos}: need {len} bytes, have {}",
            buf.len() - *pos
        );
    }
    let slice = &buf[*pos..end];
    *pos = end;
    Ok(slice)
}

fn write_header<W: Write>(w: &mut W, header: &RecordingHeader) -> eyre::Result<()> {
    if header.descriptor_yaml.len() > MAX_RECORD_BYTES {
        eyre::bail!(
            "descriptor YAML too large to write: {} bytes (max {MAX_RECORD_BYTES})",
            header.descriptor_yaml.len()
        );
    }
    w.write_all(MAGIC)?;
    w.write_all(&header.version.to_le_bytes())?;
    w.write_all(&header.start_nanos.to_le_bytes())?;
    w.write_all(header.dataflow_id.as_bytes())?;
    w.write_all(&(header.descriptor_yaml.len() as u32).to_le_bytes())?;
    w.write_all(&header.descriptor_yaml)?;
    Ok(())
}

fn read_header<R: Read>(r: &mut R) -> eyre::Result<RecordingHeader> {
    let mut magic = [0u8; 8];
    r.read_exact(&mut magic)
        .wrap_err("failed to read magic bytes")?;
    if &magic != MAGIC {
        eyre::bail!("not an dora recording file (invalid magic)");
    }

    let mut version_buf = [0u8; 2];
    r.read_exact(&mut version_buf)?;
    let version = u16::from_le_bytes(version_buf);
    if version > FORMAT_VERSION {
        eyre::bail!(
            "unsupported recording format version {version} (max supported: {FORMAT_VERSION})"
        );
    }
    if version < MIN_SUPPORTED_FORMAT_VERSION {
        eyre::bail!(
            "recording format version {version} is no longer supported (min supported: \
             {MIN_SUPPORTED_FORMAT_VERSION}); it was written by a dora release that encoded \
             events with bincode. Re-record with this version of dora."
        );
    }

    let mut nanos_buf = [0u8; 8];
    r.read_exact(&mut nanos_buf)?;
    let start_nanos = u64::from_le_bytes(nanos_buf);

    let mut uuid_buf = [0u8; 16];
    r.read_exact(&mut uuid_buf)?;
    let dataflow_id = Uuid::from_bytes(uuid_buf);

    let mut yaml_len_buf = [0u8; 4];
    r.read_exact(&mut yaml_len_buf)?;
    let yaml_len = u32::from_le_bytes(yaml_len_buf) as usize;
    if yaml_len > MAX_RECORD_BYTES {
        eyre::bail!("descriptor YAML too large: {yaml_len} bytes (max {MAX_RECORD_BYTES})");
    }

    let mut descriptor_yaml = vec![0u8; yaml_len];
    r.read_exact(&mut descriptor_yaml)?;

    Ok(RecordingHeader {
        version,
        start_nanos,
        dataflow_id,
        descriptor_yaml,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_header() -> RecordingHeader {
        RecordingHeader {
            version: FORMAT_VERSION,
            start_nanos: 1_000_000_000,
            dataflow_id: Uuid::nil(),
            descriptor_yaml: b"nodes: []".to_vec(),
        }
    }

    fn sample_entry(node: &str, output: &str, offset: u64, data: &[u8]) -> RecordEntry {
        RecordEntry {
            node_id: node.to_string(),
            output_id: output.to_string(),
            timestamp_offset_nanos: offset,
            event_bytes: data.to_vec(),
        }
    }

    #[test]
    fn header_roundtrip() {
        let header = sample_header();
        let mut buf = Vec::new();
        write_header(&mut buf, &header).unwrap();

        let mut cursor = std::io::Cursor::new(&buf);
        let read_back = read_header(&mut cursor).unwrap();
        assert_eq!(header, read_back);
    }

    /// A v1 `.drec` holds bincode-encoded `event_bytes`, which this build
    /// cannot decode. The container framing is identical, so nothing downstream
    /// would notice until every entry failed to deserialize and got skipped as
    /// "corrupt" — the reader must reject the file up front and say why.
    #[test]
    fn pre_postcard_recordings_are_rejected_with_a_version_error() {
        let mut buf = Vec::new();
        write_header(
            &mut buf,
            &RecordingHeader {
                version: 1,
                ..sample_header()
            },
        )
        .unwrap();

        let err = read_header(&mut std::io::Cursor::new(&buf))
            .expect_err("a bincode-era recording must be rejected");
        let msg = format!("{err:#}");
        assert!(
            msg.contains("no longer supported"),
            "error must name the version problem, got: {msg}"
        );
    }

    #[test]
    fn single_record_roundtrip() {
        let header = sample_header();
        let entry = sample_entry("sensor", "image", 42, b"hello world");

        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
        writer.write_entry(&entry).unwrap();
        let footer = writer.finish().unwrap();

        assert_eq!(footer.total_messages, 1);

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        assert_eq!(reader.header(), &header);

        let read_entry = reader.next_entry().unwrap().unwrap();
        assert_eq!(entry, read_entry);

        assert!(reader.next_entry().unwrap().is_none());
    }

    #[test]
    fn multi_record_with_footer() {
        let header = sample_header();
        let entries = vec![
            sample_entry("cam", "frame", 0, b"\x01\x02\x03"),
            sample_entry("lidar", "points", 100_000, b"\x04\x05"),
            sample_entry("cam", "frame", 200_000, b"\x06"),
        ];

        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
        for e in &entries {
            writer.write_entry(e).unwrap();
        }
        let footer = writer.finish().unwrap();
        assert_eq!(footer.total_messages, 3);

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        for expected in &entries {
            let actual = reader.next_entry().unwrap().unwrap();
            assert_eq!(expected, &actual);
        }
        assert!(reader.next_entry().unwrap().is_none());
    }

    #[test]
    fn truncated_file_no_footer() {
        let header = sample_header();
        let entry = sample_entry("node1", "out1", 10, b"data");

        let buf = {
            let mut buf = Vec::new();
            let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
            writer.write_entry(&entry).unwrap();
            // Don't call finish() -- simulates Ctrl-C
            drop(writer);
            buf
        };

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        let read_entry = reader.next_entry().unwrap().unwrap();
        assert_eq!(entry, read_entry);
        // Should gracefully return None at EOF
        assert!(reader.next_entry().unwrap().is_none());
    }

    /// A crash can flush a record's length prefix but only part of its body.
    /// Replay must return every fully-written record and then stop gracefully
    /// at the torn tail, rather than failing the whole replay with an error.
    #[test]
    fn truncated_record_body_stops_gracefully() {
        let header = sample_header();
        let entry = sample_entry("node1", "out1", 10, b"data");

        let mut buf = Vec::new();
        {
            let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
            writer.write_entry(&entry).unwrap();
            writer.flush().unwrap();
        }
        // Append a second record's length prefix but only part of its body,
        // simulating a crash mid-write after the prefix was flushed.
        buf.extend_from_slice(&64u32.to_le_bytes());
        buf.extend_from_slice(&[0u8; 10]);

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        // The first, complete record still replays.
        assert_eq!(entry, reader.next_entry().unwrap().unwrap());
        // The torn trailing record is discarded gracefully (no error).
        assert!(reader.next_entry().unwrap().is_none());
    }

    /// A crafted `node_id_len` larger than the record buffer must return `Err`,
    /// not panic on an out-of-bounds slice. Reachable from a user-supplied
    /// `DORA_REPLAY_FILE` (dora-rs/dora#2027).
    #[test]
    fn corrupt_node_id_len_returns_err_not_panic() {
        let header = sample_header();
        let mut buf = Vec::new();
        write_header(&mut buf, &header).unwrap();
        // Record body claims node_id_len = 100 but only the 2 length bytes follow.
        let body: &[u8] = &100u16.to_le_bytes();
        buf.extend_from_slice(&(body.len() as u32).to_le_bytes());
        buf.extend_from_slice(body);

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        let result = reader.next_entry();
        assert!(
            result.is_err(),
            "corrupt node_id_len must return Err, got: {result:?}"
        );
    }

    /// Same guarantee for a crafted `event_bytes_len` (a `u32`, so it can claim
    /// far more than the buffer holds).
    #[test]
    fn corrupt_event_bytes_len_returns_err_not_panic() {
        let header = sample_header();
        let mut buf = Vec::new();
        write_header(&mut buf, &header).unwrap();
        // Valid empty node_id + output_id + timestamp, then a huge event_bytes_len
        // with no payload following it.
        let mut body = Vec::new();
        body.extend_from_slice(&0u16.to_le_bytes()); // node_id_len
        body.extend_from_slice(&0u16.to_le_bytes()); // output_id_len
        body.extend_from_slice(&0u64.to_le_bytes()); // timestamp
        body.extend_from_slice(&u32::MAX.to_le_bytes()); // event_bytes_len
        buf.extend_from_slice(&(body.len() as u32).to_le_bytes());
        buf.extend_from_slice(&body);

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        let result = reader.next_entry();
        assert!(
            result.is_err(),
            "corrupt event_bytes_len must return Err, got: {result:?}"
        );
    }

    #[test]
    fn raw_bytes_survive_roundtrip() {
        let header = sample_header();
        // Binary payload with all byte values
        let raw: Vec<u8> = (0..=255).collect();
        let entry = sample_entry("node", "output", 0, &raw);

        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
        writer.write_entry(&entry).unwrap();
        writer.finish().unwrap();

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        let read = reader.next_entry().unwrap().unwrap();
        assert_eq!(raw, read.event_bytes);
    }

    #[test]
    fn invalid_magic_rejected() {
        let buf = b"NOT_DORA_RECORDING";
        let result = RecordingReader::open(std::io::Cursor::new(buf));
        assert!(result.is_err());
        let msg = format!("{}", result.unwrap_err());
        assert!(msg.contains("invalid magic"), "got: {msg}");
    }

    #[test]
    fn empty_descriptor_yaml() {
        let header = RecordingHeader {
            version: FORMAT_VERSION,
            start_nanos: 0,
            dataflow_id: Uuid::nil(),
            descriptor_yaml: vec![],
        };
        let buf = {
            let mut buf = Vec::new();
            let writer = RecordingWriter::new(&mut buf, &header).unwrap();
            writer.finish().unwrap();
            buf
        };

        let reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        assert!(reader.header().descriptor_yaml.is_empty());
    }

    #[test]
    fn writer_rejects_oversized_record() {
        let header = sample_header();
        let big_entry = sample_entry("n", "o", 0, &vec![0u8; MAX_RECORD_BYTES + 1]);
        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
        let err = writer.write_entry(&big_entry).unwrap_err();
        assert!(
            err.to_string().contains("too large"),
            "expected 'too large' error, got: {err}"
        );
    }

    #[test]
    fn writer_rejects_oversized_descriptor_yaml() {
        let header = RecordingHeader {
            version: FORMAT_VERSION,
            start_nanos: 0,
            dataflow_id: Uuid::nil(),
            descriptor_yaml: vec![0u8; MAX_RECORD_BYTES + 1],
        };
        let mut buf = Vec::new();
        let result = RecordingWriter::new(&mut buf, &header);
        assert!(result.is_err());
        let err = result.err().unwrap();
        assert!(
            err.to_string().contains("too large"),
            "expected 'too large' error, got: {err}"
        );
    }

    /// `node_id` longer than `u16::MAX` bytes must return `Err`, not silently
    /// truncate the length field and write a corrupt entry.
    #[test]
    fn writer_rejects_oversized_node_id() {
        let header = sample_header();
        let long_id = "x".repeat(usize::from(u16::MAX) + 1);
        let entry = sample_entry(&long_id, "out", 0, b"data");
        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
        let err = writer.write_entry(&entry).unwrap_err();
        assert!(
            err.to_string().contains("node_id too long"),
            "expected 'node_id too long' error, got: {err}"
        );
    }

    /// `output_id` longer than `u16::MAX` bytes must return `Err`, not silently
    /// truncate the length field and write a corrupt entry.
    #[test]
    fn writer_rejects_oversized_output_id() {
        let header = sample_header();
        let long_id = "y".repeat(usize::from(u16::MAX) + 1);
        let entry = sample_entry("node", &long_id, 0, b"data");
        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
        let err = writer.write_entry(&entry).unwrap_err();
        assert!(
            err.to_string().contains("output_id too long"),
            "expected 'output_id too long' error, got: {err}"
        );
    }

    #[test]
    fn reader_rejects_oversized_record_len() {
        // Craft a file with a valid header followed by a record_len > MAX_RECORD_BYTES.
        let header = sample_header();
        let mut buf = Vec::new();
        write_header(&mut buf, &header).unwrap();
        // Write a fake record_len that exceeds the cap.
        let fake_len: u32 = (MAX_RECORD_BYTES as u32) + 1;
        buf.extend_from_slice(&fake_len.to_le_bytes());
        buf.extend_from_slice(&vec![0u8; fake_len as usize]); // pad so read_exact doesn't fail first

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        let err = reader.next_entry().unwrap_err();
        assert!(
            err.to_string().contains("too large"),
            "expected 'too large' error, got: {err}"
        );
    }

    /// `next_entry_header` yields the same ids/timing as `next_entry`, in order,
    /// without materializing the payload — the counting pass in `dora replay`
    /// relies on this.
    #[test]
    fn next_entry_header_yields_ids_without_payload() {
        let header = sample_header();
        let entries = vec![
            sample_entry("cam", "frame", 0, b"\x01\x02\x03"),
            sample_entry("lidar", "points", 100_000, b"\x04\x05"),
            sample_entry("cam", "frame", 200_000, b"\x06"),
        ];

        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
        for e in &entries {
            writer.write_entry(e).unwrap();
        }
        writer.finish().unwrap();

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        for expected in &entries {
            let got = reader.next_entry_header().unwrap().unwrap();
            assert_eq!(got.node_id, expected.node_id);
            assert_eq!(got.output_id, expected.output_id);
            assert_eq!(got.timestamp_offset_nanos, expected.timestamp_offset_nanos);
        }
        assert!(reader.next_entry_header().unwrap().is_none());
    }

    /// `next_entry_for_node` returns only the requested node's entries, in
    /// order, with their payloads intact, and skips every other node's entries.
    #[test]
    fn next_entry_for_node_filters_by_node() {
        let header = sample_header();
        let entries = vec![
            sample_entry("cam", "frame", 0, b"\x01\x02\x03"),
            sample_entry("lidar", "points", 100_000, b"\x04\x05"),
            sample_entry("cam", "frame", 200_000, b"\x06"),
        ];

        let mut buf = Vec::new();
        let mut writer = RecordingWriter::new(&mut buf, &header).unwrap();
        for e in &entries {
            writer.write_entry(e).unwrap();
        }
        writer.finish().unwrap();

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        let first = reader.next_entry_for_node("cam").unwrap().unwrap();
        assert_eq!(&first, &entries[0]);
        let second = reader.next_entry_for_node("cam").unwrap().unwrap();
        assert_eq!(&second, &entries[2]);
        // Only two `cam` entries exist; the `lidar` entry must have been skipped.
        assert!(reader.next_entry_for_node("cam").unwrap().is_none());
    }

    /// A corrupt record (a length field claiming more bytes than the record
    /// holds) must still be rejected when reached by `next_entry_for_node`,
    /// even for an entry from a node the caller is skipping — the validation
    /// is not bypassed by the filter.
    #[test]
    fn next_entry_for_node_still_rejects_corrupt_records() {
        let header = sample_header();
        let mut buf = Vec::new();
        write_header(&mut buf, &header).unwrap();

        // Hand-craft one record whose declared node_id length overruns the body.
        let node_id = b"other";
        let mut record = Vec::new();
        record.extend_from_slice(&(node_id.len() as u16 + 10).to_le_bytes()); // lie: too long
        record.extend_from_slice(node_id);
        let record_len = record.len() as u32;
        buf.extend_from_slice(&record_len.to_le_bytes());
        buf.extend_from_slice(&record);

        let mut reader = RecordingReader::open(std::io::Cursor::new(&buf)).unwrap();
        let err = reader
            .next_entry_for_node("cam")
            .expect_err("a corrupt record must be rejected, not silently skipped");
        assert!(
            err.to_string().contains("buffer too short"),
            "expected a bounds error, got: {err}"
        );
    }
}
