use std::io::{self, BufReader, BufWriter, Read, Write};

use eyre::Context;
use uuid::Uuid;

const MAGIC: &[u8; 8] = b"DORAREC\x00";
const FOOTER_MAGIC: &[u8; 8] = b"DORAEND\x00";
const FORMAT_VERSION: u16 = 1;
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
    pub fn new(inner: W, header: &RecordingHeader) -> eyre::Result<Self> {
        let mut writer = BufWriter::new(inner);
        write_header(&mut writer, header)?;
        Ok(Self {
            writer,
            total_messages: 0,
            total_bytes: 0,
        })
    }

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

        self.writer.write_all(&record_len.to_le_bytes())?;
        self.writer
            .write_all(&(node_id_bytes.len() as u16).to_le_bytes())?;
        self.writer.write_all(node_id_bytes)?;
        self.writer
            .write_all(&(output_id_bytes.len() as u16).to_le_bytes())?;
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

    pub fn flush(&mut self) -> eyre::Result<()> {
        self.writer.flush().wrap_err("flush failed")
    }

    pub fn stats(&self) -> (u64, u64) {
        (self.total_messages, self.total_bytes)
    }
}

/// Reads recording entries from disk.
#[derive(Debug)]
pub struct RecordingReader<R: Read> {
    reader: BufReader<R>,
    header: RecordingHeader,
}

impl<R: Read> RecordingReader<R> {
    pub fn open(inner: R) -> eyre::Result<Self> {
        let mut reader = BufReader::new(inner);
        let header = read_header(&mut reader)?;
        Ok(Self { reader, header })
    }

    pub fn header(&self) -> &RecordingHeader {
        &self.header
    }

    /// Read the next entry. Returns `None` at EOF or footer.
    pub fn next_entry(&mut self) -> eyre::Result<Option<RecordEntry>> {
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
        self.reader
            .read_exact(&mut record_buf)
            .wrap_err("truncated record")?;

        let mut pos = 0;

        let node_id_len = u16::from_le_bytes(read_array(&record_buf, &mut pos)?) as usize;
        let node_id = std::str::from_utf8(&record_buf[pos..pos + node_id_len])
            .wrap_err("invalid node_id utf8")?
            .to_string();
        pos += node_id_len;

        let output_id_len = u16::from_le_bytes(read_array(&record_buf, &mut pos)?) as usize;
        let output_id = std::str::from_utf8(&record_buf[pos..pos + output_id_len])
            .wrap_err("invalid output_id utf8")?
            .to_string();
        pos += output_id_len;

        let timestamp_offset_nanos = u64::from_le_bytes(read_array(&record_buf, &mut pos)?);
        let event_bytes_len = u32::from_le_bytes(read_array(&record_buf, &mut pos)?) as usize;
        let event_bytes = record_buf[pos..pos + event_bytes_len].to_vec();

        Ok(Some(RecordEntry {
            node_id,
            output_id,
            timestamp_offset_nanos,
            event_bytes,
        }))
    }
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
}
