//! Rewrite a dora recording (`.drec`) as an MCAP (`.mcap`) file, remuxing the
//! recorded payload bytes without decoding or re-encoding them.
//!
//! Each recorded `Output` entry becomes one MCAP message on an `arrow-ipc`
//! channel named `{node_id}/{output_id}`:
//! - `log_time`     = the recording wall clock:
//!   `header.start_nanos + entry.timestamp_offset_nanos`,
//! - `publish_time` = the producer HLC stamp (`metadata.timestamp()`), the same
//!   source `dora topic hz` reports for that topic,
//! - `sequence`     = per-channel monotonic 1-based counter,
//! - `data`         = the recorded Arrow IPC payload bytes verbatim.
//!
//! # Limits
//!
//! - `metadata.parameters` is **not** exported. That is where image outputs
//!   carry `width`/`height`/`encoding` and where services and actions carry
//!   `request_id`/`goal_id`, so an exported image topic cannot be decoded on
//!   its own. MCAP has no per-message metadata field to put them in.
//! - A recorded message with no payload (`data: None`, what a typical
//!   `send_output("tick", pa.array([]))` trigger leaves behind) is written as
//!   the Arrow IPC stream for a zero-length `NullArray`, which is what
//!   `dora replay` would deliver, rather than as an empty message.
//! - Chunks are uncompressed: `mcap` is built with `default-features = false`.
use std::{
    collections::{BTreeMap, HashMap, HashSet},
    fs::{self, File},
    io::{BufWriter, Seek, Write},
    path::{Path, PathBuf},
};

use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use dora_recording::RecordingReader;
use eyre::{WrapErr, eyre};
use mcap::Writer;
use mcap::records::{MessageHeader, Metadata};
use same_file::is_same_file;

#[derive(Debug, clap::Args)]
pub struct Export {
    /// Path to the `.drec` recording to convert
    #[clap(value_name = "RECORDING")]
    input: String,

    /// Path of the output `.mcap` file (defaults to the input with its
    /// extension replaced, e.g. `capture.drec` -> `capture.mcap`)
    #[clap(short, long, value_name = "OUTPUT")]
    output: Option<String>,

    /// Only export the given topics (`node_id/output_id`, comma-separated).
    /// Defaults to all topics.
    #[clap(long, value_name = "TOPIC", value_delimiter = ',')]
    topics: Vec<String>,
}

impl crate::command::Executable for Export {
    fn execute(self) -> eyre::Result<()> {
        run_export(self)
    }
}

/// The default output path: the input with its extension replaced rather than
/// appended to, so `capture.drec` becomes `capture.mcap` and not
/// `capture.drec.mcap`.
fn default_output_path(input: &str) -> String {
    Path::new(input)
        .with_extension("mcap")
        .to_string_lossy()
        .into_owned()
}

fn parse_topic_filter(topics: &[String]) -> eyre::Result<Vec<(String, String)>> {
    topics
        .iter()
        .map(|topic| {
            let (node, output) = topic
                .split_once('/')
                .ok_or_else(|| eyre!("invalid topic `{topic}`, expected `node_id/output_id`"))?;
            Ok((node.to_string(), output.to_string()))
        })
        .collect()
}

fn run_export(args: Export) -> eyre::Result<()> {
    let filter = parse_topic_filter(&args.topics)?;

    let input_file = File::open(&args.input)
        .wrap_err_with(|| eyre!("failed to open recording `{}`", args.input))?;

    let output = args
        .output
        .unwrap_or_else(|| default_output_path(&args.input));
    if output_aliases_input(&args.input, &output)? {
        return Err(eyre!(
            "output `{output}` is the input recording `{}` (or an alias of it); \
             refusing to truncate the source",
            args.input
        ));
    }

    let mut reader =
        RecordingReader::open(input_file).wrap_err("failed to initialise recording reader")?;

    // Write to a sibling temp file and rename on success: an export that fails
    // halfway must not leave a truncated `.mcap` where a readable one used to
    // be, and must not clobber an existing output until it has a complete
    // replacement. The temp name lives in the output's directory so the rename
    // stays on one filesystem and is therefore atomic.
    let tmp_path = temp_output_path(&output)?;
    let out_file = File::create(&tmp_path)
        .wrap_err_with(|| eyre!("failed to create `{}`", tmp_path.display()))?;
    let message_count = match write_mcap(&mut reader, &filter, BufWriter::new(out_file)) {
        Ok(count) => count,
        Err(err) => {
            let _ = fs::remove_file(&tmp_path);
            return Err(err);
        }
    };
    fs::rename(&tmp_path, &output).wrap_err_with(|| {
        eyre!(
            "failed to move the finished export into `{output}` (left as `{}`)",
            tmp_path.display()
        )
    })?;

    eprintln!(
        "Exported {message_count} messages to `{output}` (publish_time = producer HLC stamp, matching `dora topic hz`)"
    );
    Ok(())
}

/// A hidden sibling of `output` to write the export into. The pid keeps two
/// concurrent exports of different recordings in the same directory apart.
fn temp_output_path(output: &str) -> eyre::Result<PathBuf> {
    let output = Path::new(output);
    let file_name = output
        .file_name()
        .ok_or_else(|| eyre!("`{output:?}` is not a file path"))?;
    let parent = match output.parent() {
        Some(parent) if !parent.as_os_str().is_empty() => parent,
        // A bare file name has an empty parent; write next to the cwd.
        _ => Path::new("."),
    };
    Ok(parent.join(format!(
        ".{}.{}.tmp",
        file_name.to_string_lossy(),
        std::process::id()
    )))
}

fn write_mcap<W: Write + Seek>(
    reader: &mut RecordingReader<File>,
    filter: &[(String, String)],
    out: W,
) -> eyre::Result<u64> {
    let (start_nanos, dataflow_id, descriptor_yaml) = {
        let header = reader.header();
        (
            header.start_nanos,
            header.dataflow_id.to_string(),
            String::from_utf8_lossy(&header.descriptor_yaml).into_owned(),
        )
    };

    let mut writer = Writer::new(out).wrap_err("failed to initialise MCAP writer")?;
    writer
        .write_metadata(&Metadata {
            name: "dora-recording".to_string(),
            metadata: BTreeMap::from([
                ("dataflow_id".to_string(), dataflow_id),
                ("descriptor_yaml".to_string(), descriptor_yaml),
            ]),
        })
        .wrap_err("failed to write MCAP metadata")?;

    // Constant, so encode it once rather than per empty message.
    let empty_ipc = encode_empty_ipc()?;

    // One lookup per topic: the channel it was added as, and its next
    // per-channel sequence. `write_to_known_channel` then skips rebuilding an
    // `Arc<Channel>` and re-cloning the topic for every message.
    let mut channels: HashMap<String, (u16, u32)> = HashMap::new();
    let mut matched: HashSet<usize> = HashSet::new();
    let mut message_count: u64 = 0;

    while let Some(entry) = reader
        .next_entry()
        .wrap_err("failed to read a recording entry")?
    {
        if !filter.is_empty() {
            let key = (&entry.node_id, &entry.output_id);
            let Some(pos) = filter.iter().position(|(n, o)| (n, o) == key) else {
                continue;
            };
            matched.insert(pos);
        }

        let timestamped = Timestamped::deserialize_inter_daemon_event(&entry.event_bytes)
            .wrap_err("failed to deserialize a recorded event")?;
        let (publish_time, publish_data) = match timestamped.inner {
            InterDaemonEvent::Output { metadata, data, .. } => (
                metadata.timestamp().get_time().to_duration().as_nanos() as u64,
                data,
            ),
            _ => continue,
        };

        let topic = format!("{}/{}", entry.node_id, entry.output_id);
        let (channel_id, sequence) = match channels.get_mut(&topic) {
            Some(state) => {
                state.1 = state.1.checked_add(1).ok_or_else(|| {
                    eyre!("recording has more than 2^32 messages on one topic `{topic}`")
                })?;
                *state
            }
            None => {
                let channel_id = writer
                    .add_channel(0, &topic, "arrow-ipc", &BTreeMap::new())
                    .wrap_err_with(|| eyre!("failed to add MCAP channel `{topic}`"))?;
                channels.insert(topic.clone(), (channel_id, 1));
                (channel_id, 1)
            }
        };

        // A metadata-only message is recorded as `data: None` but replays as a
        // zero-length `NullArray`, so it has to be written as that IPC stream:
        // a zero-byte message is not a decodable Arrow stream, and would fail
        // every reader on the channel.
        let data = match publish_data.as_deref() {
            Some(data) => data,
            None => empty_ipc.as_slice(),
        };

        writer
            .write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence,
                    log_time: start_nanos.saturating_add(entry.timestamp_offset_nanos),
                    publish_time,
                },
                data,
            )
            .wrap_err_with(|| eyre!("failed to write MCAP message on `{topic}`"))?;
        message_count += 1;
    }

    // `finish` writes the summary and footer and *then* flushes
    // (`write_summary_and_footer_magic` ends in `writer.flush()?`,
    // mcap-0.25.0 `write.rs:1460`), and that error is the one that matters: a
    // full disk or a quota has to fail the export here, before the rename, or
    // the write-then-rename above would put an `.mcap` with no footer where a
    // readable one used to be and still report success. Both `Writer`'s and
    // `BufWriter`'s `Drop` throw errors away, so nothing may be left to them;
    // `a_flush_that_fails_fails_the_export` pins that.
    writer.finish().wrap_err("failed to finish MCAP file")?;

    for (node, output_id) in unmatched_topics(filter, &matched) {
        eprintln!(
            "warning: --topics entry `{node}/{output_id}` matched no messages in the recording"
        );
    }

    Ok(message_count)
}

/// The `--topics` entries that never matched a recorded message: a typo, or a
/// topic this recording never carried. Worth saying out loud, since the export
/// otherwise looks successful.
fn unmatched_topics<'a>(
    filter: &'a [(String, String)],
    matched: &HashSet<usize>,
) -> Vec<&'a (String, String)> {
    filter
        .iter()
        .enumerate()
        .filter(|(pos, _)| !matched.contains(pos))
        .map(|(_, topic)| topic)
        .collect()
}

/// The Arrow IPC stream for a zero-length `NullArray`, matching what `dora
/// replay` delivers for a recorded message with no payload.
fn encode_empty_ipc() -> eyre::Result<Vec<u8>> {
    use arrow::array::NullArray;
    use arrow::ipc::writer::StreamWriter;
    use arrow::record_batch::RecordBatch;
    use arrow_schema::{DataType, Field, Schema};
    use std::sync::Arc;

    let schema = Arc::new(Schema::new(vec![Field::new("data", DataType::Null, true)]));
    let batch = RecordBatch::try_new(schema.clone(), vec![Arc::new(NullArray::new(0))])
        .wrap_err("failed to build the empty Arrow record batch")?;

    let mut buf = Vec::new();
    {
        let mut stream =
            StreamWriter::try_new(&mut buf, schema.as_ref()).wrap_err("failed to open stream")?;
        stream
            .write(&batch)
            .wrap_err("failed to write the empty Arrow record batch")?;
        stream
            .finish()
            .wrap_err("failed to finish the Arrow stream")?;
    }
    Ok(buf)
}

/// Whether `output` refers to the same file as the `input` recording — via
/// the identical path or an existing hardlink/symlink alias. `File::create`
/// would truncate such an output before the recording were read, destroying
/// the source, so this is checked before any output file is opened.
///
/// Uses `same_file::is_same_file`, which compares device + inode (Unix) or
/// volume + file index (Windows) and works with bare relative paths. An
/// output that does not exist yet cannot be an alias of the input.
fn output_aliases_input(input: &str, output: &str) -> eyre::Result<bool> {
    match is_same_file(input, output) {
        Ok(same) => Ok(same),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(false),
        Err(e) => Err(e).wrap_err_with(|| eyre!("failed to compare `{output}` with the input")),
    }
}

#[cfg(test)]
mod tests {
    use std::{
        fs,
        io::{BufWriter, Seek, Write},
    };

    use super::{Export, default_output_path, run_export, unmatched_topics, write_mcap};
    use aligned_vec::AVec;
    use dora_message::{
        common::Timestamped,
        daemon_to_daemon::InterDaemonEvent,
        id::{DataId, NodeId},
        metadata::Metadata,
        uhlc::{ID, NTP64, Timestamp},
    };
    use dora_recording::{
        FORMAT_VERSION, RecordEntry, RecordingHeader, RecordingReader, RecordingWriter,
    };
    use mcap::MessageStream;
    use tempfile::tempdir;
    use uuid::Uuid;

    /// A fixed but non-round HLC timestamp and 16-byte id, mirrored from the
    /// wire-format golden vectors (`libraries/message/tests/uhlc_wire_format.rs`).
    fn sample_timestamp() -> Timestamp {
        let id_bytes: [u8; 16] = [
            0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e,
            0x0f, 0x10,
        ];
        Timestamp::new(
            NTP64(0x1122_3344_5566_7788),
            ID::try_from(&id_bytes).expect("non-zero id"),
        )
    }

    /// Encode a recorded `Output` entry exactly the way the daemon does
    /// (`binaries/daemon/src/lib.rs`): a `Timestamped` envelope around the
    /// `InterDaemonEvent`, postcard-serialized into the entry's event bytes.
    fn output_event_bytes(node_id: &str, output_id: &str, payload: &[u8]) -> Vec<u8> {
        let producer_ts = sample_timestamp();
        // The recording envelope carries the daemon's forward-time stamp,
        // which is later than the producer's HLC stamp (the propagation
        // delta). Keep the two distinct so the test genuinely pins the
        // documented contract that MCAP `publish_time` comes from the
        // producer's `metadata.timestamp()` -- not the envelope stamp, which
        // would otherwise slip through unnoticed.
        let envelope_ts =
            Timestamp::new(*producer_ts.get_time() + 1_000_000, *producer_ts.get_id());
        let metadata = Metadata::new(producer_ts);
        let event = InterDaemonEvent::Output {
            dataflow_id: Uuid::nil(),
            node_id: NodeId::from(node_id.to_string()),
            output_id: DataId::from(output_id.to_string()),
            metadata,
            data: Some(AVec::<u8, aligned_vec::ConstAlign<128>>::from_slice(
                128, payload,
            )),
        };
        Timestamped {
            inner: event,
            timestamp: envelope_ts,
        }
        .serialize()
        .expect("serialize output event")
    }

    #[test]
    fn round_trip_drec_to_mcap_preserves_payload_time_and_topics() {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("input.drec");
        let mcap_path = dir.path().join("output.mcap");

        // Mirror the recording header the daemon writes: fixed start time and
        // an empty (never used by export) dataflow descriptor.
        let header = RecordingHeader {
            version: FORMAT_VERSION,
            start_nanos: 1_000_000_000,
            dataflow_id: Uuid::nil(),
            descriptor_yaml: b"nodes: []".to_vec(),
        };

        let payload_a = b"ARROW1\x00\x00\x00\x00\x01\x00\x00\x00\x0a\x00\x00\x00".to_vec();
        let payload_b = b"ARROW1\x00\x00\x00\x00\x02\x00\x00\x00\x0b\x00\x00\x00".to_vec();
        let payload_c = b"ARROW1\x00\x00\x00\x00\x03\x00\x00\x00\x0c\x00\x00\x00\x0d\x00".to_vec();

        {
            let file = fs::File::create(&recording_path).expect("create recording");
            let mut writer =
                RecordingWriter::new(BufWriter::new(file), &header).expect("init writer");
            writer
                .write_entry(&RecordEntry {
                    node_id: "camera".to_string(),
                    output_id: "image".to_string(),
                    timestamp_offset_nanos: 100,
                    event_bytes: output_event_bytes("camera", "image", &payload_a),
                })
                .expect("write entry");
            writer
                .write_entry(&RecordEntry {
                    node_id: "camera".to_string(),
                    output_id: "image".to_string(),
                    timestamp_offset_nanos: 500,
                    event_bytes: output_event_bytes("camera", "image", &payload_b),
                })
                .expect("write entry");
            writer
                .write_entry(&RecordEntry {
                    node_id: "lidar".to_string(),
                    output_id: "points".to_string(),
                    timestamp_offset_nanos: 300,
                    event_bytes: output_event_bytes("lidar", "points", &payload_c),
                })
                .expect("write entry");
            // A non-`Output` event must be skipped, not abort the export.
            writer
                .write_entry(&RecordEntry {
                    node_id: "lidar".to_string(),
                    output_id: "points".to_string(),
                    timestamp_offset_nanos: 400,
                    event_bytes: Timestamped {
                        inner: InterDaemonEvent::OutputClosed {
                            dataflow_id: Uuid::nil(),
                            node_id: NodeId::from("lidar".to_string()),
                            output_id: DataId::from("points".to_string()),
                        },
                        timestamp: sample_timestamp(),
                    }
                    .serialize()
                    .expect("serialize output closed"),
                })
                .expect("write entry");
            writer.finish().expect("finish recording");
        }

        let args = Export {
            input: recording_path.to_string_lossy().into(),
            output: Some(mcap_path.to_string_lossy().into()),
            topics: vec![],
        };
        run_export(args).expect("run export");

        let mcap_bytes = fs::read(&mcap_path).expect("read mcap");
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read mcap messages");

        let publish_nanos = sample_timestamp().get_time().to_duration().as_nanos() as u64;

        assert_eq!(messages.len(), 3, "OutputClosed entry must be skipped");
        assert_eq!(messages[0].channel.topic, "camera/image");
        assert_eq!(messages[0].channel.message_encoding, "arrow-ipc");
        assert_eq!(messages[0].sequence, 1);
        assert_eq!(messages[0].log_time, 1_000_000_100);
        assert_eq!(messages[0].publish_time, publish_nanos);
        assert_eq!(messages[0].data.as_ref(), &payload_a[..]);

        assert_eq!(messages[1].channel.topic, "camera/image");
        assert_eq!(messages[1].sequence, 2, "sequence increments per topic");
        assert_eq!(messages[1].log_time, 1_000_000_500);
        assert_eq!(messages[1].publish_time, publish_nanos);
        assert_eq!(messages[1].data.as_ref(), &payload_b[..]);

        assert_eq!(messages[2].channel.topic, "lidar/points");
        assert_eq!(
            messages[2].sequence, 1,
            "separate topic restarts its sequence"
        );
        assert_eq!(messages[2].log_time, 1_000_000_300);
        assert_eq!(messages[2].publish_time, publish_nanos);
        assert_eq!(messages[2].data.as_ref(), &payload_c[..]);

        let summary = mcap::Summary::read(&mcap_bytes)
            .expect("read summary")
            .expect("summary section present");
        assert_eq!(summary.metadata_indexes.len(), 1);
        assert_eq!(summary.metadata_indexes[0].name, "dora-recording");
    }

    #[test]
    fn export_respects_topic_filter() {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("input.drec");
        let mcap_path = dir.path().join("output.mcap");

        let header = RecordingHeader {
            version: FORMAT_VERSION,
            start_nanos: 1_000_000_000,
            dataflow_id: Uuid::nil(),
            descriptor_yaml: b"nodes: []".to_vec(),
        };
        {
            let file = fs::File::create(&recording_path).expect("create recording");
            let mut writer =
                RecordingWriter::new(BufWriter::new(file), &header).expect("init writer");
            writer
                .write_entry(&RecordEntry {
                    node_id: "camera".to_string(),
                    output_id: "image".to_string(),
                    timestamp_offset_nanos: 100,
                    event_bytes: output_event_bytes("camera", "image", b"camera-payload"),
                })
                .expect("write entry");
            writer
                .write_entry(&RecordEntry {
                    node_id: "lidar".to_string(),
                    output_id: "points".to_string(),
                    timestamp_offset_nanos: 200,
                    event_bytes: output_event_bytes("lidar", "points", b"lidar-payload"),
                })
                .expect("write entry");
            writer.finish().expect("finish recording");
        }

        let args = Export {
            input: recording_path.to_string_lossy().into(),
            output: Some(mcap_path.to_string_lossy().into()),
            topics: vec!["camera/image".to_string()],
        };
        run_export(args).expect("run export");

        let mcap_bytes = fs::read(&mcap_path).expect("read mcap");
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read mcap messages");

        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].channel.topic, "camera/image");
        assert_eq!(messages[0].data.as_ref(), &b"camera-payload"[..]);
    }

    #[test]
    fn parse_topic_filter_splits_node_and_output() {
        let filter = super::parse_topic_filter(&["camera/image".to_string()]).expect("parse");
        assert_eq!(filter, vec![("camera".to_string(), "image".to_string())]);
    }

    #[test]
    fn parse_topic_filter_rejects_missing_output_slash() {
        let err = super::parse_topic_filter(&["camera".to_string()]).expect_err("must reject");
        assert!(
            err.to_string().contains("expected `node_id/output_id`"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn parse_topic_filter_defaults_to_all_when_empty() {
        let filter = super::parse_topic_filter(&[]).expect("parse");
        assert!(filter.is_empty());
    }

    /// Write a single-entry recording; enough for the same-file guard tests,
    /// which only run far enough to hit the rejection.
    fn write_minimal_recording(path: &std::path::Path) {
        let header = RecordingHeader {
            version: FORMAT_VERSION,
            start_nanos: 1_000_000_000,
            dataflow_id: Uuid::nil(),
            descriptor_yaml: b"nodes: []".to_vec(),
        };
        let file = fs::File::create(path).expect("create recording");
        let mut writer = RecordingWriter::new(BufWriter::new(file), &header).expect("init writer");
        writer
            .write_entry(&RecordEntry {
                node_id: "camera".to_string(),
                output_id: "image".to_string(),
                timestamp_offset_nanos: 100,
                event_bytes: output_event_bytes("camera", "image", b"camera-payload"),
            })
            .expect("write entry");
        writer.finish().expect("finish recording");
    }

    fn same_file_err(args: Export) -> String {
        run_export(args)
            .expect_err("export must refuse an output aliasing the input")
            .to_string()
    }

    #[test]
    fn export_rejects_output_matching_input() {
        let dir = tempdir().expect("tempdir");
        let path = dir.path().join("sample.drec");
        write_minimal_recording(&path);

        let err = same_file_err(Export {
            input: path.to_string_lossy().into(),
            output: Some(path.to_string_lossy().into()),
            topics: vec![],
        });
        assert!(
            err.contains("refusing to truncate"),
            "unexpected error: {err}"
        );
    }

    #[cfg(unix)]
    #[test]
    fn export_rejects_output_hardlinked_to_input() {
        let dir = tempdir().expect("tempdir");
        let path = dir.path().join("sample.drec");
        write_minimal_recording(&path);

        let alias = dir.path().join("alias.mcap");
        fs::hard_link(&path, &alias).expect("create hard link");

        let err = same_file_err(Export {
            input: path.to_string_lossy().into(),
            output: Some(alias.to_string_lossy().into()),
            topics: vec![],
        });
        assert!(
            err.contains("refusing to truncate"),
            "unexpected error: {err}"
        );
    }

    #[cfg(unix)]
    #[test]
    fn export_rejects_output_symlinked_to_input() {
        let dir = tempdir().expect("tempdir");
        let path = dir.path().join("sample.drec");
        write_minimal_recording(&path);

        let alias = dir.path().join("alias.mcap");
        std::os::unix::fs::symlink(&path, &alias).expect("create symlink");

        let err = same_file_err(Export {
            input: path.to_string_lossy().into(),
            output: Some(alias.to_string_lossy().into()),
            topics: vec![],
        });
        assert!(
            err.contains("refusing to truncate"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn export_allows_distinct_fresh_output() {
        let dir = tempdir().expect("tempdir");
        let path = dir.path().join("sample.drec");
        write_minimal_recording(&path);

        let out = dir.path().join("fresh.mcap");
        let args = Export {
            input: path.to_string_lossy().into(),
            output: Some(out.to_string_lossy().into()),
            topics: vec![],
        };
        run_export(args).expect("fresh output must succeed");
        assert!(!fs::read(&out).expect("read mcap").is_empty());
    }

    /// Serializes tests that change the process working directory.
    static CWD_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Restores the process working directory on `Drop`, so a failing
    /// assertion after `set_current_dir` cannot leak the tempdir as the
    /// next test's cwd.
    struct CwdGuard(std::path::PathBuf);
    impl Drop for CwdGuard {
        fn drop(&mut self) {
            let _ = std::env::set_current_dir(&self.0);
        }
    }

    #[test]
    fn export_rejects_bare_relative_output_matching_input() {
        let _lock = CWD_LOCK.lock().expect("cwd lock");
        let dir = tempdir().expect("tempdir");
        write_minimal_recording(&dir.path().join("sample.drec"));

        let _guard = CwdGuard(std::env::current_dir().expect("current dir"));
        std::env::set_current_dir(dir.path()).expect("chdir to tempdir");
        let err = run_export(Export {
            input: "sample.drec".to_string(),
            output: Some("sample.drec".to_string()),
            topics: vec![],
        })
        .expect_err("must refuse output = input")
        .to_string();

        assert!(
            err.contains("refusing to truncate"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn export_allows_bare_relative_fresh_output() {
        let _lock = CWD_LOCK.lock().expect("cwd lock");
        let dir = tempdir().expect("tempdir");
        write_minimal_recording(&dir.path().join("sample.drec"));

        let _guard = CwdGuard(std::env::current_dir().expect("current dir"));
        std::env::set_current_dir(dir.path()).expect("chdir to tempdir");
        let result = run_export(Export {
            input: "sample.drec".to_string(),
            output: Some("fresh.mcap".to_string()),
            topics: vec![],
        });

        result.expect("fresh bare-relative output must succeed");
        assert!(dir.path().join("fresh.mcap").exists(), "output written");
    }

    /// A recorded `Output` entry with no payload, the way a `send_output("tick",
    /// pa.array([]))` trigger is recorded.
    fn empty_output_event_bytes(node_id: &str, output_id: &str) -> Vec<u8> {
        let event = InterDaemonEvent::Output {
            dataflow_id: Uuid::nil(),
            node_id: NodeId::from(node_id.to_string()),
            output_id: DataId::from(output_id.to_string()),
            metadata: Metadata::new(sample_timestamp()),
            data: None,
        };
        Timestamped {
            inner: event,
            timestamp: sample_timestamp(),
        }
        .serialize()
        .expect("serialize empty output event")
    }

    fn header() -> RecordingHeader {
        RecordingHeader {
            version: FORMAT_VERSION,
            start_nanos: 1_000_000_000,
            dataflow_id: Uuid::nil(),
            descriptor_yaml: b"nodes: []".to_vec(),
        }
    }

    /// A recorded `data: None` message has to land as a decodable Arrow stream.
    /// The old remux wrote zero bytes, which is not a valid Arrow IPC stream at
    /// all: every consumer of that channel would fail on the message.
    #[test]
    fn empty_payload_is_exported_as_a_decodable_arrow_stream() {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("input.drec");
        let mcap_path = dir.path().join("output.mcap");

        {
            let file = fs::File::create(&recording_path).expect("create recording");
            let mut writer =
                RecordingWriter::new(BufWriter::new(file), &header()).expect("init writer");
            writer
                .write_entry(&RecordEntry {
                    node_id: "tick".to_string(),
                    output_id: "tick".to_string(),
                    timestamp_offset_nanos: 100,
                    event_bytes: empty_output_event_bytes("tick", "tick"),
                })
                .expect("write entry");
            writer.finish().expect("finish recording");
        }

        run_export(Export {
            input: recording_path.to_string_lossy().into(),
            output: Some(mcap_path.to_string_lossy().into()),
            topics: vec![],
        })
        .expect("run export");

        let mcap_bytes = fs::read(&mcap_path).expect("read mcap");
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read mcap messages");
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].channel.topic, "tick/tick");

        // Decodes as the zero-length `NullArray` that `dora replay` delivers.
        let reader = arrow::ipc::reader::StreamReader::try_new(
            std::io::Cursor::new(messages[0].data.as_ref()),
            None,
        )
        .expect("exported empty payload must be a valid Arrow IPC stream");
        let schema = reader.schema();
        assert_eq!(schema.fields().len(), 1);
        assert_eq!(schema.field(0).data_type(), &arrow_schema::DataType::Null);
        let batches: Vec<_> = reader.collect::<Result<Vec<_>, _>>().expect("read batches");
        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].num_rows(), 0);
        assert_eq!(batches[0].num_columns(), 1);
    }

    #[test]
    fn default_output_replaces_the_recording_extension() {
        assert_eq!(default_output_path("capture.drec"), "capture.mcap");
        assert_eq!(
            default_output_path("/data/runs/capture.drec"),
            "/data/runs/capture.mcap"
        );
        // An extension-less or differently-named recording still gets `.mcap`
        // rather than `capture.mcap.drec`.
        assert_eq!(default_output_path("capture"), "capture.mcap");
        assert_eq!(default_output_path("capture.tar.drec"), "capture.tar.mcap");
    }

    #[test]
    fn default_output_lands_next_to_the_recording() {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("input.drec");
        write_minimal_recording(&recording_path);

        run_export(Export {
            input: recording_path.to_string_lossy().into(),
            output: None,
            topics: vec![],
        })
        .expect("run export");

        assert!(
            dir.path().join("input.mcap").exists(),
            "default output replaces the input extension, it does not append it"
        );
    }

    /// A failed export must not leave a half-written file where a good one
    /// used to be, and must not leave its temp file behind either.
    #[test]
    fn failed_export_keeps_the_previous_output_and_cleans_up() {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("garbled.drec");
        let mcap_path = dir.path().join("output.mcap");
        // A structurally valid recording whose one entry carries unparseable
        // event bytes: the header parses and the temp file is created, so the
        // export only fails once it is already writing.
        {
            let file = fs::File::create(&recording_path).expect("create recording");
            let mut writer =
                RecordingWriter::new(BufWriter::new(file), &header()).expect("init writer");
            writer
                .write_entry(&RecordEntry {
                    node_id: "camera".to_string(),
                    output_id: "image".to_string(),
                    timestamp_offset_nanos: 100,
                    event_bytes: vec![0xff; 16],
                })
                .expect("write entry");
            writer.finish().expect("finish recording");
        }
        fs::write(&mcap_path, b"previous good export").expect("seed output");

        let err = run_export(Export {
            input: recording_path.to_string_lossy().into(),
            output: Some(mcap_path.to_string_lossy().into()),
            topics: vec![],
        })
        .expect_err("unparseable event bytes must fail the export");

        assert!(
            err.to_string().contains("deserialize"),
            "the failure must come from the entry loop, i.e. after the temp file \
             exists, otherwise this test proves nothing: {err}"
        );

        assert_eq!(
            fs::read(&mcap_path).expect("read previous output"),
            b"previous good export",
            "a failed export must not clobber the existing output"
        );
        let leftovers: Vec<_> = fs::read_dir(dir.path())
            .expect("read dir")
            .filter_map(|entry| entry.ok().map(|entry| entry.file_name()))
            .filter(|name| name.to_string_lossy().ends_with(".tmp"))
            .collect();
        assert!(
            leftovers.is_empty(),
            "a failed export must remove its temp file, left {leftovers:?}"
        );
    }

    /// A flush that fails has to fail the export. `finish` writes the footer
    /// into the buffer and never flushes it, and both `Writer`'s and
    /// `BufWriter`'s `Drop` throw the error away — so a full disk used to be
    /// renamed into place as an `.mcap` with no footer, while the CLI reported
    /// success. That is the one outcome the temp-file-and-rename exists to
    /// prevent (#3541 review).
    #[test]
    fn a_flush_that_fails_fails_the_export() {
        /// Takes every byte, then fails the way a full disk does at the one
        /// moment the export cannot recover from.
        struct FailsOnFlush(std::io::Cursor<Vec<u8>>);

        impl Write for FailsOnFlush {
            fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
                self.0.write(buf)
            }

            fn flush(&mut self) -> std::io::Result<()> {
                Err(std::io::Error::other("no space left on device"))
            }
        }

        impl Seek for FailsOnFlush {
            fn seek(&mut self, pos: std::io::SeekFrom) -> std::io::Result<u64> {
                self.0.seek(pos)
            }
        }

        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("input.drec");
        {
            let file = fs::File::create(&recording_path).expect("create recording");
            let mut writer =
                RecordingWriter::new(BufWriter::new(file), &header()).expect("init writer");
            writer
                .write_entry(&RecordEntry {
                    node_id: "tick".to_string(),
                    output_id: "tick".to_string(),
                    timestamp_offset_nanos: 100,
                    event_bytes: empty_output_event_bytes("tick", "tick"),
                })
                .expect("write entry");
            writer.finish().expect("finish recording");
        }

        let input_file = fs::File::open(&recording_path).expect("open recording");
        let mut reader = RecordingReader::open(input_file).expect("init reader");
        // The export must fail, rather than rename a footerless `.mcap` into
        // place over a good one and report success. If this ever returns `Ok`,
        // the flush stopped being checked — either here or in `mcap` — and the
        // temp-file-and-rename is no longer atomic in the only way it can fail.
        let err = write_mcap(
            &mut reader,
            &[],
            FailsOnFlush(std::io::Cursor::new(Vec::new())),
        )
        .expect_err("a flush that fails must fail the export");
        assert!(
            err.to_string().contains("finish") || err.to_string().contains("flush"),
            "the failure must come from the end of the write, not from an early \
             read or write, otherwise this test proves nothing: {err}"
        );
    }

    #[test]
    fn unmatched_topic_filter_entries_are_reported() {
        let filter = vec![
            ("camera".to_string(), "image".to_string()),
            ("lidar".to_string(), "points".to_string()),
            ("typo".to_string(), "nope".to_string()),
        ];
        let matched = [0usize, 1].into_iter().collect();

        assert_eq!(
            unmatched_topics(&filter, &matched),
            vec![&("typo".to_string(), "nope".to_string())]
        );
    }
}
