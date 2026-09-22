//! Rewrite a dora recording (`.drec`) as an MCAP (`.mcap`) file **without
//! re-encoding any payload**.
//!
//! Tier 1 is a pure remux. Each recorded `Output` entry is decoded only enough
//! to recover:
//!  - its **topic** (`{node_id}/{output_id}` — the same topic string
//!    Foxglove/Foxglove Mesh V2 + `dora replay` use to name it, and the same
//!    per-topic key `dora topic hz` reports),
//!  - its **producer HLC stamp** ([`MetadataHeader::timestamp`]) — the same
//!    source the merged hz producer-stamp fix (#3523) reads for the same
//!    topic (`metadata.timestamp().get_time().to_duration().as_nanos()`),
//!  - and the **Arrow IPC payload bytes verbatim** — never decoded, never
//!    re-encoded.
//!
//! Time mapping (the contract the merged hz fix establishes, so a
//! publish-vs-log scatter agrees with what `dora topic hz` reports for the
//! same topic):
//! - `log_time`     = the recording wall clock: `header.start_nanos +
//!   entry.timestamp_offset_nanos`. (For the Arrow IPC payload this is **not**
//!   the value on the wire — the wire value is the length-prefixed Arrow IPC
//!   header of the payload itself. We preserve that verbatim as the message
//!   `data`, and put the wall clock in `log_time` as the recording contract
//!   requires.)
//! - `publish_time` = the producer HLC stamp, expressed as elapsed nanos
//!   (the same source `hz` uses),
//! - `sequence`     = per-channel monotonic 1-based counter (matching what
//!   `dora replay` delivers).
use std::{
    borrow::Cow,
    collections::{BTreeMap, HashMap},
    fs::File,
    io::BufWriter,
    sync::Arc,
};

use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};
use dora_recording::RecordingReader;
use eyre::{WrapErr, eyre};
use mcap::{Channel, Message, Writer};

#[derive(Debug, clap::Args)]
pub struct Export {
    /// Path to the `.drec` recording to convert
    #[clap(value_name = "RECORDING")]
    input: String,

    /// Path of the output `.mcap` file (defaults to `<input>.mcap`)
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
        .unwrap_or_else(|| format!("{}.mcap", args.input));
    if same_file(&input_file, &args.input, &output)? {
        return Err(eyre!(
            "output `{output}` is the input recording `{}` (or an alias of it); \
             refusing to truncate the source",
            args.input
        ));
    }

    let mut reader =
        RecordingReader::open(input_file).wrap_err("failed to initialise recording reader")?;
    let start_nanos: u64 = reader.header().start_nanos;

    let out_file =
        File::create(&output).wrap_err_with(|| eyre!("failed to create output `{}`", output))?;
    let mut writer =
        Writer::new(BufWriter::new(out_file)).wrap_err("failed to initialise MCAP writer")?;

    let mut channel_ids: HashMap<(String, String), u16> = HashMap::new();
    let mut sequences: HashMap<(String, String), u64> = HashMap::new();
    let mut message_count: u64 = 0;

    while let Some(entry) = reader
        .next_entry()
        .wrap_err("failed to read a recording entry")?
    {
        let node_id = entry.node_id.clone();
        let output_id = entry.output_id.clone();
        let topic = format!("{node_id}/{output_id}");
        if !filter.is_empty() && !filter.contains(&(node_id.clone(), output_id.clone())) {
            continue;
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

        let channel_id = if let Some(&id) = channel_ids.get(&(node_id.clone(), output_id.clone())) {
            id
        } else {
            let id = writer
                .add_channel(0, &topic, "arrow-ipc", &BTreeMap::new())
                .wrap_err_with(|| eyre!("failed to add MCAP channel `{topic}`"))?;
            channel_ids.insert((node_id.clone(), output_id.clone()), id);
            id
        };

        let sequence = sequences
            .entry((node_id.clone(), output_id.clone()))
            .or_insert(0);
        *sequence += 1;

        let log_time = start_nanos.saturating_add(entry.timestamp_offset_nanos);

        let message = Message {
            channel: Arc::new(Channel {
                id: channel_id,
                topic,
                schema: None,
                message_encoding: "arrow-ipc".to_string(),
                metadata: BTreeMap::new(),
            }),
            sequence: *sequence as u32,
            log_time,
            publish_time,
            data: Cow::Borrowed(publish_data.as_deref().unwrap_or(&[])),
        };
        writer
            .write(&message)
            .wrap_err("failed to write MCAP message")?;
        message_count += 1;
    }

    writer.finish().wrap_err("failed to finish MCAP file")?;
    eprintln!(
        "Exported {message_count} messages to `{output}` (publish_time = producer HLC stamp, matching `dora topic hz`)"
    );
    Ok(())
}

/// Normalizes `path` without requiring the file to exist yet: the parent
/// directory is canonicalized (resolving symlinks and `.`/`..`), then the
/// final component is re-applied.
fn normalized(path: &std::path::Path) -> eyre::Result<std::path::PathBuf> {
    let parent = path.parent().unwrap_or_else(|| std::path::Path::new("."));
    let name = path.file_name();
    let resolved = std::fs::canonicalize(parent)
        .wrap_err_with(|| eyre!("failed to resolve parent directory of `{}`", path.display()))?;
    Ok(match name {
        Some(name) => resolved.join(name),
        None => resolved,
    })
}

/// The platform's stable file identity (device + inode on Unix).
#[cfg(unix)]
fn file_identity(meta: &std::fs::Metadata) -> (u64, u64) {
    use std::os::unix::fs::MetadataExt;
    (meta.dev(), meta.ino())
}

/// The platform's stable file identity (volume + file index on Windows).
#[cfg(windows)]
fn file_identity(meta: &std::fs::Metadata) -> (u64, u64) {
    use std::os::windows::fs::MetadataExt;
    (meta.volume_serial_number(), meta.file_index())
}

/// Whether `output` refers to the same file as the open `input` recording —
/// via an identical path or an existing hardlink/symlink alias. `File::create`
/// would truncate such an output before the recording were read, destroying
/// the source, so this is checked before any output file is opened.
fn same_file(input: &File, input_path: &str, output: &str) -> eyre::Result<bool> {
    if normalized(std::path::Path::new(input_path))? == normalized(std::path::Path::new(output))? {
        return Ok(true);
    }

    // An output that already exists may be a hardlink or symlink alias of the
    // input; compare file identity rather than path strings.
    match std::fs::metadata(output) {
        Ok(out_meta) => {
            let in_meta = input
                .metadata()
                .wrap_err("failed to stat the input recording")?;
            Ok(file_identity(&in_meta) == file_identity(&out_meta))
        }
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(false),
        Err(e) => Err(e).wrap_err_with(|| eyre!("failed to stat output `{output}`")),
    }
}

#[cfg(test)]
mod tests {
    use std::{fs, io::BufWriter};

    use super::{Export, run_export};
    use aligned_vec::AVec;
    use dora_message::{
        common::Timestamped,
        daemon_to_daemon::InterDaemonEvent,
        id::{DataId, NodeId},
        metadata::Metadata,
        uhlc::{ID, NTP64, Timestamp},
    };
    use dora_recording::{FORMAT_VERSION, RecordEntry, RecordingHeader, RecordingWriter};
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
        assert!(fs::read(&out).expect("read mcap").len() > 0);
    }
}
