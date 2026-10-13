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

mod foxglove;

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

    /// How to encode messages. `arrow` (the default) copies every payload
    /// byte for byte. `foxglove` re-encodes recognised image outputs as
    /// Foxglove's `RawImage` / `CompressedImage` (JSON) so Foxglove can
    /// display them; other outputs stay `arrow`.
    #[clap(long, value_enum, default_value_t = ExportSchema::Arrow)]
    schema: ExportSchema,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum ExportSchema {
    Arrow,
    Foxglove,
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

/// Parse `--topics` into `(node_id, output_id)` pairs, dropping repeats.
///
/// A repeated topic must not get its own slot: matching records only the
/// first slot a message hits, so the repeat would never be marked matched and
/// the export would warn that a topic it did export "matched no messages".
fn parse_topic_filter(topics: &[String]) -> eyre::Result<Vec<(String, String)>> {
    let mut filter = Vec::with_capacity(topics.len());
    for topic in topics {
        let (node, output) = topic
            .split_once('/')
            .ok_or_else(|| eyre!("invalid topic `{topic}`, expected `node_id/output_id`"))?;
        let entry = (node.to_string(), output.to_string());
        if !filter.contains(&entry) {
            filter.push(entry);
        }
    }
    Ok(filter)
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
    let message_count =
        match write_mcap(&mut reader, &filter, args.schema, BufWriter::new(out_file)) {
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
    schema: ExportSchema,
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

    let foxglove = (schema == ExportSchema::Foxglove)
        .then(|| foxglove::Mapper::from_descriptor_yaml(&descriptor_yaml));
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

    let mut foxglove_schemas: HashMap<foxglove::Kind, u16> = HashMap::new();
    // Per topic: how many image messages were dropped, and why the first was.
    let mut dropped: BTreeMap<String, (u64, String)> = BTreeMap::new();

    // One lookup per topic: the channel it was added as, and its next
    // per-channel sequence. `write_to_known_channel` then skips rebuilding an
    // `Arc<Channel>` and re-cloning the topic for every message.
    let mut channels: HashMap<String, TopicChannel> = HashMap::new();
    let mut matched: HashSet<usize> = HashSet::new();
    let mut message_count: u64 = 0;
    let mut skipped: u64 = 0;

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

        // Skip an entry we cannot decode, and say so, rather than failing the
        // whole export: `InterDaemonEvent` is `#[non_exhaustive]` and postcard
        // rejects an unknown variant index outright, so one event added by a
        // newer 1.x daemon makes its recordings unexportable by an older
        // binary, and a single corrupt record costs the user every other
        // message in the file. `dora replay` already resolves this the same way
        // (`replay-node/src/main.rs`). The count is reported below, and a pass
        // that decoded nothing usable still fails, so systematic format drift
        // cannot pass for an empty recording.
        let timestamped = match Timestamped::deserialize_inter_daemon_event(&entry.event_bytes) {
            Ok(timestamped) => timestamped,
            Err(err) => {
                eprintln!(
                    "warning: skipping undecodable event for {}/{}: {err}",
                    entry.node_id, entry.output_id
                );
                skipped += 1;
                continue;
            }
        };
        let (metadata, publish_data) = match timestamped.inner {
            InterDaemonEvent::Output { metadata, data, .. } => (metadata, data),
            _ => continue,
        };
        let publish_time = metadata.timestamp().get_time().to_duration().as_nanos() as u64;

        let topic = format!("{}/{}", entry.node_id, entry.output_id);

        // With `--schema foxglove`, a topic's first message decides whether it
        // is an image topic (JSON channel) or stays Arrow. On an image topic,
        // a message that cannot be converted is dropped and reported: it
        // cannot go on a JSON channel as Arrow.
        let existing = channels.get(&topic).map(|channel| channel.foxglove);
        let converted = match (&foxglove, existing) {
            (Some(mapper), None | Some(Some(_))) => mapper.convert(
                &entry.node_id,
                &entry.output_id,
                &metadata,
                publish_data.as_deref(),
            ),
            _ => foxglove::Recognition::NotRecognised(None),
        };
        let json = match converted {
            foxglove::Recognition::Converted(kind, json) => match existing {
                Some(Some(channel_kind)) if channel_kind != kind => {
                    let reason = format!(
                        "message is a {}, not a {}",
                        kind.schema_name(),
                        channel_kind.schema_name()
                    );
                    drop_message(&mut dropped, &topic, reason);
                    continue;
                }
                _ => Some((kind, json)),
            },
            foxglove::Recognition::Failed(kind, reason) => {
                let reason = format!("{}: {reason}", kind.schema_name());
                drop_message(&mut dropped, &topic, reason);
                continue;
            }
            foxglove::Recognition::NotRecognised(why) => match existing {
                Some(Some(channel_kind)) => {
                    let reason = match why {
                        Some(why) => format!("not a {}: {why}", channel_kind.schema_name()),
                        None => format!("message is not a {}", channel_kind.schema_name()),
                    };
                    drop_message(&mut dropped, &topic, reason);
                    continue;
                }
                _ => None,
            },
        };

        let (channel_id, sequence) = match channels.get_mut(&topic) {
            Some(channel) => {
                channel.sequence = channel.sequence.checked_add(1).ok_or_else(|| {
                    eyre!("recording has more than 2^32 messages on one topic `{topic}`")
                })?;
                (channel.id, channel.sequence)
            }
            None => {
                let (schema_id, encoding, kind) = match &json {
                    Some((kind, _)) => {
                        let schema_id = match foxglove_schemas.get(kind) {
                            Some(id) => *id,
                            None => {
                                let id = writer
                                    .add_schema(
                                        kind.schema_name(),
                                        "jsonschema",
                                        kind.json_schema().as_bytes(),
                                    )
                                    .wrap_err_with(|| {
                                        eyre!("failed to add MCAP schema `{}`", kind.schema_name())
                                    })?;
                                foxglove_schemas.insert(*kind, id);
                                id
                            }
                        };
                        (schema_id, "json", Some(*kind))
                    }
                    None => (0, "arrow-ipc", None),
                };
                let id = writer
                    .add_channel(schema_id, &topic, encoding, &BTreeMap::new())
                    .wrap_err_with(|| eyre!("failed to add MCAP channel `{topic}`"))?;
                channels.insert(
                    topic.clone(),
                    TopicChannel {
                        id,
                        sequence: 1,
                        foxglove: kind,
                    },
                );
                (id, 1)
            }
        };

        // A metadata-only message is recorded as `data: None` but replays as a
        // zero-length `NullArray`, so it has to be written as that IPC stream:
        // a zero-byte message is not a decodable Arrow stream, and would fail
        // every reader on the channel.
        let data = match (&json, publish_data.as_deref()) {
            (Some((_, json)), _) => json.as_slice(),
            (None, Some(data)) => data,
            (None, None) => empty_ipc.as_slice(),
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

    for (topic, (count, reason)) in &dropped {
        eprintln!(
            "warning: dropped {count} message(s) on `{topic}` that could not be converted (first: {reason})"
        );
    }

    for (node, output_id) in unmatched_topics(filter, &matched) {
        eprintln!(
            "warning: --topics entry `{node}/{output_id}` matched no messages in the recording"
        );
    }

    if skipped > 0 {
        eprintln!("Skipped {skipped} undecodable record(s)");
        // Skipping individual records keeps a mostly-good recording exportable,
        // but a *systematically* undecodable one (format drift, truncation) must
        // not pass for a recording that had nothing in it. Same rule, same
        // reason as `dora replay`'s `replay_emitted_nothing_usable`.
        if message_count == 0 {
            eyre::bail!(
                "export emitted nothing: all {skipped} record(s) were undecodable \
                 (corrupt or format-drifted recording)"
            );
        }
    }

    Ok(message_count)
}

/// An MCAP channel added for a recorded topic.
struct TopicChannel {
    id: u16,
    /// The last sequence number written on the channel.
    sequence: u32,
    /// The Foxglove schema of the channel's messages; `None` for an Arrow channel.
    foxglove: Option<foxglove::Kind>,
}

fn drop_message(dropped: &mut BTreeMap<String, (u64, String)>, topic: &str, reason: String) {
    dropped
        .entry(topic.to_owned())
        .and_modify(|(count, _)| *count += 1)
        .or_insert((1, reason));
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

    use super::{
        Export, ExportSchema, default_output_path, run_export, unmatched_topics, write_mcap,
    };
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
            schema: ExportSchema::Arrow,
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
            schema: ExportSchema::Arrow,
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
    fn parse_topic_filter_drops_repeated_topics() {
        let topics = ["a/b", "x/y", "a/b"].map(String::from);
        let filter = super::parse_topic_filter(&topics).expect("parse");
        assert_eq!(
            filter,
            vec![
                ("a".to_string(), "b".to_string()),
                ("x".to_string(), "y".to_string())
            ]
        );
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
            schema: ExportSchema::Arrow,
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
            schema: ExportSchema::Arrow,
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
            schema: ExportSchema::Arrow,
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
            schema: ExportSchema::Arrow,
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
            schema: ExportSchema::Arrow,
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
            schema: ExportSchema::Arrow,
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
            schema: ExportSchema::Arrow,
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
            schema: ExportSchema::Arrow,
        })
        .expect("run export");

        assert!(
            dir.path().join("input.mcap").exists(),
            "default output replaces the input extension, it does not append it"
        );
    }

    /// One undecodable record must not cost the user the rest of the file.
    ///
    /// `InterDaemonEvent` is `#[non_exhaustive]` and postcard rejects an unknown
    /// variant index outright, so this is not only a corrupt-file case: a
    /// recording written by a 1.x daemon that added an event cannot be exported
    /// at all by an older binary if we abort on the first one. `dora replay`
    /// skips the same way.
    #[test]
    fn an_undecodable_record_is_skipped_and_the_rest_still_exports() {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("mixed.drec");
        let mcap_path = dir.path().join("output.mcap");
        {
            let file = fs::File::create(&recording_path).expect("create recording");
            let mut writer =
                RecordingWriter::new(BufWriter::new(file), &header()).expect("init writer");
            let entries: [(u64, Vec<u8>); 3] = [
                (100, output_event_bytes("camera", "image", b"first")),
                // Stands in for a variant a newer daemon added: postcard cannot
                // decode a tag it does not know.
                (200, vec![0xff; 16]),
                (300, output_event_bytes("camera", "image", b"second")),
            ];
            for (offset, event_bytes) in entries {
                writer
                    .write_entry(&RecordEntry {
                        node_id: "camera".to_string(),
                        output_id: "image".to_string(),
                        timestamp_offset_nanos: offset,
                        event_bytes,
                    })
                    .expect("write entry");
            }
            writer.finish().expect("finish recording");
        }

        run_export(Export {
            input: recording_path.to_string_lossy().into(),
            output: Some(mcap_path.to_string_lossy().into()),
            topics: vec![],
            schema: ExportSchema::Arrow,
        })
        .expect("one bad record must not fail the whole export");

        let mcap_bytes = fs::read(&mcap_path).expect("read mcap");
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("read mcap")
            .map(|result| result.expect("read message"))
            .collect();
        assert_eq!(
            messages.len(),
            2,
            "both decodable messages must survive the skipped one"
        );
        assert_eq!(messages[0].data.as_ref(), &b"first"[..]);
        assert_eq!(messages[1].data.as_ref(), &b"second"[..]);
        // The skipped entry must not leave a hole in the sequence numbering.
        assert_eq!(messages[0].sequence, 1);
        assert_eq!(
            messages[1].sequence, 2,
            "sequence counts messages, not records"
        );
    }

    /// …but a recording of *nothing but* undecodable records is a failure, not
    /// an empty export. Skipping must not turn format drift into a file that
    /// looks like a recording with no messages in it.
    #[test]
    fn a_recording_of_only_undecodable_records_fails() {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("alldrift.drec");
        let mcap_path = dir.path().join("output.mcap");
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

        let err = run_export(Export {
            input: recording_path.to_string_lossy().into(),
            output: Some(mcap_path.to_string_lossy().into()),
            topics: vec![],
            schema: ExportSchema::Arrow,
        })
        .expect_err("an export that emitted nothing must not report success");

        assert!(
            err.to_string().contains("undecodable"),
            "the error must name the cause: {err}"
        );
        assert!(
            !mcap_path.exists(),
            "a failed export must leave no output behind"
        );
    }

    /// A failed export must not leave a half-written file where a good one
    /// used to be, and must not leave its temp file behind either.
    #[test]
    fn failed_export_keeps_the_previous_output_and_cleans_up() {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("garbled.drec");
        let mcap_path = dir.path().join("output.mcap");
        // A recording with one good entry, then a record claiming more bytes
        // than a recording may hold: the header parses, the temp file is
        // created, the first message goes out, and the read then fails.
        //
        // The failure has to come from the reader, not from an undecodable
        // event: those are skipped on purpose now, so a newer daemon's variant
        // cannot make a whole recording unexportable. It also cannot be a torn
        // tail, because `read_next_record` treats *every* short read as a clean
        // EOF on purpose (`libraries/recording/src/lib.rs`) — a crashed
        // recording must still export the records that were fully written. A
        // record over `MAX_RECORD_BYTES` is the failure that survives that rule,
        // and it is a real one: it means the length prefix is not what the
        // writer would have written.
        {
            let file = fs::File::create(&recording_path).expect("create recording");
            let mut writer =
                RecordingWriter::new(BufWriter::new(file), &header()).expect("init writer");
            writer
                .write_entry(&RecordEntry {
                    node_id: "camera".to_string(),
                    output_id: "image".to_string(),
                    timestamp_offset_nanos: 100,
                    event_bytes: output_event_bytes("camera", "image", b"good-payload"),
                })
                .expect("write entry");
            writer.finish().expect("finish recording");
            // Splice in a length prefix past the per-record cap, so the reader
            // rejects it after the first message is already in the temp file.
            let mut full = fs::read(&recording_path).expect("read recording");
            let at = full.len() - 24; // start of the 8-byte magic + two u64s
            // Anything over the cap is rejected; one byte past it also stays
            // clear of the footer magic's first four bytes, which the reader
            // would take as a clean end of the recording instead.
            let over_cap = (dora_recording::MAX_RECORD_BYTES + 1) as u32;
            full.splice(at..at, over_cap.to_le_bytes());
            fs::write(&recording_path, &full).expect("write oversized record prefix");
        }
        fs::write(&mcap_path, b"previous good export").expect("seed output");

        let err = run_export(Export {
            input: recording_path.to_string_lossy().into(),
            output: Some(mcap_path.to_string_lossy().into()),
            topics: vec![],
            schema: ExportSchema::Arrow,
        })
        .expect_err("an unreadable record must fail the export");

        assert!(
            err.to_string().contains("read a recording entry"),
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

    /// Pins the guarantee the write-then-rename rests on. `finish` writes the
    /// footer *and then* flushes (`write_summary_and_footer_magic` ends in
    /// `writer.flush()?`), so a flush error comes back as `Err` and fails the
    /// export before the rename; both `Writer`'s and `BufWriter`'s `Drop` would
    /// otherwise throw it away. Nothing in this repo states that, so the test
    /// hands the export a stream that fails only at that flush and requires it
    /// to fail — a `mcap` that moves the flush makes the test go red, which is
    /// where `into_inner().into_inner()` would earn its place (#3541 review).
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
            ExportSchema::Arrow,
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

    // ---- `--schema foxglove` ----

    /// An Arrow IPC stream holding `array` as its single column, the way dora
    /// sends an output.
    fn ipc_bytes(array: arrow::array::ArrayRef) -> Vec<u8> {
        use arrow::ipc::writer::StreamWriter;
        use arrow::record_batch::RecordBatch;
        use arrow_schema::{Field, Schema};
        let schema = std::sync::Arc::new(Schema::new(vec![Field::new(
            "data",
            array.data_type().clone(),
            true,
        )]));
        let batch = RecordBatch::try_new(schema.clone(), vec![array]).expect("batch");
        let mut buf = Vec::new();
        let mut writer = StreamWriter::try_new(&mut buf, &schema).expect("ipc writer");
        writer.write(&batch).expect("write batch");
        writer.finish().expect("finish ipc");
        drop(writer);
        buf
    }

    fn output_event_bytes_with_params(
        node_id: &str,
        output_id: &str,
        payload: &[u8],
        params: &[(&str, dora_message::metadata::Parameter)],
    ) -> Vec<u8> {
        let mut metadata = Metadata::new(sample_timestamp());
        for (key, value) in params {
            metadata.parameters.insert(key.to_string(), value.clone());
        }
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
            timestamp: sample_timestamp(),
        }
        .serialize()
        .expect("serialize output event")
    }

    /// Records `events` (node, output, event bytes) under a dataflow
    /// `descriptor`, exports them with `--schema foxglove` and returns the
    /// MCAP bytes.
    fn export_foxglove(descriptor: &str, events: Vec<(&str, &str, Vec<u8>)>) -> Vec<u8> {
        let dir = tempdir().expect("tempdir");
        let recording_path = dir.path().join("input.drec");
        let mcap_path = dir.path().join("output.mcap");
        let header = RecordingHeader {
            descriptor_yaml: descriptor.as_bytes().to_vec(),
            ..header()
        };
        {
            let file = fs::File::create(&recording_path).expect("create recording");
            let mut writer =
                RecordingWriter::new(BufWriter::new(file), &header).expect("init writer");
            for (offset, (node_id, output_id, event_bytes)) in events.into_iter().enumerate() {
                writer
                    .write_entry(&RecordEntry {
                        node_id: node_id.to_string(),
                        output_id: output_id.to_string(),
                        timestamp_offset_nanos: offset as u64,
                        event_bytes,
                    })
                    .expect("write entry");
            }
            writer.finish().expect("finish recording");
        }
        run_export(Export {
            input: recording_path.to_string_lossy().into(),
            output: Some(mcap_path.to_string_lossy().into()),
            topics: vec![],
            schema: ExportSchema::Foxglove,
        })
        .expect("run export");
        fs::read(&mcap_path).expect("read mcap")
    }

    fn json_message(message: &mcap::Message<'_>) -> serde_json::Value {
        serde_json::from_slice(&message.data).expect("message is JSON")
    }

    fn rgb_image_struct(width: u32, height: u32, data: &[u8]) -> arrow::array::ArrayRef {
        use arrow::array::{LargeBinaryArray, StringArray, StructArray, UInt32Array};
        use arrow_schema::{DataType, Field};
        std::sync::Arc::new(StructArray::from(vec![
            (
                std::sync::Arc::new(Field::new("width", DataType::UInt32, false)),
                std::sync::Arc::new(UInt32Array::from(vec![width])) as arrow::array::ArrayRef,
            ),
            (
                std::sync::Arc::new(Field::new("height", DataType::UInt32, false)),
                std::sync::Arc::new(UInt32Array::from(vec![height])) as arrow::array::ArrayRef,
            ),
            (
                std::sync::Arc::new(Field::new("encoding", DataType::Utf8, false)),
                std::sync::Arc::new(StringArray::from(vec!["rgb8"])) as arrow::array::ArrayRef,
            ),
            (
                std::sync::Arc::new(Field::new("data", DataType::LargeBinary, false)),
                std::sync::Arc::new(LargeBinaryArray::from(vec![data])) as arrow::array::ArrayRef,
            ),
        ]))
    }

    const CAMERA_DESCRIPTOR: &str = "nodes:
  - id: camera
    outputs: [image, jpeg, png, temperature]
    output_types:
      image: std/media/v1/Image
      jpeg: std/media/v1/CompressedImage[format=jpeg]
      png: std/media/v1/CompressedImage
";

    #[test]
    fn foxglove_exports_a_declared_image_as_raw_image() {
        use base64::Engine as _;
        use dora_message::metadata::Parameter;
        let pixels: Vec<u8> = (0..12).collect();
        let mcap_bytes = export_foxglove(
            CAMERA_DESCRIPTOR,
            vec![(
                "camera",
                "image",
                output_event_bytes_with_params(
                    "camera",
                    "image",
                    &ipc_bytes(rgb_image_struct(2, 2, &pixels)),
                    &[("frame_id", Parameter::String("camera_optical".into()))],
                ),
            )],
        );
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read messages");
        assert_eq!(messages.len(), 1);
        let channel = &messages[0].channel;
        assert_eq!(channel.topic, "camera/image");
        assert_eq!(channel.message_encoding, "json");
        let schema = channel.schema.as_ref().expect("json channel has a schema");
        assert_eq!(schema.name, "foxglove.RawImage");
        assert_eq!(schema.encoding, "jsonschema");

        let json = json_message(&messages[0]);
        let publish = sample_timestamp().get_time().to_duration();
        assert_eq!(json["timestamp"]["sec"], publish.as_secs());
        assert_eq!(json["timestamp"]["nsec"], publish.subsec_nanos());
        assert_eq!(json["frame_id"], "camera_optical");
        assert_eq!(json["width"], 2);
        assert_eq!(json["height"], 2);
        assert_eq!(json["encoding"], "rgb8");
        assert_eq!(json["step"], 6);
        assert_eq!(
            json["data"],
            base64::engine::general_purpose::STANDARD.encode(&pixels)
        );
    }

    #[test]
    fn foxglove_exports_a_metadata_described_camera_frame_as_raw_image() {
        use dora_message::metadata::Parameter;
        let frame = arrow::array::UInt8Array::from((0..24).collect::<Vec<u8>>());
        let mcap_bytes = export_foxglove(
            "nodes: []",
            vec![(
                "webcam",
                "frame",
                output_event_bytes_with_params(
                    "webcam",
                    "frame",
                    &ipc_bytes(std::sync::Arc::new(frame)),
                    &[
                        ("width", Parameter::Integer(4)),
                        ("height", Parameter::Integer(2)),
                        ("encoding", Parameter::String("bgr8".into())),
                    ],
                ),
            )],
        );
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read messages");
        assert_eq!(messages.len(), 1);
        assert_eq!(
            messages[0].channel.schema.as_ref().expect("schema").name,
            "foxglove.RawImage"
        );
        let json = json_message(&messages[0]);
        assert_eq!(json["width"], 4);
        assert_eq!(json["height"], 2);
        assert_eq!(json["encoding"], "bgr8");
        assert_eq!(json["step"], 12);
        assert_eq!(json["frame_id"], "");
    }

    #[test]
    fn foxglove_exports_compressed_images_with_declared_or_detected_format() {
        use arrow::array::LargeBinaryArray;
        let jpeg = [0xFF, 0xD8, 0xFF, 0xE0, 1, 2, 3];
        let png = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A, 9];
        let mcap_bytes = export_foxglove(
            CAMERA_DESCRIPTOR,
            vec![
                (
                    "camera",
                    "jpeg",
                    output_event_bytes(
                        "camera",
                        "jpeg",
                        &ipc_bytes(std::sync::Arc::new(LargeBinaryArray::from(vec![&jpeg[..]]))),
                    ),
                ),
                (
                    "camera",
                    "png",
                    output_event_bytes(
                        "camera",
                        "png",
                        &ipc_bytes(std::sync::Arc::new(LargeBinaryArray::from(vec![&png[..]]))),
                    ),
                ),
            ],
        );
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read messages");
        assert_eq!(messages.len(), 2);
        for (message, format) in messages.iter().zip(["jpeg", "png"]) {
            assert_eq!(
                message.channel.schema.as_ref().expect("schema").name,
                "foxglove.CompressedImage"
            );
            assert_eq!(json_message(message)["format"], format);
        }
    }

    #[test]
    fn foxglove_keeps_unrecognised_outputs_as_arrow() {
        let payload = ipc_bytes(std::sync::Arc::new(arrow::array::Float64Array::from(vec![
            21.5,
        ])));
        let mcap_bytes = export_foxglove(
            CAMERA_DESCRIPTOR,
            vec![(
                "camera",
                "temperature",
                output_event_bytes("camera", "temperature", &payload),
            )],
        );
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read messages");
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].channel.message_encoding, "arrow-ipc");
        assert_eq!(messages[0].data.as_ref(), &payload[..]);
    }

    #[test]
    fn foxglove_drops_an_image_it_cannot_convert_and_keeps_going() {
        let good = ipc_bytes(rgb_image_struct(2, 2, &[0; 12]));
        // 7 bytes cannot be 2 equal rows.
        let bad = ipc_bytes(rgb_image_struct(2, 2, &[0; 7]));
        let mcap_bytes = export_foxglove(
            CAMERA_DESCRIPTOR,
            vec![
                (
                    "camera",
                    "image",
                    output_event_bytes("camera", "image", &good),
                ),
                (
                    "camera",
                    "image",
                    output_event_bytes("camera", "image", &bad),
                ),
                (
                    "camera",
                    "image",
                    output_event_bytes("camera", "image", &good),
                ),
            ],
        );
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read messages");
        assert_eq!(messages.len(), 2, "the unconvertible frame is dropped");
        assert!(
            messages
                .iter()
                .all(|m| m.channel.message_encoding == "json")
        );
        assert_eq!(messages[1].sequence, 2, "sequence counts written messages");
    }

    #[test]
    fn foxglove_compressed_format_from_metadata_signature_or_short_name() {
        use arrow::array::LargeBinaryArray;
        use dora_message::metadata::Parameter;
        let descriptor = "nodes:
  - id: cam
    output_types:
      a: std/media/v1/CompressedImage
      b: std/media/v1/CompressedImage[format=JPG]
      c: std/media/v1/CompressedImage
      d: std/media/v1/CompressedImage
      e: std/media/v1/CompressedImage[format=jpeg]
";
        let webp = [
            b'R', b'I', b'F', b'F', 0, 0, 0, 0, b'W', b'E', b'B', b'P', 1,
        ];
        let unknown = [1, 2, 3, 4];
        let jpeg = [0xFF, 0xD8, 0xFF, 0xE0, 1, 2];
        let png = [0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A, 1];
        let payload =
            |bytes: &[u8]| ipc_bytes(std::sync::Arc::new(LargeBinaryArray::from(vec![bytes])));
        let mcap_bytes = export_foxglove(
            descriptor,
            vec![
                ("cam", "a", output_event_bytes("cam", "a", &payload(&webp))),
                ("cam", "b", output_event_bytes("cam", "b", &payload(&jpeg))),
                (
                    "cam",
                    "c",
                    output_event_bytes_with_params(
                        "cam",
                        "c",
                        &payload(&png),
                        &[("format", Parameter::String("png".into()))],
                    ),
                ),
                // Declared JPEG, but the data is a PNG.
                ("cam", "e", output_event_bytes("cam", "e", &payload(&png))),
                // No declared format, no metadata, no known signature.
                (
                    "cam",
                    "d",
                    output_event_bytes("cam", "d", &payload(&unknown)),
                ),
            ],
        );
        let messages: Vec<_> = MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .collect::<Result<Vec<_>, _>>()
            .expect("read messages");
        let formats: Vec<_> = messages
            .iter()
            .map(|m| (m.channel.topic.clone(), json_message(m)["format"].clone()))
            .collect();
        assert_eq!(
            formats,
            vec![
                ("cam/a".to_string(), serde_json::json!("webp")),
                ("cam/b".to_string(), serde_json::json!("jpeg")),
                ("cam/c".to_string(), serde_json::json!("png")),
            ],
            "cam/d (no known format) and cam/e (not the declared format) are dropped"
        );
    }

    /// Rows too short for the encoding: an undeclared output is then not
    /// treated as an image (stays Arrow); a declared one is dropped and
    /// reported.
    #[test]
    fn foxglove_rows_too_short_for_the_encoding() {
        // 4 bytes per row cannot hold 4 rgb8 pixels.
        let frame = || {
            ipc_bytes(std::sync::Arc::new(arrow::array::UInt8Array::from(vec![
                0u8;
                8
            ])))
        };
        let undeclared = export_messages(
            "nodes: []",
            vec![(
                "webcam",
                "frame",
                output_event_bytes_with_params(
                    "webcam",
                    "frame",
                    &frame(),
                    &camera_params(4, 2, "rgb8"),
                ),
            )],
        );
        assert_eq!(undeclared.len(), 1);
        assert_eq!(undeclared[0].1, "arrow-ipc");

        let declared = export_messages(
            CAMERA_DESCRIPTOR,
            vec![(
                "camera",
                "image",
                output_event_bytes_with_params(
                    "camera",
                    "image",
                    &frame(),
                    &camera_params(4, 2, "rgb8"),
                ),
            )],
        );
        assert!(
            declared.is_empty(),
            "a declared image that can't convert is dropped"
        );
    }

    // ---- real producers: what the dora-hub camera nodes actually send ----

    fn export_messages(
        descriptor: &str,
        events: Vec<(&str, &str, Vec<u8>)>,
    ) -> Vec<(String, String, Option<String>, Vec<u8>)> {
        let mcap_bytes = export_foxglove(descriptor, events);
        MessageStream::new(&mcap_bytes)
            .expect("open mcap")
            .map(|m| {
                let m = m.expect("message");
                (
                    m.channel.topic.clone(),
                    m.channel.message_encoding.clone(),
                    m.channel.schema.as_ref().map(|s| s.name.clone()),
                    m.data.to_vec(),
                )
            })
            .collect()
    }

    fn camera_params(
        width: i64,
        height: i64,
        encoding: &str,
    ) -> Vec<(&'static str, dora_message::metadata::Parameter)> {
        use dora_message::metadata::Parameter;
        vec![
            ("width", Parameter::Integer(width)),
            ("height", Parameter::Integer(height)),
            ("encoding", Parameter::String(encoding.into())),
        ]
    }

    /// `opencv-video-capture` with `ENCODING=jpeg`: a flat uint8 JPEG plus
    /// width/height/encoding metadata.
    #[test]
    fn foxglove_camera_jpeg_frames_become_compressed_images() {
        let jpeg: Vec<u8> = vec![0xFF, 0xD8, 0xFF, 0xE0, 1, 2, 3];
        let messages = export_messages(
            "nodes: []",
            vec![(
                "camera",
                "image",
                output_event_bytes_with_params(
                    "camera",
                    "image",
                    &ipc_bytes(std::sync::Arc::new(arrow::array::UInt8Array::from(jpeg))),
                    &camera_params(640, 480, "jpeg"),
                ),
            )],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].2.as_deref(), Some("foxglove.CompressedImage"));
        let json: serde_json::Value = serde_json::from_slice(&messages[0].3).unwrap();
        assert_eq!(json["format"], "jpeg");
    }

    /// Python nodes often forward their input's metadata
    /// (`node.send_output(id, arr, event["metadata"])`), so a node behind a
    /// `jpeg` camera sends non-image outputs with `width`/`height`/
    /// `encoding=jpeg` too. Undeclared, such an output must stay Arrow unless
    /// its bytes really are a JPEG.
    #[test]
    fn foxglove_forwarded_camera_metadata_does_not_make_an_image() {
        let scores = ipc_bytes(std::sync::Arc::new(arrow::array::Float32Array::from(vec![
            0.5f32, 0.25, 0.125,
        ])));
        let not_jpeg = ipc_bytes(std::sync::Arc::new(arrow::array::UInt8Array::from(vec![
            1u8, 2, 3, 4,
        ])));
        let messages = export_messages(
            "nodes: []",
            vec![
                (
                    "detector",
                    "scores",
                    output_event_bytes_with_params(
                        "detector",
                        "scores",
                        &scores,
                        &camera_params(640, 480, "jpeg"),
                    ),
                ),
                (
                    "detector",
                    "labels",
                    output_event_bytes_with_params(
                        "detector",
                        "labels",
                        &not_jpeg,
                        &camera_params(640, 480, "jpeg"),
                    ),
                ),
            ],
        );
        assert_eq!(messages.len(), 2);
        for (topic, encoding, schema, data) in &messages {
            assert_eq!(encoding, "arrow-ipc", "{topic}");
            assert_eq!(schema, &None, "{topic}");
            let expected = if topic == "detector/scores" {
                &scores
            } else {
                &not_jpeg
            };
            assert_eq!(data, expected, "{topic} must be exported unchanged");
        }
    }

    /// The raw-encoding side of forwarded camera metadata: a same-size float32
    /// map behind an `rgb8` camera must not become an `rgb8` RawImage, and an
    /// undeclared frame whose rows don't hold exactly `width` pixels isn't
    /// trusted to be one either. Real frames of the right type still convert.
    #[test]
    fn foxglove_undeclared_raw_frames_must_match_their_encoding() {
        use arrow::array::{Float32Array, UInt8Array};
        let float_map = ipc_bytes(std::sync::Arc::new(Float32Array::from(vec![0.5f32; 8 * 4])));
        // 2x2 rgb8 is 12 bytes; 16 bytes means rows of 8: not a plain frame.
        let padded = ipc_bytes(std::sync::Arc::new(UInt8Array::from(vec![7u8; 16])));
        let depth = ipc_bytes(std::sync::Arc::new(Float32Array::from(vec![1.5f32; 4])));
        let messages = export_messages(
            "nodes: []",
            vec![
                (
                    "depth",
                    "map",
                    output_event_bytes_with_params(
                        "depth",
                        "map",
                        &float_map,
                        &camera_params(8, 4, "rgb8"),
                    ),
                ),
                (
                    "filter",
                    "rgb",
                    output_event_bytes_with_params(
                        "filter",
                        "rgb",
                        &padded,
                        &camera_params(2, 2, "rgb8"),
                    ),
                ),
                (
                    "stereo",
                    "depth",
                    output_event_bytes_with_params(
                        "stereo",
                        "depth",
                        &depth,
                        &camera_params(2, 2, "32FC1"),
                    ),
                ),
            ],
        );
        assert_eq!(messages.len(), 3);
        let by_topic = |topic: &str| {
            messages
                .iter()
                .find(|message| message.0 == topic)
                .unwrap_or_else(|| panic!("no {topic}"))
        };
        for (topic, payload) in [("depth/map", &float_map), ("filter/rgb", &padded)] {
            let (_, encoding, schema, data) = by_topic(topic);
            assert_eq!(encoding, "arrow-ipc", "{topic}");
            assert_eq!(schema, &None, "{topic}");
            assert_eq!(data, payload, "{topic} must stay unchanged");
        }
        let (_, _, schema, data) = by_topic("stereo/depth");
        assert_eq!(schema.as_deref(), Some("foxglove.RawImage"));
        let json: serde_json::Value = serde_json::from_slice(data).unwrap();
        assert_eq!(json["encoding"], "32FC1");
        assert_eq!(json["step"], 8);
    }

    /// A declared image whose payload doesn't fit its encoding is reported and
    /// dropped, not exported as a garbage image; padded rows are fine there.
    #[test]
    fn foxglove_declared_frames_check_their_pixel_type() {
        use arrow::array::{Float32Array, UInt8Array};
        let float_frame = ipc_bytes(std::sync::Arc::new(Float32Array::from(vec![0.5f32; 4 * 3])));
        let padded = ipc_bytes(std::sync::Arc::new(UInt8Array::from(vec![7u8; 16])));
        let messages = export_messages(
            CAMERA_DESCRIPTOR,
            vec![
                (
                    "camera",
                    "image",
                    output_event_bytes_with_params(
                        "camera",
                        "image",
                        &padded,
                        &camera_params(2, 2, "rgb8"),
                    ),
                ),
                (
                    "camera",
                    "image",
                    output_event_bytes_with_params(
                        "camera",
                        "image",
                        &float_frame,
                        &camera_params(4, 1, "rgb8"),
                    ),
                ),
            ],
        );
        assert_eq!(messages.len(), 1, "the float32 frame is dropped");
        let json: serde_json::Value = serde_json::from_slice(&messages[0].3).unwrap();
        assert_eq!(json["step"], 8, "declared rows may be padded");
    }

    fn jpeg_bytes() -> Vec<u8> {
        let mut jpeg = vec![0xFFu8, 0xD8, 0xFF, 0xE0];
        jpeg.extend([7u8; 60]);
        jpeg
    }

    /// The declared type decides: `Image[encoding=jpeg]` stays a JPEG even if
    /// the node also forwards raw camera metadata, a `CompressedImage` with a
    /// raw `encoding` in its metadata uses the file signature, and an output
    /// declared as something other than an image is never guessed at.
    #[test]
    fn foxglove_declared_type_wins_over_forwarded_metadata() {
        use arrow::array::{Float32Array, LargeBinaryArray, UInt8Array};
        let descriptor = "nodes:
  - id: enc
    output_types:
      jpeg: std/media/v1/Image[encoding=jpeg]
      cjpeg: std/media/v1/CompressedImage
      classes: std/core/v1/UInt8
      conf: std/core/v1/Float32
";
        let classes = ipc_bytes(std::sync::Arc::new(UInt8Array::from(vec![1u8; 8])));
        let conf = ipc_bytes(std::sync::Arc::new(Float32Array::from(vec![0.5f32; 8])));
        let event =
            |output: &str, payload: &[u8], params: &[(&str, dora_message::metadata::Parameter)]| {
                (
                    "enc",
                    output.to_owned(),
                    output_event_bytes_with_params("enc", output, payload, params),
                )
            };
        let events = [
            event(
                "jpeg",
                &ipc_bytes(std::sync::Arc::new(UInt8Array::from(jpeg_bytes()))),
                &camera_params(640, 480, "bgr8"),
            ),
            event(
                "cjpeg",
                &ipc_bytes(std::sync::Arc::new(LargeBinaryArray::from(vec![
                    jpeg_bytes().as_slice(),
                ]))),
                &camera_params(640, 480, "bgr8"),
            ),
            event("classes", &classes, &camera_params(4, 2, "mono8")),
            event("conf", &conf, &camera_params(4, 2, "32FC1")),
        ];
        let messages = export_messages(
            descriptor,
            events
                .iter()
                .map(|(node, output, bytes)| (*node, output.as_str(), bytes.clone()))
                .collect(),
        );
        let schema = |topic: &str| {
            messages
                .iter()
                .find(|message| message.0 == topic)
                .unwrap_or_else(|| panic!("{topic} was dropped"))
                .2
                .clone()
        };
        assert_eq!(
            schema("enc/jpeg").as_deref(),
            Some("foxglove.CompressedImage")
        );
        assert_eq!(
            schema("enc/cjpeg").as_deref(),
            Some("foxglove.CompressedImage")
        );
        assert_eq!(schema("enc/classes"), None);
        assert_eq!(schema("enc/conf"), None);
    }

    /// Frames Foxglove can't display, or that can't be an image, are not
    /// exported as images: zero width, odd sizes where NV12 / YUV 4:2:2 need
    /// even ones, and compressed data that isn't the format it claims.
    #[test]
    fn foxglove_rejects_frames_foxglove_cannot_show() {
        use arrow::array::UInt8Array;
        let bytes = |n: usize| ipc_bytes(std::sync::Arc::new(UInt8Array::from(vec![9u8; n])));
        let undeclared =
            |output: &str,
             n: usize,
             params: Vec<(&'static str, dora_message::metadata::Parameter)>| {
                (
                    output.to_owned(),
                    output_event_bytes_with_params("cam", output, &bytes(n), &params),
                )
            };
        let events = [
            undeclared("zero_width", 0, camera_params(0, 1, "rgb8")),
            undeclared("odd_nv12", 15, camera_params(3, 3, "nv12")),
            undeclared("odd_uyvy", 12, camera_params(3, 2, "uyvy")),
            undeclared("even_nv12", 12, camera_params(4, 2, "nv12")),
        ];
        let messages = export_messages(
            "nodes: []",
            events
                .iter()
                .map(|(output, bytes)| ("cam", output.as_str(), bytes.clone()))
                .collect(),
        );
        let schema = |topic: &str| {
            messages
                .iter()
                .find(|message| message.0 == topic)
                .unwrap_or_else(|| panic!("{topic} was dropped"))
                .2
                .clone()
        };
        assert_eq!(schema("cam/zero_width"), None);
        assert_eq!(schema("cam/odd_nv12"), None);
        assert_eq!(schema("cam/odd_uyvy"), None);
        assert_eq!(
            schema("cam/even_nv12").as_deref(),
            Some("foxglove.RawImage")
        );

        // Declared: raw bytes claimed as JPEG (by the type) or PNG (by the
        // metadata), and a zero-width frame, are dropped.
        let declared = export_messages(
            CAMERA_DESCRIPTOR,
            vec![
                (
                    "camera",
                    "jpeg",
                    output_event_bytes("camera", "jpeg", &bytes(12)),
                ),
                (
                    "camera",
                    "image",
                    output_event_bytes_with_params(
                        "camera",
                        "image",
                        &bytes(12),
                        &camera_params(2, 2, "png"),
                    ),
                ),
                (
                    "camera",
                    "image",
                    output_event_bytes_with_params(
                        "camera",
                        "image",
                        &bytes(12),
                        &camera_params(0, 2, "rgb8"),
                    ),
                ),
            ],
        );
        assert!(declared.is_empty(), "exported {declared:?}");
    }

    /// `dora-pyorbbecksdk` sends JPEGs with only `encoding: jpeg`: no width or
    /// height, which a compressed image doesn't need.
    #[test]
    fn foxglove_compressed_frames_need_only_their_encoding() {
        use arrow::array::UInt8Array;
        use dora_message::metadata::Parameter;
        let jpeg = ipc_bytes(std::sync::Arc::new(UInt8Array::from(jpeg_bytes())));
        let encoding = [("encoding", Parameter::String("jpeg".into()))];
        let undeclared = export_messages(
            "nodes: []",
            vec![(
                "orbbec",
                "image",
                output_event_bytes_with_params("orbbec", "image", &jpeg, &encoding),
            )],
        );
        assert_eq!(undeclared.len(), 1);
        assert_eq!(undeclared[0].2.as_deref(), Some("foxglove.CompressedImage"));
        let declared = export_messages(
            CAMERA_DESCRIPTOR,
            vec![(
                "camera",
                "image",
                output_event_bytes_with_params("camera", "image", &jpeg, &encoding),
            )],
        );
        assert_eq!(declared.len(), 1);
        assert_eq!(declared[0].2.as_deref(), Some("foxglove.CompressedImage"));
    }

    /// A raw encoding on the declared type is used like `[encoding=jpeg]`: it
    /// wins over the metadata, and the metadata then only needs the size.
    #[test]
    fn foxglove_declared_raw_encoding_wins_over_metadata() {
        use arrow::array::UInt8Array;
        use dora_message::metadata::Parameter;
        let descriptor = "nodes:
  - id: cam
    output_types:
      image: std/media/v1/Image[encoding=rgb8]
";
        let frame = ipc_bytes(std::sync::Arc::new(UInt8Array::from(vec![5u8; 12])));
        let size = [
            ("width", Parameter::Integer(2)),
            ("height", Parameter::Integer(2)),
        ];
        let messages = export_messages(
            descriptor,
            vec![
                (
                    "cam",
                    "image",
                    output_event_bytes_with_params("cam", "image", &frame, &size),
                ),
                (
                    "cam",
                    "image",
                    output_event_bytes_with_params(
                        "cam",
                        "image",
                        &frame,
                        &camera_params(2, 2, "bgr8"),
                    ),
                ),
            ],
        );
        assert_eq!(messages.len(), 2);
        for message in &messages {
            let json: serde_json::Value = serde_json::from_slice(&message.3).unwrap();
            assert_eq!(json["encoding"], "rgb8");
        }
    }

    /// What dora's ROS 2 bridge produces for `sensor_msgs/Image`: a struct
    /// with `step`, `is_bigendian` and `data` as a list of uint8.
    #[test]
    fn foxglove_declared_ros_image_struct_converts() {
        use arrow::array::{
            Array, ArrayRef, ListArray, StringArray, StructArray, UInt8Array, UInt32Array,
        };
        use arrow_schema::{DataType, Field};
        use std::sync::Arc;
        let ros_image = |encoding: &str, step: u32, bigendian: u8, data: Vec<u8>| -> ArrayRef {
            let list =
                ListArray::from_iter_primitive::<arrow::datatypes::UInt8Type, _, _>(vec![Some(
                    data.into_iter().map(Some).collect::<Vec<_>>(),
                )]);
            Arc::new(StructArray::from(vec![
                (
                    Arc::new(Field::new("height", DataType::UInt32, false)),
                    Arc::new(UInt32Array::from(vec![2])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("width", DataType::UInt32, false)),
                    Arc::new(UInt32Array::from(vec![2])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("encoding", DataType::Utf8, false)),
                    Arc::new(StringArray::from(vec![encoding])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("is_bigendian", DataType::UInt8, false)),
                    Arc::new(UInt8Array::from(vec![bigendian])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("step", DataType::UInt32, false)),
                    Arc::new(UInt32Array::from(vec![step])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("data", list.data_type().clone(), false)),
                    Arc::new(list) as ArrayRef,
                ),
            ]))
        };
        let descriptor = "nodes:
  - id: ros
    output_types:
      image: std/media/v1/Image
";
        let event = |image: ArrayRef| {
            (
                "ros",
                "image",
                output_event_bytes("ros", "image", &ipc_bytes(image)),
            )
        };
        let messages = export_messages(
            descriptor,
            vec![
                event(ros_image("rgb8", 6, 0, vec![1; 12])),
                // `step` doesn't match the data: dropped.
                event(ros_image("rgb8", 8, 0, vec![1; 12])),
                // Big-endian 16-bit data: dropped.
                event(ros_image("mono16", 4, 1, vec![1; 8])),
            ],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].2.as_deref(), Some("foxglove.RawImage"));
        let json: serde_json::Value = serde_json::from_slice(&messages[0].3).unwrap();
        assert_eq!(json["step"], 6);
        assert_eq!(json["encoding"], "rgb8");
    }

    fn png_bytes() -> Vec<u8> {
        let mut png = vec![0x89u8, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A];
        png.extend([3u8; 40]);
        png
    }

    /// Metadata names "a compressed image"; the file signature says which
    /// format. Only a format on the declared type has to match it exactly.
    #[test]
    fn foxglove_forwarded_compressed_name_does_not_drop_another_format() {
        use arrow::array::{LargeBinaryArray, UInt8Array};
        let png = ipc_bytes(std::sync::Arc::new(UInt8Array::from(png_bytes())));
        let png_binary = ipc_bytes(std::sync::Arc::new(LargeBinaryArray::from(vec![
            png_bytes().as_slice(),
        ])));
        let messages = export_messages(
            CAMERA_DESCRIPTOR,
            vec![
                // declared CompressedImage without a format
                (
                    "camera",
                    "png",
                    output_event_bytes_with_params(
                        "camera",
                        "png",
                        &png_binary,
                        &camera_params(640, 480, "jpeg"),
                    ),
                ),
                // undeclared
                (
                    "reencoder",
                    "image",
                    output_event_bytes_with_params(
                        "reencoder",
                        "image",
                        &png,
                        &camera_params(640, 480, "jpeg"),
                    ),
                ),
                // declared `[format=jpeg]`: has to be a JPEG
                (
                    "camera",
                    "jpeg",
                    output_event_bytes("camera", "jpeg", &png_binary),
                ),
            ],
        );
        let formats: Vec<_> = messages
            .iter()
            .map(|message| {
                let json: serde_json::Value = serde_json::from_slice(&message.3).unwrap();
                (message.0.clone(), json["format"].clone())
            })
            .collect();
        assert_eq!(
            formats,
            vec![
                ("camera/png".to_string(), serde_json::json!("png")),
                ("reencoder/image".to_string(), serde_json::json!("png")),
            ]
        );
    }

    /// A malformed type annotation still declares the output as something:
    /// it is never guessed to be an image.
    #[test]
    fn foxglove_malformed_type_is_not_treated_as_undeclared() {
        use arrow::array::UInt8Array;
        let jpeg = ipc_bytes(std::sync::Arc::new(UInt8Array::from(jpeg_bytes())));
        let messages = export_messages(
            "nodes:\n  - id: n\n    output_types:\n      scores: std/core/v1/UInt8[\n",
            vec![(
                "n",
                "scores",
                output_event_bytes_with_params(
                    "n",
                    "scores",
                    &jpeg,
                    &camera_params(640, 480, "jpeg"),
                ),
            )],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].1, "arrow-ipc");
    }

    /// ROS `sensor_msgs/CompressedImage` (from `image_transport`, via the ROS 2
    /// bridge): a struct with `format` and `data` as a list of uint8. And an
    /// `Image` struct whose encoding contradicts the declared one is dropped.
    #[test]
    fn foxglove_declared_ros_compressed_struct_and_encoding_conflicts() {
        use arrow::array::{Array, ArrayRef, ListArray, StringArray, StructArray, UInt32Array};
        use arrow_schema::Field;
        use std::sync::Arc;
        let list = |bytes: Vec<u8>| -> ArrayRef {
            Arc::new(ListArray::from_iter_primitive::<
                arrow::datatypes::UInt8Type,
                _,
                _,
            >(vec![Some(
                bytes.into_iter().map(Some).collect::<Vec<_>>(),
            )]))
        };
        let field = |name: &str, array: ArrayRef| {
            (
                Arc::new(Field::new(name, array.data_type().clone(), false)),
                array,
            )
        };
        let ros_compressed: ArrayRef = Arc::new(StructArray::from(vec![
            field("format", Arc::new(StringArray::from(vec!["jpeg"]))),
            field("data", list(jpeg_bytes())),
        ]));
        let rgb_struct: ArrayRef = Arc::new(StructArray::from(vec![
            field("width", Arc::new(UInt32Array::from(vec![2]))),
            field("height", Arc::new(UInt32Array::from(vec![2]))),
            field("encoding", Arc::new(StringArray::from(vec!["rgb8"]))),
            field("data", list(vec![1; 12])),
        ]));
        let descriptor = "nodes:
  - id: ros
    output_types:
      compressed: std/media/v1/CompressedImage
      jpeg: std/media/v1/Image[encoding=jpeg]
";
        let messages = export_messages(
            descriptor,
            vec![
                (
                    "ros",
                    "compressed",
                    output_event_bytes("ros", "compressed", &ipc_bytes(ros_compressed)),
                ),
                (
                    "ros",
                    "jpeg",
                    output_event_bytes("ros", "jpeg", &ipc_bytes(rgb_struct)),
                ),
            ],
        );
        assert_eq!(
            messages.len(),
            1,
            "the rgb8 struct on `[encoding=jpeg]` is dropped"
        );
        assert_eq!(messages[0].0, "ros/compressed");
        let json: serde_json::Value = serde_json::from_slice(&messages[0].3).unwrap();
        assert_eq!(json["format"], "jpeg");
    }

    /// YAML numbers are ids too: dora records `1:` as output `1`, so its
    /// declaration must count.
    #[test]
    fn foxglove_reads_numeric_ids_in_the_descriptor() {
        use arrow::array::UInt8Array;
        use dora_message::metadata::Parameter;
        let descriptor = "nodes:
  - id: 7
    outputs: [1]
    output_types:
      1: std/core/v1/Bytes
";
        let jpeg = ipc_bytes(std::sync::Arc::new(UInt8Array::from(jpeg_bytes())));
        let messages = export_messages(
            descriptor,
            vec![(
                "7",
                "1",
                output_event_bytes_with_params(
                    "7",
                    "1",
                    &jpeg,
                    &[("encoding", Parameter::String("jpeg".into()))],
                ),
            )],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].1, "arrow-ipc", "declared as Bytes");
    }

    /// Foxglove lists `8UC3`/`bgr8`, `8UC1`/`mono8`, `16UC1`/`mono16`,
    /// `uyvy`/`yuv422` and `yuyv`/`yuv422_yuy2` as the same encodings.
    #[test]
    fn foxglove_encoding_aliases_are_not_contradictions() {
        use arrow::array::{ArrayRef, LargeBinaryArray, StringArray, StructArray, UInt32Array};
        use arrow_schema::{DataType, Field};
        use std::sync::Arc;
        let image = |encoding: &str, bytes: usize| -> ArrayRef {
            Arc::new(StructArray::from(vec![
                (
                    Arc::new(Field::new("width", DataType::UInt32, false)),
                    Arc::new(UInt32Array::from(vec![2])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("height", DataType::UInt32, false)),
                    Arc::new(UInt32Array::from(vec![2])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("encoding", DataType::Utf8, false)),
                    Arc::new(StringArray::from(vec![encoding])) as ArrayRef,
                ),
                (
                    Arc::new(Field::new("data", DataType::LargeBinary, false)),
                    Arc::new(LargeBinaryArray::from(vec![vec![1u8; bytes].as_slice()])) as ArrayRef,
                ),
            ]))
        };
        let descriptor = "nodes:
  - id: cam
    output_types:
      a: std/media/v1/Image[encoding=8UC3]
      b: std/media/v1/Image[encoding=uyvy]
      c: std/media/v1/Image[encoding=16UC1]
";
        let messages = export_messages(
            descriptor,
            vec![
                (
                    "cam",
                    "a",
                    output_event_bytes("cam", "a", &ipc_bytes(image("bgr8", 12))),
                ),
                (
                    "cam",
                    "b",
                    output_event_bytes("cam", "b", &ipc_bytes(image("yuv422", 8))),
                ),
                (
                    "cam",
                    "c",
                    output_event_bytes("cam", "c", &ipc_bytes(image("mono16", 8))),
                ),
            ],
        );
        assert_eq!(messages.len(), 3, "aliases must not be dropped");
    }

    /// Once an undeclared topic's first frame converted, the channel is a
    /// Foxglove image channel, so a later frame that doesn't convert can't go
    /// on it as Arrow: it is dropped (and reported), not written.
    #[test]
    fn foxglove_drops_a_later_bad_frame_on_an_undeclared_image_topic() {
        let jpeg = || {
            ipc_bytes(std::sync::Arc::new(arrow::array::UInt8Array::from(vec![
                0xFFu8, 0xD8, 0xFF, 0xE0, 1, 2,
            ])))
        };
        let messages = export_messages(
            "nodes: []",
            vec![
                (
                    "camera",
                    "image",
                    output_event_bytes_with_params(
                        "camera",
                        "image",
                        &jpeg(),
                        &camera_params(640, 480, "jpeg"),
                    ),
                ),
                (
                    "camera",
                    "image",
                    output_event_bytes("camera", "image", &jpeg()),
                ),
            ],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].2.as_deref(), Some("foxglove.CompressedImage"));
    }

    /// `dora-pyrealsense` depth: a uint16 array with `encoding: mono16`.
    #[test]
    fn foxglove_mono16_depth_frames_become_raw_images() {
        use base64::Engine as _;
        let depth = arrow::array::UInt16Array::from(vec![1u16, 2, 3, 0x0102]);
        let messages = export_messages(
            "nodes: []",
            vec![(
                "realsense",
                "depth",
                output_event_bytes_with_params(
                    "realsense",
                    "depth",
                    &ipc_bytes(std::sync::Arc::new(depth)),
                    &camera_params(2, 2, "mono16"),
                ),
            )],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].2.as_deref(), Some("foxglove.RawImage"));
        let json: serde_json::Value = serde_json::from_slice(&messages[0].3).unwrap();
        assert_eq!(json["step"], 4);
        assert_eq!(
            json["data"],
            base64::engine::general_purpose::STANDARD.encode([1, 0, 2, 0, 3, 0, 2, 1])
        );
    }

    /// An undeclared output whose encoding Foxglove can't display (here
    /// `yuv420`, which opencv-video-capture can send) is not an image to this
    /// export: it stays Arrow rather than being dropped.
    #[test]
    fn foxglove_keeps_undeclared_frames_it_cannot_display_as_arrow() {
        let payload = ipc_bytes(std::sync::Arc::new(arrow::array::UInt8Array::from(vec![
            0u8;
            12
        ])));
        let messages = export_messages(
            "nodes: []",
            vec![(
                "camera",
                "image",
                output_event_bytes_with_params(
                    "camera",
                    "image",
                    &payload,
                    &camera_params(4, 2, "yuv420"),
                ),
            )],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].1, "arrow-ipc");
        assert_eq!(messages[0].3, payload);
    }

    /// The README and examples/hub-dataflow declare `std/media/v1/Image` for a
    /// camera that sends the flat-array convention, not the struct.
    #[test]
    fn foxglove_declared_image_sent_as_camera_frame_is_converted() {
        let messages = export_messages(
            CAMERA_DESCRIPTOR,
            vec![(
                "camera",
                "image",
                output_event_bytes_with_params(
                    "camera",
                    "image",
                    &ipc_bytes(std::sync::Arc::new(arrow::array::UInt8Array::from(vec![
                        0u8;
                        24
                    ]))),
                    &camera_params(4, 2, "bgr8"),
                ),
            )],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].2.as_deref(), Some("foxglove.RawImage"));
    }

    /// NV12 is a full-size Y plane plus a half-height UV plane: `step` is the
    /// Y row, not `len / height`.
    #[test]
    fn foxglove_nv12_step_is_the_luma_row() {
        let messages = export_messages(
            "nodes: []",
            vec![(
                "camera",
                "image",
                output_event_bytes_with_params(
                    "camera",
                    "image",
                    &ipc_bytes(std::sync::Arc::new(arrow::array::UInt8Array::from(vec![
                        0u8;
                        12
                    ]))),
                    &camera_params(4, 2, "nv12"),
                ),
            )],
        );
        assert_eq!(messages.len(), 1);
        let json: serde_json::Value = serde_json::from_slice(&messages[0].3).unwrap();
        assert_eq!(json["step"], 4);
    }

    /// Outputs of `operators:` are recorded as `<operator>/<output>`.
    #[test]
    fn foxglove_reads_operator_output_types() {
        let descriptor = "nodes:
  - id: runtime
    operators:
      - id: cam
        output_types:
          image: std/media/v1/Image
";
        let messages = export_messages(
            descriptor,
            vec![(
                "runtime",
                "cam/image",
                output_event_bytes(
                    "runtime",
                    "cam/image",
                    &ipc_bytes(rgb_image_struct(2, 2, &[0; 12])),
                ),
            )],
        );
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].2.as_deref(), Some("foxglove.RawImage"));
    }

    /// A single `operator:` node's outputs are recorded under the bare output
    /// id, by `dora record` and by `--proxy` alike, whatever the operator's id.
    #[test]
    fn foxglove_reads_single_operator_output_types() {
        let descriptor = "nodes:
  - id: node
    operator:
      python: cam.py
      outputs: [image]
      output_types:
        image: std/media/v1/Image
  - id: named
    operator:
      id: cam
      python: cam.py
      outputs: [image]
      output_types:
        image: std/media/v1/Image
";
        let image = || ipc_bytes(rgb_image_struct(2, 2, &[0; 12]));
        let messages = export_messages(
            descriptor,
            vec![
                (
                    "node",
                    "image",
                    output_event_bytes("node", "image", &image()),
                ),
                (
                    "named",
                    "image",
                    output_event_bytes("named", "image", &image()),
                ),
            ],
        );
        assert_eq!(messages.len(), 2);
        for message in &messages {
            assert_eq!(message.2.as_deref(), Some("foxglove.RawImage"));
        }
    }

    /// `std/media/v1/Image[encoding=jpeg]` (the example in dora-core's type
    /// docs) and an `Image` struct whose `encoding` is a compressed format are
    /// compressed images, not raw ones.
    #[test]
    fn foxglove_image_with_a_compressed_encoding_is_a_compressed_image() {
        use arrow::array::{LargeBinaryArray, StringArray, StructArray, UInt32Array};
        use arrow_schema::{DataType, Field};
        let descriptor = "nodes:
  - id: camera
    output_types:
      jpeg: std/media/v1/Image[encoding=jpeg]
      image: std/media/v1/Image
";
        let jpeg = vec![0xFFu8, 0xD8, 0xFF, 0xE0, 1, 2];
        let png = [0x89u8, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A, 9];
        let png_struct: arrow::array::ArrayRef = std::sync::Arc::new(StructArray::from(vec![
            (
                std::sync::Arc::new(Field::new("width", DataType::UInt32, false)),
                std::sync::Arc::new(UInt32Array::from(vec![640])) as arrow::array::ArrayRef,
            ),
            (
                std::sync::Arc::new(Field::new("height", DataType::UInt32, false)),
                std::sync::Arc::new(UInt32Array::from(vec![480])) as arrow::array::ArrayRef,
            ),
            (
                std::sync::Arc::new(Field::new("encoding", DataType::Utf8, false)),
                std::sync::Arc::new(StringArray::from(vec!["png"])) as arrow::array::ArrayRef,
            ),
            (
                std::sync::Arc::new(Field::new("data", DataType::LargeBinary, false)),
                std::sync::Arc::new(LargeBinaryArray::from(vec![&png[..]]))
                    as arrow::array::ArrayRef,
            ),
        ]));
        let messages = export_messages(
            descriptor,
            vec![
                (
                    "camera",
                    "jpeg",
                    output_event_bytes(
                        "camera",
                        "jpeg",
                        &ipc_bytes(std::sync::Arc::new(arrow::array::UInt8Array::from(jpeg))),
                    ),
                ),
                (
                    "camera",
                    "image",
                    output_event_bytes("camera", "image", &ipc_bytes(png_struct)),
                ),
            ],
        );
        assert_eq!(messages.len(), 2);
        for (message, format) in messages.iter().zip(["jpeg", "png"]) {
            assert_eq!(
                message.2.as_deref(),
                Some("foxglove.CompressedImage"),
                "{}",
                message.0
            );
            let json: serde_json::Value = serde_json::from_slice(&message.3).unwrap();
            assert_eq!(json["format"], format);
        }
    }
}
