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

    let file = File::open(&args.input)
        .wrap_err_with(|| eyre!("failed to open recording `{}`", args.input))?;
    let mut reader =
        RecordingReader::open(file).wrap_err("failed to initialise recording reader")?;
    let start_nanos: u64 = reader.header().start_nanos;

    let output = args
        .output
        .unwrap_or_else(|| format!("{}.mcap", args.input));
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
