use std::{
    collections::{BTreeMap, HashMap},
    fs::File,
    time::SystemTime,
};

use aligned_vec::{AVec, ConstAlign};
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
};
use dora_node_api::{DoraNode, Event, arrow_utils, arrow_v59::datatypes::DataType};
use dora_recording::{RecordEntry, RecordingHeader, RecordingWriter};
use eyre::Context;

/// Parse `DORA_RECORD_TOPICS` (`{ "input_id": "source_node/source_output" }`)
/// into a lookup from the record node's own input id to the validated
/// `(source_node, source_output)` ids.
///
/// The node and output ids are parsed via [`str::parse`] (`FromStr`), which
/// *validates* the character set and returns an error, rather than the
/// panicking `NodeId::from`/`DataId::from` (`From<String>`) conversions. A
/// malformed topic (e.g. a source node id containing a space) therefore fails
/// the node cleanly at startup instead of panicking mid-run when the offending
/// event is first recorded.
fn build_reverse_map(topics_json: &str) -> eyre::Result<HashMap<String, (NodeId, DataId)>> {
    let topic_map: HashMap<String, String> =
        serde_json::from_str(topics_json).wrap_err("failed to parse DORA_RECORD_TOPICS")?;

    let mut reverse_map: HashMap<String, (NodeId, DataId)> = HashMap::new();
    for (input_id, source) in &topic_map {
        let (node_id, output_id) = source
            .split_once('/')
            .ok_or_else(|| eyre::eyre!("invalid topic format: {source}"))?;
        let node_id: NodeId = node_id
            .parse()
            .wrap_err_with(|| format!("invalid source node id in topic `{source}`"))?;
        let output_id: DataId = output_id
            .parse()
            .wrap_err_with(|| format!("invalid source output id in topic `{source}`"))?;
        reverse_map.insert(input_id.clone(), (node_id, output_id));
    }
    Ok(reverse_map)
}

/// Nanoseconds since the Unix epoch, saturating to 0 when the wall clock reads
/// a pre-epoch time.
///
/// `SystemTime::duration_since(UNIX_EPOCH)` returns `Err` whenever the clock is
/// set before 1970 — common on battery-less embedded/robotics hardware that
/// boots at (or before) the epoch until NTP/GPS sync lands. This runs once per
/// recorded message, so an `.unwrap()` here would abort the recorder mid-capture.
/// Saturating to 0 matches the `saturating_sub` already used when computing the
/// per-entry offset from `start_nanos`.
fn unix_nanos(now: SystemTime) -> u64 {
    now.duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// Fold one batch of per-input drop counts into the running total.
///
/// The node API caps every input queue and evicts the oldest events once a
/// producer outruns this node's disk writes. It reports those evictions only
/// through `tracing`, and this binary installs no subscriber, so without this
/// accounting the drops are invisible: the recording is short and the final
/// `Messages:` line still reports a confident total.
fn accumulate_drops(total: &mut BTreeMap<String, u64>, batch: HashMap<DataId, u64>) {
    for (input_id, count) in batch {
        *total.entry(input_id.to_string()).or_insert(0) += count;
    }
}

/// Render the incomplete-recording warning, or `None` when nothing was dropped.
///
/// Input ids are translated back to their `source_node/source_output` topic via
/// `reverse_map` -- the record node's own input ids are the sanitized
/// `node___output` form, which is not what the user asked to record.
fn format_drop_report(
    dropped: &BTreeMap<String, u64>,
    reverse_map: &HashMap<String, (NodeId, DataId)>,
) -> Option<String> {
    if dropped.is_empty() {
        return None;
    }
    let total: u64 = dropped.values().sum();
    let mut report = format!(
        "  WARNING:  {total} message(s) were dropped before reaching the recorder.\n            \
         THIS RECORDING IS INCOMPLETE."
    );
    for (input_id, count) in dropped {
        let topic = match reverse_map.get(input_id) {
            Some((node, output)) => format!("{node}/{output}"),
            None => input_id.clone(),
        };
        report.push_str(&format!("\n              {topic}: {count}"));
    }
    report.push_str(
        "\n            The recorder could not keep up with its producers. Re-record with a \
         larger `dora record --queue-size`, or record fewer topics with `--topics`.",
    );
    Some(report)
}

fn main() -> eyre::Result<()> {
    let output_file =
        std::env::var("DORA_RECORD_FILE").wrap_err("DORA_RECORD_FILE env var not set")?;
    let topics_json =
        std::env::var("DORA_RECORD_TOPICS").wrap_err("DORA_RECORD_TOPICS env var not set")?;
    let descriptor_yaml = std::env::var("DORA_RECORD_DESCRIPTOR").unwrap_or_default();

    // Build reverse map: input_id -> (source_node_id, source_output_id).
    let reverse_map = build_reverse_map(&topics_json)?;

    let (_node, mut events) = DoraNode::init_from_env()?;

    let start_nanos = unix_nanos(SystemTime::now());

    let header = RecordingHeader {
        version: dora_recording::FORMAT_VERSION,
        start_nanos,
        dataflow_id: uuid::Uuid::new_v4(),
        descriptor_yaml: descriptor_yaml.into_bytes(),
    };

    let file =
        File::create(&output_file).wrap_err_with(|| format!("failed to create {output_file}"))?;
    let mut writer = RecordingWriter::new(file, &header)?;
    let mut msg_count: u64 = 0;
    let mut dropped: BTreeMap<String, u64> = BTreeMap::new();

    eprintln!("dora-record-node: recording to {output_file}");

    while let Some(event) = events.recv() {
        // Drain per-event: the counters reset on read, so folding them here
        // keeps a complete total even for a recording that ends abruptly.
        accumulate_drops(&mut dropped, events.drain_drop_counts());
        match event {
            Event::Input { id, metadata, data } => {
                let (source_node, source_output) = match reverse_map.get(&*id) {
                    Some(pair) => pair,
                    None => continue,
                };

                // Record the payload as a self-describing Arrow IPC stream so
                // replay can reconstruct the array without a type sidecar.
                let arrow_data = &data;
                let raw_data = if matches!(arrow_data.as_array().data_type(), DataType::Null)
                    && arrow_data.is_empty()
                {
                    // Exactly the unit array that replay rebuilds from an absent
                    // payload (`NullArray::new(0)`) — record `None` to skip the
                    // IPC framing.
                    None
                } else {
                    // Encode every other array — including a zero-length *typed*
                    // array (e.g. an empty `Float32Array`) and a non-empty
                    // `NullArray` — as a self-describing IPC stream, so replay
                    // preserves the declared type and length instead of
                    // collapsing it to `NullArray::new(0)`. (The deleted
                    // `type_info` sidecar used to preserve this; #2027/#2083.)
                    //
                    // Use the hand-rolled 1-copy fast path when the array type
                    // is eligible — it copies each buffer straight into the
                    // aligned target — falling back to the official writer
                    // otherwise, mirroring `DoraNode::send_output_array`. The
                    // previous code always went through the official writer
                    // (which stages the body in an internal `Vec`, ~2 payload
                    // copies) and then copied the result a third time into the
                    // `AVec`, on every recorded message.
                    let encoded: AVec<u8, ConstAlign<128>> =
                        match arrow_utils::ipc_encode::ipc_fast_path_len(arrow_data) {
                            Some(len) => {
                                let mut buf: AVec<u8, ConstAlign<128>> =
                                    AVec::__from_elem(128, 0, len);
                                arrow_utils::ipc_encode::encode_ipc_into(arrow_data, &mut buf)
                                    .wrap_err("failed to Arrow-IPC-encode recorded output")?;
                                buf
                            }
                            None => {
                                let ipc_bytes =
                                    arrow_utils::ipc_encode::encode_ipc_to_vec(arrow_data)
                                        .wrap_err("failed to Arrow-IPC-encode recorded output")?;
                                AVec::from_slice(128, &ipc_bytes)
                            }
                        };
                    Some(encoded)
                };

                let timestamp = metadata.timestamp();
                let inter_event = InterDaemonEvent::Output {
                    dataflow_id: uuid::Uuid::nil(),
                    node_id: source_node.clone(),
                    output_id: source_output.clone(),
                    metadata,
                    data: raw_data,
                };

                let timestamped = Timestamped {
                    inner: inter_event,
                    timestamp,
                };
                let event_bytes = timestamped.serialize()?;

                let now_nanos = unix_nanos(SystemTime::now());

                let entry = RecordEntry {
                    node_id: source_node.to_string(),
                    output_id: source_output.to_string(),
                    timestamp_offset_nanos: now_nanos.saturating_sub(start_nanos),
                    event_bytes,
                };
                writer.write_entry(&entry)?;
                msg_count += 1;
            }
            Event::Stop(_) => break,
            _ => {}
        }
    }

    // Anything the scheduler evicted between the last event and the stop.
    accumulate_drops(&mut dropped, events.drain_drop_counts());

    let footer = writer.finish()?;
    eprintln!("dora-record-node: recording complete");
    eprintln!("  Messages: {msg_count}");
    eprintln!("  Bytes:    {}", footer.total_bytes);
    eprintln!("  File:     {output_file}");
    if let Some(report) = format_drop_report(&dropped, &reverse_map) {
        eprintln!("{report}");
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{accumulate_drops, build_reverse_map, format_drop_report, unix_nanos};
    use dora_message::id::DataId;
    use std::collections::{BTreeMap, HashMap};
    use std::time::{Duration, SystemTime};

    #[test]
    fn drop_counts_accumulate_across_drains() {
        // `drain_drop_counts` resets on read, so the totals must be summed
        // across calls -- not overwritten by the latest batch.
        let mut total = BTreeMap::new();
        accumulate_drops(
            &mut total,
            HashMap::from([(DataId::from("a".to_string()), 2)]),
        );
        accumulate_drops(
            &mut total,
            HashMap::from([
                (DataId::from("a".to_string()), 3),
                (DataId::from("b".to_string()), 1),
            ]),
        );
        assert_eq!(total.get("a"), Some(&5));
        assert_eq!(total.get("b"), Some(&1));
    }

    #[test]
    fn a_clean_recording_reports_no_drops() {
        let reverse_map = build_reverse_map(r#"{"camera___image":"camera/image"}"#).unwrap();
        assert!(format_drop_report(&BTreeMap::new(), &reverse_map).is_none());
    }

    #[test]
    fn drop_report_names_the_topic_not_the_sanitized_input_id() {
        // The record node's input ids are the sanitized `node___output` form;
        // a user who asked to record `camera/image` needs to read that back.
        let reverse_map = build_reverse_map(r#"{"camera___image":"camera/image"}"#).unwrap();
        let dropped = BTreeMap::from([("camera___image".to_string(), 7)]);

        let report = format_drop_report(&dropped, &reverse_map).expect("drops must be reported");
        assert!(report.contains("camera/image: 7"), "{report}");
        assert!(report.contains("INCOMPLETE"), "{report}");
        assert!(report.contains("--queue-size"), "{report}");
    }

    #[test]
    fn drop_report_falls_back_to_the_raw_id_for_an_unmapped_input() {
        // Non-topic pseudo-inputs (the node API's internal event queue) have
        // no entry in the reverse map; report them rather than dropping them
        // from the total.
        let reverse_map = build_reverse_map(r#"{"camera___image":"camera/image"}"#).unwrap();
        let dropped = BTreeMap::from([("dora/some_internal".to_string(), 3)]);

        let report = format_drop_report(&dropped, &reverse_map).expect("drops must be reported");
        assert!(report.contains("dora/some_internal: 3"), "{report}");
    }

    #[test]
    fn unix_nanos_saturates_on_pre_epoch_clock() {
        // A pre-epoch wall clock must not panic the recorder mid-capture.
        let before_epoch = SystemTime::UNIX_EPOCH - Duration::from_secs(60);
        assert_eq!(unix_nanos(before_epoch), 0);
        // The epoch itself is 0, and a post-epoch time is its offset in nanos.
        assert_eq!(unix_nanos(SystemTime::UNIX_EPOCH), 0);
        assert_eq!(
            unix_nanos(SystemTime::UNIX_EPOCH + Duration::from_nanos(1_500)),
            1_500
        );
    }

    #[test]
    fn valid_topics_parse() {
        let map = build_reverse_map(r#"{"in":"camera/image"}"#).unwrap();
        let (node, output) = map.get("in").unwrap();
        assert_eq!(node.to_string(), "camera");
        assert_eq!(output.to_string(), "image");
    }

    #[test]
    fn missing_slash_is_an_error() {
        assert!(build_reverse_map(r#"{"in":"camera"}"#).is_err());
    }

    // A source node id with an invalid character (a space) must yield a clean
    // startup error, not a panic from the `NodeId::from(String)` conversion
    // that the recorder used to call in its hot loop.
    #[test]
    fn invalid_node_id_is_an_error_not_a_panic() {
        let err = build_reverse_map(r#"{"in":"my node/image"}"#).unwrap_err();
        assert!(
            format!("{err:#}").contains("invalid source node id"),
            "unexpected error: {err:#}"
        );
    }

    #[test]
    fn invalid_output_id_is_an_error_not_a_panic() {
        let err = build_reverse_map(r#"{"in":"camera/bad output"}"#).unwrap_err();
        assert!(
            format!("{err:#}").contains("invalid source output id"),
            "unexpected error: {err:#}"
        );
    }
}
