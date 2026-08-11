use std::{
    collections::HashMap,
    fs::File,
    io::Write,
    time::{Duration, SystemTime},
};

use aligned_vec::{AVec, ConstAlign};
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
};
use dora_node_api::{DoraNode, Event, arrow::datatypes::DataType, arrow_utils};
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

/// Flush the recording's `BufWriter` after this many records even if the node
/// never idles (a busy, high-rate stream). Bounds crash loss by record *count*.
const FLUSH_EVERY_N_RECORDS: u64 = 100;

/// Poll interval for the event loop. When no event arrives within this window
/// the loop flushes any buffered records, bounding crash loss by wall-clock
/// *time* — the count bound alone lets a low-rate stream sit unflushed for
/// (100 / rate) seconds (a 1 Hz stream ≈ 100 s), which a SIGKILL would lose.
const FLUSH_INTERVAL: Duration = Duration::from_secs(1);

/// Decides when to flush buffered records to disk so an abrupt termination
/// (SIGKILL / Ctrl-C without a clean `Stop`) can only lose the records written
/// since the last flush, keeping the on-disk file within the reader's torn-tail
/// recovery model (see `dora_recording::RecordingReader`). Two independent
/// bounds cap the loss window: a record *count* bound and a wall-clock *time*
/// bound driven by the event loop's idle ticks.
struct FlushPolicy {
    records_since_flush: u64,
}

impl FlushPolicy {
    fn new() -> Self {
        Self {
            records_since_flush: 0,
        }
    }

    /// Call after each record is written: flush once `FLUSH_EVERY_N_RECORDS`
    /// have accumulated.
    fn after_write<W: Write>(&mut self, writer: &mut RecordingWriter<W>) -> eyre::Result<()> {
        self.records_since_flush += 1;
        if self.records_since_flush >= FLUSH_EVERY_N_RECORDS {
            writer.flush()?;
            self.records_since_flush = 0;
        }
        Ok(())
    }

    /// Call on an idle tick (no event within `FLUSH_INTERVAL`): flush anything
    /// still buffered so a slow stream can't sit unflushed indefinitely.
    fn on_idle<W: Write>(&mut self, writer: &mut RecordingWriter<W>) -> eyre::Result<()> {
        if self.records_since_flush > 0 {
            writer.flush()?;
            self.records_since_flush = 0;
        }
        Ok(())
    }
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

    let start_nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    let header = RecordingHeader {
        version: 1,
        start_nanos,
        dataflow_id: uuid::Uuid::new_v4(),
        descriptor_yaml: descriptor_yaml.into_bytes(),
    };

    let file =
        File::create(&output_file).wrap_err_with(|| format!("failed to create {output_file}"))?;
    let mut writer = RecordingWriter::new(file, &header)?;
    let mut msg_count: u64 = 0;
    let mut flush_policy = FlushPolicy::new();

    eprintln!("dora-record-node: recording to {output_file}");

    // `recv_timeout` returns `None` only when the stream is closed; on an idle
    // timeout it yields `Event::Error`, which (like any other non-input event)
    // falls through to the `_` arm and drives a time-based flush.
    while let Some(event) = events.recv_timeout(FLUSH_INTERVAL) {
        match event {
            Event::Input { id, metadata, data } => {
                let (source_node, source_output) = match reverse_map.get(&*id) {
                    Some(pair) => pair,
                    None => continue,
                };

                // Record the payload as a self-describing Arrow IPC stream so
                // replay can reconstruct the array without a type sidecar.
                let arrow_data = data.to_data();
                let raw_data = if matches!(arrow_data.data_type(), DataType::Null)
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
                        match arrow_utils::ipc_encode::ipc_fast_path_len(&arrow_data) {
                            Some(len) => {
                                let mut buf: AVec<u8, ConstAlign<128>> =
                                    AVec::__from_elem(128, 0, len);
                                arrow_utils::ipc_encode::encode_ipc_into(&arrow_data, &mut buf)
                                    .wrap_err("failed to Arrow-IPC-encode recorded output")?;
                                buf
                            }
                            None => {
                                let ipc_bytes =
                                    arrow_utils::ipc_encode::encode_ipc_to_vec(&arrow_data)
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

                let now_nanos = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64;

                let entry = RecordEntry {
                    node_id: source_node.to_string(),
                    output_id: source_output.to_string(),
                    timestamp_offset_nanos: now_nanos.saturating_sub(start_nanos),
                    event_bytes,
                };
                writer.write_entry(&entry)?;
                msg_count += 1;
                flush_policy.after_write(&mut writer)?;
            }
            Event::Stop(_) => break,
            // Idle-timeout tick (or any other ignored event): flush buffered
            // records so the durability window is time-bounded, not just
            // count-bounded.
            _ => flush_policy.on_idle(&mut writer)?,
        }
    }

    let footer = writer.finish()?;
    eprintln!("dora-record-node: recording complete");
    eprintln!("  Messages: {msg_count}");
    eprintln!("  Bytes:    {}", footer.total_bytes);
    eprintln!("  File:     {output_file}");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;

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

    /// A `Write` sink that records how many bytes have actually reached it.
    /// `RecordingWriter` buffers through a `BufWriter`, so bytes land here only
    /// when that buffer is flushed — letting a test observe flush behaviour
    /// without touching the filesystem.
    #[derive(Clone, Default)]
    struct ProbeSink(Rc<RefCell<Vec<u8>>>);

    impl std::io::Write for ProbeSink {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.0.borrow_mut().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl ProbeSink {
        /// Bytes that have been flushed through to the sink so far.
        fn flushed_len(&self) -> usize {
            self.0.borrow().len()
        }
    }

    fn test_writer() -> (ProbeSink, RecordingWriter<ProbeSink>) {
        let sink = ProbeSink::default();
        let header = RecordingHeader {
            version: 1,
            start_nanos: 0,
            dataflow_id: uuid::Uuid::nil(),
            descriptor_yaml: Vec::new(),
        };
        let writer = RecordingWriter::new(sink.clone(), &header).expect("build writer");
        (sink, writer)
    }

    fn sample_entry() -> RecordEntry {
        RecordEntry {
            node_id: "src".to_string(),
            output_id: "out".to_string(),
            timestamp_offset_nanos: 0,
            event_bytes: vec![1, 2, 3, 4],
        }
    }

    #[test]
    fn records_below_the_count_bound_stay_buffered() {
        // A handful of records, far below the count bound and with no idle tick:
        // nothing reaches the sink, so a SIGKILL here would lose them. This is
        // the durability gap the wall-clock bound closes.
        let (sink, mut writer) = test_writer();
        let mut policy = FlushPolicy::new();
        for _ in 0..3 {
            writer.write_entry(&sample_entry()).unwrap();
            policy.after_write(&mut writer).unwrap();
        }
        assert_eq!(
            sink.flushed_len(),
            0,
            "records below the count bound must remain buffered (unflushed)"
        );
    }

    #[test]
    fn idle_tick_flushes_records_the_count_bound_would_hold() {
        let (sink, mut writer) = test_writer();
        let mut policy = FlushPolicy::new();
        for _ in 0..3 {
            writer.write_entry(&sample_entry()).unwrap();
            policy.after_write(&mut writer).unwrap();
        }
        assert_eq!(sink.flushed_len(), 0, "precondition: still buffered");

        // The idle tick (what the event loop drives on a `recv_timeout` timeout)
        // must push the buffered records to the sink.
        policy.on_idle(&mut writer).unwrap();
        assert!(
            sink.flushed_len() > 0,
            "idle flush must persist buffered records so a slow stream is time-bounded"
        );

        // A second idle tick with nothing buffered must not re-flush.
        let after_first = sink.flushed_len();
        policy.on_idle(&mut writer).unwrap();
        assert_eq!(
            sink.flushed_len(),
            after_first,
            "idle flush with nothing buffered must be a no-op"
        );
    }

    #[test]
    fn count_bound_flushes_every_n_records() {
        let (sink, mut writer) = test_writer();
        let mut policy = FlushPolicy::new();

        // One short of the count bound: nothing flushed yet.
        for _ in 0..(FLUSH_EVERY_N_RECORDS - 1) {
            writer.write_entry(&sample_entry()).unwrap();
            policy.after_write(&mut writer).unwrap();
        }
        assert_eq!(
            sink.flushed_len(),
            0,
            "must not flush before {FLUSH_EVERY_N_RECORDS} records"
        );

        // The Nth record trips the count bound.
        writer.write_entry(&sample_entry()).unwrap();
        policy.after_write(&mut writer).unwrap();
        assert!(
            sink.flushed_len() > 0,
            "count bound must flush every {FLUSH_EVERY_N_RECORDS} records"
        );
    }
}
