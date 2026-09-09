use std::{
    collections::HashMap,
    fs::File,
    io::Write,
    time::{Duration, Instant, SystemTime},
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

/// Flush the recording's `BufWriter` after this many records. Bounds crash
/// loss by record *count* on a high-rate stream.
const FLUSH_EVERY_N_RECORDS: u64 = 100;

/// Flush the recording's `BufWriter` at least this often. Bounds crash loss by
/// wall-clock *time*: the count bound alone lets a stream below
/// `FLUSH_EVERY_N_RECORDS / FLUSH_INTERVAL` (~100 Hz) sit unflushed for up to
/// `FLUSH_EVERY_N_RECORDS / rate` seconds (a 2 Hz stream ≈ 50 s), which a
/// SIGKILL would lose. Also used as the event loop's idle poll interval so a
/// stream that goes quiet still gets its tail flushed.
const FLUSH_INTERVAL: Duration = Duration::from_secs(1);

/// Decides when to flush buffered records to disk so an abrupt termination
/// (SIGKILL, or SIGINT/Ctrl-C without a clean `Stop`) can only lose the records
/// written since the last flush, keeping the on-disk file within the reader's
/// torn-tail recovery model (see `dora_recording::RecordingReader`). Two
/// independent bounds cap the loss window: a record *count* bound and a
/// wall-clock *time* bound. Both are enforced on the write path (in
/// [`after_write`](Self::after_write)) so the time bound actually binds on an
/// active stream — an idle-only timeout never fires while events keep arriving.
/// [`on_idle`](Self::on_idle) additionally flushes the tail of a stream that has
/// gone quiet without closing.
///
/// (`flush()` reaches the OS page cache, not the platter, so this bounds loss on
/// process death — SIGKILL/SIGINT — not on power loss, which would need
/// `sync_data`.)
struct FlushPolicy {
    records_since_flush: u64,
    last_flush: Instant,
}

impl FlushPolicy {
    fn new() -> Self {
        Self {
            records_since_flush: 0,
            last_flush: Instant::now(),
        }
    }

    /// Whether any records are buffered that a crash right now would lose.
    fn has_buffered(&self) -> bool {
        self.records_since_flush > 0
    }

    /// Call after each record is written. Flushes when *either* bound trips:
    /// `FLUSH_EVERY_N_RECORDS` records have accumulated, or `FLUSH_INTERVAL` has
    /// elapsed since the last flush. Checking the time bound here, on the write
    /// path, is what makes it bind on an active stream.
    fn after_write<W: Write>(&mut self, writer: &mut RecordingWriter<W>) -> eyre::Result<()> {
        self.records_since_flush += 1;
        if self.records_since_flush >= FLUSH_EVERY_N_RECORDS
            || self.last_flush.elapsed() >= FLUSH_INTERVAL
        {
            self.flush(writer)?;
        }
        Ok(())
    }

    /// Call on an idle tick (no event within `FLUSH_INTERVAL`): flush anything
    /// still buffered so a stream that goes quiet doesn't leave its last few
    /// records unflushed indefinitely.
    fn on_idle<W: Write>(&mut self, writer: &mut RecordingWriter<W>) -> eyre::Result<()> {
        if self.has_buffered() {
            self.flush(writer)?;
        }
        Ok(())
    }

    fn flush<W: Write>(&mut self, writer: &mut RecordingWriter<W>) -> eyre::Result<()> {
        writer.flush()?;
        self.records_since_flush = 0;
        self.last_flush = Instant::now();
        Ok(())
    }
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
    let mut flush_policy = FlushPolicy::new();

    eprintln!("dora-record-node: recording to {output_file}");

    loop {
        // Only arm the idle-flush timer when records are buffered that a crash
        // would lose; when everything is already flushed there is nothing to
        // rescue on a timeout, so block on `recv()` and pay no per-event timer
        // setup. `recv_timeout` returns `None` only on stream close and yields
        // `Event::Error` on an idle timeout, which falls through to the `_` arm
        // and drives a time-based flush of the buffered tail.
        let event = if flush_policy.has_buffered() {
            events.recv_timeout(FLUSH_INTERVAL)
        } else {
            events.recv()
        };
        let Some(event) = event else { break };
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

                let now_nanos = unix_nanos(SystemTime::now());

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
            // `Event::Stop` is deliberately NOT a `break`. The node API gives
            // `Stop` strict priority over inputs that were already queued behind
            // it and re-delivers those inputs on the following `recv()`s before
            // closing (see `EventStream` docs and the
            // `recv_drains_buffered_scheduler_inputs_after_stop` test). Breaking
            // here would drop that tail of already-produced messages — exactly
            // what a recorder must not do. Falling through records nothing for
            // the Stop itself but flushes the buffered tail; the stream then
            // drains its queued inputs and returns `None`, ending the loop.
            //
            // The same `_` arm handles an idle-timeout tick and any other
            // ignored control event: flush the buffered records so the
            // durability window stays time-bounded.
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
            version: dora_recording::FORMAT_VERSION,
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
    fn records_below_both_bounds_stay_buffered() {
        // A handful of records written back-to-back: below the count bound, and
        // `last_flush.elapsed()` is far below `FLUSH_INTERVAL`, so neither bound
        // trips and nothing reaches the sink yet. (This is the lower edge the
        // count bound is supposed to hold; the time-bound and idle tests below
        // pin what rescues these records before a crash.)
        let (sink, mut writer) = test_writer();
        let mut policy = FlushPolicy::new();
        for _ in 0..3 {
            writer.write_entry(&sample_entry()).unwrap();
            policy.after_write(&mut writer).unwrap();
        }
        assert_eq!(
            sink.flushed_len(),
            0,
            "records below both bounds must remain buffered (unflushed)"
        );
    }

    #[test]
    fn time_bound_flushes_on_the_write_path_below_the_count_bound() {
        // The core of the fix: on a stream faster than 1 Hz the idle timeout
        // never fires, so the time bound must bind on the *write* path. Backdate
        // `last_flush` past `FLUSH_INTERVAL`, then write a single record (well
        // under the count bound): `after_write` must flush it. Without the
        // write-path time check (count bound only) this record would sit
        // buffered and a SIGKILL would lose it.
        let (sink, mut writer) = test_writer();
        let mut policy = FlushPolicy::new();
        policy.last_flush = Instant::now()
            .checked_sub(FLUSH_INTERVAL + Duration::from_millis(50))
            .expect("monotonic clock is younger than FLUSH_INTERVAL");

        writer.write_entry(&sample_entry()).unwrap();
        policy.after_write(&mut writer).unwrap();

        assert!(
            sink.flushed_len() > 0,
            "the elapsed time bound must flush on the write path, below the count bound"
        );
        assert_eq!(
            policy.records_since_flush, 0,
            "flush must reset the record counter"
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

        // One short of the count bound. The writes run back-to-back, so
        // `last_flush.elapsed()` stays far below `FLUSH_INTERVAL` and the time
        // bound does not fire; the 99 × ~30-byte records also stay well under
        // `BufWriter`'s 8 KiB default, so none reach the sink on their own.
        for _ in 0..(FLUSH_EVERY_N_RECORDS - 1) {
            writer.write_entry(&sample_entry()).unwrap();
            policy.after_write(&mut writer).unwrap();
        }
        assert_eq!(
            policy.records_since_flush,
            FLUSH_EVERY_N_RECORDS - 1,
            "no flush before {FLUSH_EVERY_N_RECORDS} records"
        );
        assert_eq!(
            sink.flushed_len(),
            0,
            "must not flush before {FLUSH_EVERY_N_RECORDS} records"
        );

        // The Nth record trips the count bound.
        writer.write_entry(&sample_entry()).unwrap();
        policy.after_write(&mut writer).unwrap();
        assert_eq!(
            policy.records_since_flush, 0,
            "count bound must reset the counter on the {FLUSH_EVERY_N_RECORDS}th record"
        );
        assert!(
            sink.flushed_len() > 0,
            "count bound must flush every {FLUSH_EVERY_N_RECORDS} records"
        );
    }
}
