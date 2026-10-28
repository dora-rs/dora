use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use arrow_schema::DataType;
use clap::Args;
use dora_core::config::InputMapping;
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};

use crate::{
    command::{
        Executable, default_tracing,
        topic::{
            catch_arrow_panic,
            selector::{TopicSelector, node_topic_inputs},
        },
    },
    common::CoordinatorOptions,
};

/// Display detailed metadata of a topic.
///
/// Shows topic type, publisher, subscribers, and statistics (message count,
/// bandwidth, publishing frequency).
///
/// Topic inspection requires debug mode on the dataflow:
///
/// ```yaml
/// debug:
///   enable_debug_inspection: true
/// ```
///
/// Examples:
///
/// Get info for a single topic:
///   dora topic info -d my-dataflow camera_node/image
///
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Info {
    #[clap(flatten)]
    selector: TopicSelector,

    /// Duration in seconds to collect statistics (default: 5)
    #[clap(long, default_value_t = 5, value_parser = clap::value_parser!(u64).range(1..))]
    duration: u64,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Info {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        info(self.coordinator, self.selector, self.duration)
    }
}

#[derive(Clone, Default)]
struct TopicStats {
    message_count: Arc<Mutex<u64>>,
    total_bytes: Arc<Mutex<u64>>,
    timestamps: Arc<Mutex<Vec<Instant>>>,
    data_type: Arc<Mutex<Option<DataType>>>,
}

/// Maximum timestamps to keep for frequency calculation.
const MAX_TIMESTAMPS: usize = 10_000;

impl TopicStats {
    fn record(&self, data_size: usize, data_type: Option<DataType>, now: Instant) {
        *self.message_count.lock().unwrap_or_else(|e| e.into_inner()) += 1;
        *self.total_bytes.lock().unwrap_or_else(|e| e.into_inner()) += data_size as u64;
        let mut ts = self.timestamps.lock().unwrap_or_else(|e| e.into_inner());
        if ts.len() >= MAX_TIMESTAMPS {
            // Keep the recent half
            let drain_to = ts.len() / 2;
            ts.drain(..drain_to);
        }
        ts.push(now);
        drop(ts);
        if let Some(data_type) = data_type {
            *self.data_type.lock().unwrap_or_else(|e| e.into_inner()) = Some(data_type);
        }
    }

    fn calculate_hz(&self, window: Duration) -> Option<f64> {
        let timestamps = self.timestamps.lock().unwrap_or_else(|e| e.into_inner());
        if timestamps.len() < 2 {
            return None;
        }

        let now = Instant::now();
        // `now - window` would panic ("overflow when subtracting duration from
        // instant") when `window` exceeds the monotonic clock value -- reachable
        // via a large `--duration` (the flag accepts any `u64`) when the topic
        // closes early, so the full window never elapses before this runs. When
        // the cutoff predates the monotonic epoch, every recorded timestamp is
        // within the window, so keep them all.
        let recent: Vec<_> = match now.checked_sub(window) {
            Some(cutoff) => timestamps
                .iter()
                .filter(|&&t| t >= cutoff)
                .copied()
                .collect(),
            None => timestamps.iter().copied().collect(),
        };

        if recent.len() < 2 {
            return None;
        }

        let intervals: Vec<f64> = recent
            .windows(2)
            .map(|w| w[1].duration_since(w[0]).as_secs_f64())
            .filter(|&dt| dt > 0.0)
            .collect();

        if intervals.is_empty() {
            return None;
        }

        let avg_interval: f64 = intervals.iter().sum::<f64>() / intervals.len() as f64;
        Some(1.0 / avg_interval)
    }
}

fn info(
    coordinator: CoordinatorOptions,
    selector: TopicSelector,
    duration_secs: u64,
) -> eyre::Result<()> {
    let session = coordinator.connect()?;
    // Resolve once: this yields both the topic and the dataflow descriptor used
    // to find subscribers. Resolving a second time would repeat the coordinator
    // round-trip and, when the dataflow is ambiguous, prompt the user to pick a
    // dataflow again (and could even pick a different one).
    let (dataflow_id, topics, descriptor) = selector.resolve_with_descriptor(&session)?;

    if topics.is_empty() {
        eyre::bail!("No topics specified");
    }

    if topics.len() > 1 {
        eyre::bail!("Please specify exactly one topic");
    }

    let topic = topics.into_iter().next().unwrap();

    // Find subscribers
    let mut subscribers = Vec::new();
    for node in &descriptor.nodes {
        for (input_id, input) in node_topic_inputs(node) {
            if let InputMapping::User(user) = &input.mapping
                && user.source == topic.node_id
                && user.output == topic.data_id
            {
                subscribers.push(format!("{}/{}", node.id, input_id));
            }
        }
    }

    // Subscribe via WS
    let ws_topics = vec![(topic.node_id.clone(), topic.data_id.clone())];
    let (_subscription_id, data_rx) = session.subscribe_topics(dataflow_id, ws_topics)?;

    let stats = Arc::new(TopicStats::default());
    let stats_clone = stats.clone();

    // Collect messages for the specified duration in a background thread
    let duration = Duration::from_secs(duration_secs);
    let start_time = Instant::now();
    std::thread::spawn(move || {
        while start_time.elapsed() < duration {
            match data_rx.recv_timeout(duration.saturating_sub(start_time.elapsed())) {
                Ok(Ok(payload)) => {
                    let event = match Timestamped::deserialize_inter_daemon_event(&payload) {
                        Ok(e) => e,
                        Err(e) => {
                            eprintln!("warning: failed to deserialize event: {e}");
                            continue;
                        }
                    };
                    match event.inner {
                        InterDaemonEvent::Output { data, metadata, .. } => {
                            // Charge the real on-wire size, not the rebuilt
                            // self-describing `data` (which re-prepends the schema
                            // a schema-once output ships only once) — see
                            // `debug_frame_wire_size` (#2584).
                            let data_size = dora_message::metadata::debug_frame_wire_size(
                                &metadata.parameters,
                                data.as_deref(),
                            );
                            // The payload is a self-describing Arrow IPC stream;
                            // read its data type from the schema (best effort —
                            // a malformed schema just leaves it unknown).
                            let data_type =
                                data.as_ref().and_then(|bytes| decode_ipc_data_type(bytes));
                            stats_clone.record(data_size, data_type, Instant::now());
                        }
                        InterDaemonEvent::OutputClosed { .. } => break,
                        // `InterDaemonEvent` is `#[non_exhaustive]`: skip events this build predates.
                        _ => continue,
                    }
                }
                Ok(Err(_)) => continue,
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => break,
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
            }
        }
    })
    .join()
    .map_err(|_| eyre::eyre!("stats collection thread panicked"))?;

    // Use the actual collection time rather than the configured window: the
    // loop breaks early when the publisher closes (`OutputClosed`), so dividing
    // by the full `duration_secs` would under-report the bandwidth of a topic
    // that stops mid-window.
    let elapsed_secs = start_time.elapsed().as_secs_f64();

    // Display the information
    let message_count = *stats
        .message_count
        .lock()
        .unwrap_or_else(|e| e.into_inner());
    let total_bytes = *stats.total_bytes.lock().unwrap_or_else(|e| e.into_inner());
    let data_type = stats
        .data_type
        .lock()
        .unwrap_or_else(|e| e.into_inner())
        .clone();
    let hz = stats.calculate_hz(Duration::from_secs(duration_secs));

    println!("Topic: {}/{}", topic.node_id, topic.data_id);
    println!();

    // Type
    if let Some(data_type) = &data_type {
        println!("Type: {}", format_arrow_type(data_type));
    } else {
        println!("Type: <unknown> (no messages received)");
    }
    println!();

    // Publisher
    println!("Publisher:");
    if let Some(freq) = hz {
        println!("  * {} ({:.1} Hz)", topic.node_id, freq);
    } else {
        println!("  * {} (frequency unknown)", topic.node_id);
    }
    println!();

    // Subscribers
    println!("Subscribers:");
    if subscribers.is_empty() {
        println!("  <none>");
    } else {
        for subscriber in &subscribers {
            println!("  * {}", subscriber);
        }
    }
    println!();

    // Statistics
    println!("Statistics:");
    println!("  Total messages: {}", message_count);
    match bandwidth_mbps(total_bytes, message_count, elapsed_secs) {
        Some(mbps) => println!("  Bandwidth: {:.2} Mbps", mbps),
        None => println!("  Bandwidth: <unknown>"),
    }

    Ok(())
}

/// Read the data type from the schema message of a self-describing Arrow IPC
/// stream. Returns `None` for a malformed schema or one that is not a single
/// column (the type is left unknown).
///
/// Only the first message's flatbuffer metadata is parsed. Decoding the
/// record batch would copy the whole payload, for every sampled frame, just
/// to read a type the schema already names. Going through `StreamReader` is
/// not safe either: it allocates the message's declared `bodyLength` before
/// checking it against the input, so a corrupt frame could abort the CLI.
fn decode_ipc_data_type(bytes: &[u8]) -> Option<DataType> {
    // A stream message is an optional 0xFFFFFFFF continuation marker, the
    // little-endian i32 length of its metadata, then the metadata itself.
    let bytes = bytes.strip_prefix(&[0xff; 4]).unwrap_or(bytes);
    let (len, rest) = bytes.split_first_chunk::<4>()?;
    let len = usize::try_from(i32::from_le_bytes(*len)).ok()?;
    let metadata = rest.get(..len)?;
    // arrow-ipc panics instead of erroring on some malformed schemas (e.g. one
    // without a `fields` vector), and the frame comes from another process.
    let schema = catch_arrow_panic("failed to read the IPC schema: arrow panicked", || {
        Ok(arrow::ipc::convert::try_schema_from_flatbuffer_bytes(
            metadata,
        )?)
    })
    .ok()?;
    match schema.fields().as_ref() {
        [field] => Some(field.data_type().clone()),
        _ => None,
    }
}

/// Compute the average bandwidth in Mbps over the actual collection window.
///
/// Returns `None` when no messages were observed or the elapsed time is
/// non-positive, so the caller can render `<unknown>` instead of a misleading
/// `0.00` or a division-by-zero result.
fn bandwidth_mbps(total_bytes: u64, message_count: u64, elapsed_secs: f64) -> Option<f64> {
    // `elapsed_secs` comes from `Duration::as_secs_f64`, so it is finite and
    // non-negative; `<= 0.0` only rejects a zero window.
    if message_count == 0 || elapsed_secs <= 0.0 {
        return None;
    }
    Some((total_bytes as f64 * 8.0) / (elapsed_secs * 1_000_000.0))
}

fn format_arrow_type(data_type: &DataType) -> String {
    match data_type {
        DataType::Null => "Null".to_string(),
        DataType::Boolean => "Boolean".to_string(),
        DataType::Int8 => "Int8".to_string(),
        DataType::Int16 => "Int16".to_string(),
        DataType::Int32 => "Int32".to_string(),
        DataType::Int64 => "Int64".to_string(),
        DataType::UInt8 => "UInt8".to_string(),
        DataType::UInt16 => "UInt16".to_string(),
        DataType::UInt32 => "UInt32".to_string(),
        DataType::UInt64 => "UInt64".to_string(),
        DataType::Float16 => "Float16".to_string(),
        DataType::Float32 => "Float32".to_string(),
        DataType::Float64 => "Float64".to_string(),
        DataType::Timestamp(unit, tz) => {
            format!("Timestamp({:?}, {:?})", unit, tz)
        }
        DataType::Date32 => "Date32".to_string(),
        DataType::Date64 => "Date64".to_string(),
        DataType::Time32(unit) => format!("Time32({:?})", unit),
        DataType::Time64(unit) => format!("Time64({:?})", unit),
        DataType::Duration(unit) => format!("Duration({:?})", unit),
        DataType::Interval(unit) => format!("Interval({:?})", unit),
        DataType::Binary => "Binary".to_string(),
        DataType::FixedSizeBinary(size) => {
            format!("FixedSizeBinary({})", size)
        }
        DataType::LargeBinary => "LargeBinary".to_string(),
        DataType::Utf8 => "String".to_string(),
        DataType::LargeUtf8 => "LargeString".to_string(),
        DataType::List(field) => {
            format!("List<{}>", format_arrow_type(field.data_type()))
        }
        DataType::LargeList(field) => {
            format!("LargeList<{}>", format_arrow_type(field.data_type()))
        }
        DataType::FixedSizeList(field, size) => {
            format!(
                "FixedSizeList<{}>({})",
                format_arrow_type(field.data_type()),
                size
            )
        }
        DataType::Struct(fields) => {
            let field_strs: Vec<String> = fields
                .iter()
                .map(|f| format!("{}: {}", f.name(), format_arrow_type(f.data_type())))
                .collect();
            format!("Struct{{{}}}", field_strs.join(", "))
        }
        DataType::Union(_fields, mode) => {
            format!("Union({:?})", mode)
        }
        DataType::Dictionary(key_type, value_type) => {
            format!(
                "Dictionary<{}, {}>",
                format_arrow_type(key_type),
                format_arrow_type(value_type)
            )
        }
        DataType::Decimal128(precision, scale) => {
            format!("Decimal128({}, {})", precision, scale)
        }
        DataType::Decimal256(precision, scale) => {
            format!("Decimal256({}, {})", precision, scale)
        }
        DataType::Map(field, sorted) => {
            format!(
                "Map<{}>(sorted={})",
                format_arrow_type(field.data_type()),
                sorted
            )
        }
        DataType::RunEndEncoded(run_ends, values) => {
            format!(
                "RunEndEncoded<{}, {}>",
                format_arrow_type(run_ends.data_type()),
                format_arrow_type(values.data_type())
            )
        }
        _ => format!("{:?}", data_type),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ipc_stream(columns: Vec<(&str, arrow::array::ArrayRef)>) -> Vec<u8> {
        let batch = arrow::record_batch::RecordBatch::try_from_iter(columns).unwrap();
        let mut writer =
            arrow::ipc::writer::StreamWriter::try_new(Vec::new(), &batch.schema()).unwrap();
        writer.write(&batch).unwrap();
        writer.finish().unwrap();
        writer.into_inner().unwrap()
    }

    #[test]
    fn data_type_is_read_from_the_schema_alone() {
        let array: arrow::array::ArrayRef =
            std::sync::Arc::new(arrow::array::UInt8Array::from(vec![0u8; 4096]));
        let bytes = ipc_stream(vec![("data", array.clone())]);
        assert_eq!(decode_ipc_data_type(&bytes), Some(DataType::UInt8));

        // The record batch is never decoded: a stream cut off inside the
        // body still yields the schema's type.
        let truncated = &bytes[..bytes.len() - 2048];
        assert_eq!(decode_ipc_data_type(truncated), Some(DataType::UInt8));

        let two_columns = ipc_stream(vec![("a", array.clone()), ("b", array)]);
        assert_eq!(decode_ipc_data_type(&two_columns), None);
        assert_eq!(decode_ipc_data_type(b"not arrow"), None);
        assert_eq!(
            decode_ipc_data_type(&[0xff, 0xff, 0xff, 0xff, 0xff, 0xff]),
            None
        );
    }

    /// A schema message claiming a 1 TiB body: `StreamReader` would try to
    /// allocate that before reading anything and abort the process.
    #[test]
    fn a_huge_declared_body_length_is_not_allocated() {
        let schema = arrow::datatypes::Schema::new(vec![arrow::datatypes::Field::new(
            "data",
            DataType::UInt8,
            false,
        )]);
        let mut fbb = flatbuffers::FlatBufferBuilder::new();
        let fb_schema = arrow::ipc::convert::schema_to_fb_offset(&mut fbb, &schema);
        let mut message = arrow::ipc::MessageBuilder::new(&mut fbb);
        message.add_version(arrow::ipc::MetadataVersion::V5);
        message.add_header_type(arrow::ipc::MessageHeader::Schema);
        message.add_header(fb_schema.as_union_value());
        message.add_bodyLength(1 << 40);
        let message = message.finish();
        fbb.finish(message, None);
        let metadata = fbb.finished_data();

        let mut bytes = vec![0xff; 4];
        bytes.extend_from_slice(&i32::try_from(metadata.len()).unwrap().to_le_bytes());
        bytes.extend_from_slice(metadata);
        assert_eq!(decode_ipc_data_type(&bytes), Some(DataType::UInt8));
    }

    /// A schema message without a `fields` vector makes arrow-ipc panic
    /// (`fb.fields().unwrap()` in `convert.rs`) instead of returning an error.
    /// One such frame must leave the type unknown rather than take down the
    /// stats thread and fail the whole command (#3863).
    #[test]
    fn a_schema_that_makes_arrow_panic_leaves_the_type_unknown() {
        let mut fbb = flatbuffers::FlatBufferBuilder::new();
        let fb_schema = arrow::ipc::SchemaBuilder::new(&mut fbb).finish();
        let mut message = arrow::ipc::MessageBuilder::new(&mut fbb);
        message.add_version(arrow::ipc::MetadataVersion::V5);
        message.add_header_type(arrow::ipc::MessageHeader::Schema);
        message.add_header(fb_schema.as_union_value());
        let message = message.finish();
        fbb.finish(message, None);
        let metadata = fbb.finished_data();

        let mut bytes = vec![0xff; 4];
        bytes.extend_from_slice(&i32::try_from(metadata.len()).unwrap().to_le_bytes());
        bytes.extend_from_slice(metadata);
        assert_eq!(decode_ipc_data_type(&bytes), None);
    }

    #[test]
    fn calculate_hz_does_not_panic_on_oversized_window() {
        // Regression: a large `--duration` (the flag accepts any u64) made
        // `now - window` underflow the monotonic clock and panic with
        // "overflow when subtracting duration from instant". This is reachable
        // when the topic closes early so the full window never elapses.
        let stats = TopicStats::default();
        let now = Instant::now();
        {
            let mut ts = stats.timestamps.lock().unwrap();
            ts.push(now);
            ts.push(now + Duration::from_millis(100));
        }
        // A window far larger than any plausible monotonic-clock value.
        let hz = stats.calculate_hz(Duration::from_secs(u64::MAX));
        // With the cutoff before the monotonic epoch, every timestamp counts,
        // so a finite positive rate is returned instead of a panic.
        let hz = hz.expect("expected a frequency from two timestamps");
        assert!(hz.is_finite() && hz > 0.0, "unexpected hz: {hz}");
    }

    #[test]
    fn bandwidth_uses_actual_elapsed_window() {
        // 1_000_000 bytes over 1 s = 8 Mbps. A publisher that closed after 1 s
        // of a 10 s window must report 8 Mbps (over the actual elapsed time),
        // not 0.8 Mbps (over the full configured window).
        let mbps = bandwidth_mbps(1_000_000, 5, 1.0).expect("some bandwidth");
        assert!((mbps - 8.0).abs() < 1e-9, "expected 8 Mbps, got {mbps}");
    }

    #[test]
    fn bandwidth_unknown_without_messages_or_time() {
        assert_eq!(bandwidth_mbps(0, 0, 5.0), None, "no messages -> unknown");
        assert_eq!(bandwidth_mbps(100, 1, 0.0), None, "no elapsed -> unknown");
    }
}
