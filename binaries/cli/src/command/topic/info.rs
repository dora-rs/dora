use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use arrow_schema::DataType;
use clap::Args;
use dora_core::config::InputMapping;
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent};

use crate::{
    command::{Executable, default_tracing, topic::selector::TopicSelector},
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
/// _unstable_debug:
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
        for (input_id, input) in &node.inputs {
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
                        InterDaemonEvent::Output { data, .. } => {
                            let data_size = data.as_ref().map(|d| d.len()).unwrap_or(0);
                            // The payload is a self-describing Arrow IPC stream;
                            // read its data type from the decoded array (best
                            // effort — a malformed stream just leaves it unknown).
                            let data_type =
                                data.as_ref().and_then(|bytes| decode_ipc_data_type(bytes));
                            stats_clone.record(data_size, data_type, Instant::now());
                        }
                        InterDaemonEvent::OutputClosed { .. } => break,
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

/// Decode just the schema/data type from a self-describing Arrow IPC stream.
/// Returns `None` for a malformed or empty stream (the type is left unknown).
fn decode_ipc_data_type(bytes: &[u8]) -> Option<DataType> {
    use arrow::ipc::reader::StreamReader;
    use std::io::Cursor;

    let mut reader = StreamReader::try_new(Cursor::new(bytes), None).ok()?;
    let batch = reader.next()?.ok()?;
    if batch.num_columns() != 1 {
        return None;
    }
    Some(batch.column(0).data_type().clone())
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
