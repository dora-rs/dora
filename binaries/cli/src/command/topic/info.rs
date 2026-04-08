use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use adora_core::config::InputMapping;
use adora_message::{
    common::Timestamped, daemon_to_daemon::InterDaemonEvent, metadata::ArrowTypeInfo,
};
use arrow_schema::DataType;
use clap::Args;

use crate::{
    command::{Executable, default_tracing, topic::selector::TopicSelector},
    common::CoordinatorOptions,
};

/// Display detailed metadata of a topic.
///
/// Shows topic type, publisher, subscribers, and statistics (message count,
/// bandwidth, publishing frequency).
///
/// Examples:
///
/// Get info for a single topic:
///   adora topic info -d my-dataflow camera_node/image
///
/// Note: The dataflow descriptor must include the following snippet so that
/// runtime messages can be inspected (or messages must cross machine
/// boundaries so they are forwarded through zenoh):
///
/// ```yaml
/// _unstable_debug:
///   publish_all_messages_to_zenoh: true
/// ```
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
    data_type: Arc<Mutex<Option<ArrowTypeInfo>>>,
}

/// Maximum timestamps to keep for frequency calculation.
const MAX_TIMESTAMPS: usize = 10_000;

impl TopicStats {
    fn record(&self, data_size: usize, type_info: &ArrowTypeInfo, now: Instant) {
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
        *self.data_type.lock().unwrap_or_else(|e| e.into_inner()) = Some(type_info.clone());
    }

    fn calculate_hz(&self, window: Duration) -> Option<f64> {
        let timestamps = self.timestamps.lock().unwrap_or_else(|e| e.into_inner());
        if timestamps.len() < 2 {
            return None;
        }

        let now = Instant::now();
        let cutoff = now - window;
        let recent: Vec<_> = timestamps
            .iter()
            .filter(|&&t| t >= cutoff)
            .copied()
            .collect();

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
    let (dataflow_id, topics) = selector.resolve(&session)?;

    if topics.is_empty() {
        eyre::bail!("No topics specified");
    }

    if topics.len() > 1 {
        eyre::bail!("Please specify exactly one topic");
    }

    let topic = topics.into_iter().next().unwrap();

    // Get dataflow descriptor to find subscribers
    let (_, descriptor) = selector.dataflow.resolve(&session)?;

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
                        InterDaemonEvent::Output { metadata, data, .. } => {
                            let data_size = data.as_ref().map(|d| d.len()).unwrap_or(0);
                            stats_clone.record(data_size, &metadata.type_info, Instant::now());
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
    if let Some(type_info) = &data_type {
        println!("Type: {}", format_arrow_type(&type_info.data_type));
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
    if duration_secs > 0 {
        let bandwidth_mbps = (total_bytes as f64 * 8.0) / (duration_secs as f64 * 1_000_000.0);
        println!("  Bandwidth: {:.2} Mbps", bandwidth_mbps);
    } else {
        println!("  Bandwidth: <unknown>");
    }

    Ok(())
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
