use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use arrow_schema::DataType;
use clap::Args;
use dora_core::{
    config::InputMapping,
    topics::{open_zenoh_session, zenoh_output_publish_topic},
};
use dora_message::{
    common::Timestamped, daemon_to_daemon::InterDaemonEvent, metadata::ArrowTypeInfo,
};
use eyre::{Context, eyre};

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
///   dora topic info -d my-dataflow camera_node/image
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
    #[clap(long, default_value_t = 5)]
    duration: u64,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Info {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        info(self.coordinator, self.selector, self.duration).await
    }
}

#[derive(Clone, Default)]
struct TopicStats {
    message_count: Arc<Mutex<u64>>,
    total_bytes: Arc<Mutex<u64>>,
    timestamps: Arc<Mutex<Vec<Instant>>>,
    data_type: Arc<Mutex<Option<ArrowTypeInfo>>>,
}

impl TopicStats {
    fn record(&self, data_size: usize, type_info: &ArrowTypeInfo, now: Instant) {
        *self.message_count.lock().unwrap() += 1;
        *self.total_bytes.lock().unwrap() += data_size as u64;
        self.timestamps.lock().unwrap().push(now);
        *self.data_type.lock().unwrap() = Some(type_info.clone());
    }

    fn calculate_hz(&self, window: Duration) -> Option<f64> {
        let timestamps = self.timestamps.lock().unwrap();
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

async fn info(
    coordinator: CoordinatorOptions,
    selector: TopicSelector,
    duration_secs: u64,
) -> eyre::Result<()> {
    let client = coordinator.connect_rpc().await?;
    let (dataflow_id, topics) = selector.resolve(&client).await?;

    if topics.is_empty() {
        eyre::bail!("No topics specified");
    }

    if topics.len() > 1 {
        eyre::bail!("Please specify exactly one topic");
    }

    let topic = topics.into_iter().next().unwrap();

    // Get dataflow descriptor to find subscribers
    let (_, descriptor) = selector.dataflow.resolve(&client).await?;

    // Find subscribers
    let mut subscribers = Vec::new();
    for node in &descriptor.nodes {
        for (input_id, input) in &node.inputs {
            if let InputMapping::User(user) = &input.mapping {
                if user.source == topic.node_id && user.output == topic.data_id {
                    subscribers.push(format!("{}/{}", node.id, input_id));
                }
            }
        }
    }

    // Collect statistics by subscribing to messages
    let stats = TopicStats::default();

    let (coordinator_addr, _) = coordinator.resolve();

    let zenoh_session = open_zenoh_session(Some(coordinator_addr))
        .await
        .context("failed to open zenoh session")?;

    let subscribe_topic = zenoh_output_publish_topic(dataflow_id, &topic.node_id, &topic.data_id);
    let subscriber = zenoh_session
        .declare_subscriber(subscribe_topic)
        .await
        .map_err(|e| eyre!(e))
        .wrap_err_with(|| format!("failed to subscribe to {}", topic))?;

    let start_time = Instant::now();
    let duration = Duration::from_secs(duration_secs);
    let end_time = start_time + duration;
    let deadline = tokio::time::Instant::from_std(end_time);

    // Collect messages for the specified duration
    while Instant::now() < end_time {
        let Ok(sample) = tokio::time::timeout_at(deadline, subscriber.recv_async()).await else {
            break;
        };

        match sample {
            Ok(sample) => {
                let event =
                    match Timestamped::deserialize_inter_daemon_event(&sample.payload().to_bytes())
                    {
                        Ok(event) => event,
                        Err(_) => continue,
                    };

                match event.inner {
                    InterDaemonEvent::Output { metadata, data, .. } => {
                        let data_size = data.as_ref().map(|d| d.len()).unwrap_or(0);
                        let now = Instant::now();
                        stats.record(data_size, &metadata.type_info, now);
                    }
                    InterDaemonEvent::OutputClosed { .. } => {
                        break;
                    }
                    InterDaemonEvent::NodeFailed { .. } => {
                        // Node failed, stop collecting statistics
                        break;
                    }
                }
            }
            Err(_) => break,
        }
    }

    // Display the information
    let message_count = *stats.message_count.lock().unwrap();
    let total_bytes = *stats.total_bytes.lock().unwrap();
    let data_type = stats.data_type.lock().unwrap().clone();
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
