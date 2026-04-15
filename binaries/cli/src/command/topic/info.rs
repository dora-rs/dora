use std::{
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use clap::Args;
use dora_core::{
    config::InputMapping,
    topics::{open_zenoh_session, zenoh_output_publish_topic},
};
use dora_message::metadata::{ArrowTypeInfo, Metadata};
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

    let coordinator_addr = coordinator.coordinator_addr;

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

    // Collect messages for the specified duration.
    //
    // Wire format (matches DoraNode::send_output_sample): the zenoh payload
    // carries the raw arrow buffer and the attachment carries the bincode-
    // serialized Metadata.
    while Instant::now() < end_time {
        let Ok(sample) = tokio::time::timeout_at(deadline, subscriber.recv_async()).await else {
            break;
        };

        match sample {
            Ok(sample) => {
                let Some(attachment) = sample.attachment() else {
                    continue;
                };
                let metadata: Metadata = match bincode::deserialize(&attachment.to_bytes()) {
                    Ok(m) => m,
                    Err(_) => continue,
                };
                let data_size = sample.payload().len();
                stats.record(data_size, &metadata.type_info, Instant::now());
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
        println!("Type: {}", type_info.data_type);
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
