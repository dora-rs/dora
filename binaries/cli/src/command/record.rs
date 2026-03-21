use std::{path::PathBuf, sync::Arc, time::SystemTime};

use arrow::{
    array::{BinaryBuilder, RecordBatch, StringBuilder, UInt64Builder},
    datatypes::{DataType, Field, Schema},
    ipc::writer::FileWriter,
};
use clap::Args;
use dora_core::topics::{open_zenoh_session, zenoh_output_publish_topic};
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent, id::DataId};
use eyre::{Context, bail, eyre};
use tokio::{
    select,
    sync::mpsc::{UnboundedReceiver, UnboundedSender, unbounded_channel},
    task::JoinSet,
    time::{Duration, timeout},
};
use uuid::Uuid;

use crate::{
    command::{
        Executable, default_tracing,
        topic::selector::{TopicIdentifier, TopicSelector},
    },
    common::CoordinatorOptions,
};

/// Record dataflow messages to an Arrow IPC file.
///
/// This command subscribes to output topics from a running dataflow and
/// records all messages to a file in Arrow IPC format. The recordings can
/// later be played back using `dora play`.
///
/// Examples:
///
/// Record all outputs for 10 seconds:
///   dora record --all -d 10 -o recording.arrows
///
/// Record specific topics:
///   dora record -d my-dataflow robot/pose sensor/camera
///
/// Record continuously until Ctrl+C:
///   dora record --all -o recording.arrows
///
/// Note: The dataflow must have the following configuration:
///
/// ```yaml
/// _unstable_debug:
///   publish_all_messages_to_zenoh: true
/// ```
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Record {
    #[clap(flatten)]
    selector: TopicSelector,

    /// Output file path (defaults to recording-{timestamp}.arrows)
    #[clap(long, short = 'o', value_name = "FILE")]
    pub output_file: Option<PathBuf>,

    /// Duration to record in seconds (omit for continuous recording)
    #[clap(long, short = 't', value_name = "SECS")]
    pub duration: Option<f64>,

    /// Record all outputs from the dataflow (can also specify DATA arguments)
    #[clap(long, short = 'a')]
    pub all: bool,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

#[derive(Debug)]
struct RecordedMessage {
    timestamp_us: u64,
    node_id: String,
    output_id: String,
    data: Option<Vec<u8>>,
    type_info: String,
    metadata: String,
}

impl Executable for Record {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        // Validate arguments
        if !self.all && self.selector.data.is_empty() {
            bail!(
                "Either specify output topics (e.g., 'node/output') or use --all to record all outputs"
            );
        }

        record(
            self.coordinator,
            self.selector,
            self.output_file,
            self.duration,
        )
        .await
    }
}

async fn record(
    coordinator: CoordinatorOptions,
    selector: TopicSelector,
    output_file: Option<PathBuf>,
    duration: Option<f64>,
) -> eyre::Result<()> {
    let client = coordinator.connect_rpc().await?;
    let (dataflow_id, topics) = selector.resolve(&client).await?;

    if topics.is_empty() {
        bail!("No topics found to record");
    }

    println!(
        "Recording {} topics from dataflow {}",
        topics.len(),
        dataflow_id
    );
    for topic in &topics {
        println!("  - {}", topic);
    }

    // Generate output filename if not specified
    let output_path = output_file.unwrap_or_else(|| {
        let timestamp = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        PathBuf::from(format!("recording-{}.arrows", timestamp))
    });

    println!("Recording to: {}", output_path.display());

    let zenoh_session = open_zenoh_session(Some(coordinator.coordinator_addr))
        .await
        .context("failed to open zenoh session")?;

    // Wait for Zenoh peer connection before subscribing. Without this, short
    // recordings can finish before the Zenoh peers discover each other via
    // scouting, resulting in 0 captured messages.
    print!("Connecting to daemon...");
    let connected = loop {
        let mut peers = zenoh_session.info().peers_zid().await;
        if peers.next().is_some() {
            break true;
        }
        let mut routers = zenoh_session.info().routers_zid().await;
        if routers.next().is_some() {
            break true;
        }
        if timeout(Duration::from_millis(200), wait_for_ctrl_c())
            .await
            .is_ok()
        {
            break false;
        }
    };
    if !connected {
        println!(" interrupted");
        return Ok(());
    }
    println!(" ok");

    // Create channel for collecting messages from all topics
    let (tx, rx) = unbounded_channel::<RecordedMessage>();

    // Spawn subscriber tasks for each topic
    let mut join_set = JoinSet::new();
    for TopicIdentifier { node_id, data_id } in topics {
        join_set.spawn(subscribe_and_record(
            zenoh_session.clone(),
            dataflow_id,
            node_id,
            data_id,
            tx.clone(),
        ));
    }

    // Drop the sender so the writer knows when all senders are done
    drop(tx);

    // Start the recording writer task
    let writer_handle = tokio::spawn(write_recording(rx, output_path.clone()));

    // Handle duration timeout or wait for Ctrl+C
    if let Some(duration_secs) = duration {
        println!(
            "Recording for {} seconds (press Ctrl+C to stop early)",
            duration_secs
        );

        select! {
            _ = timeout(Duration::from_secs_f64(duration_secs), wait_for_ctrl_c()) => {
                println!("\nDuration elapsed, stopping recording...");
            }
            _ = wait_for_ctrl_c() => {
                println!("\nCtrl+C received, stopping recording...");
            }
        }
    } else {
        println!("Recording... (press Ctrl+C to stop)");
        wait_for_ctrl_c().await;
        println!("\nCtrl+C received, stopping recording...");
    }

    // Abort all subscriber tasks
    join_set.abort_all();

    // Wait for writer to finish
    let message_count = writer_handle.await??;

    println!(
        "✓ Recorded {} messages to {}",
        message_count,
        output_path.display()
    );

    Ok(())
}

async fn subscribe_and_record(
    zenoh_session: zenoh::Session,
    dataflow_id: Uuid,
    node_id: dora_message::id::NodeId,
    output_id: DataId,
    tx: UnboundedSender<RecordedMessage>,
) -> eyre::Result<()> {
    let subscribe_topic = zenoh_output_publish_topic(dataflow_id, &node_id, &output_id);

    let subscriber = zenoh_session
        .declare_subscriber(&subscribe_topic)
        .await
        .map_err(|e| eyre!(e))
        .wrap_err_with(|| format!("failed to subscribe to {node_id}/{output_id}"))?;

    while let Ok(sample) = subscriber.recv_async().await {
        let event = match Timestamped::deserialize_inter_daemon_event(&sample.payload().to_bytes())
        {
            Ok(event) => event,
            Err(_) => {
                eprintln!("Received invalid event from {node_id}/{output_id}");
                continue;
            }
        };

        match event.inner {
            InterDaemonEvent::Output { metadata, data, .. } => {
                // Get current timestamp in microseconds
                let timestamp_us = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_micros() as u64;

                // Serialize type_info and metadata to JSON
                let type_info_json = serde_json::to_string(&metadata.type_info)
                    .context("failed to serialize type_info")?;
                let metadata_json = serde_json::to_string(&metadata.parameters)
                    .context("failed to serialize metadata")?;

                let data_bytes = data.map(|data| data.to_vec());

                let message = RecordedMessage {
                    timestamp_us,
                    node_id: node_id.to_string(),
                    output_id: output_id.to_string(),
                    data: data_bytes,
                    type_info: type_info_json,
                    metadata: metadata_json,
                };

                if tx.send(message).is_err() {
                    // Receiver dropped, stop recording
                    break;
                }
            }
            InterDaemonEvent::OutputClosed { .. } => {
                break;
            }
            InterDaemonEvent::NodeFailed { .. } => {
                // Not relevant for recording
                continue;
            }
        }
    }

    Ok(())
}

async fn write_recording(
    mut rx: UnboundedReceiver<RecordedMessage>,
    output_path: PathBuf,
) -> eyre::Result<usize> {
    // Define Arrow schema for recorded messages
    let schema = Arc::new(Schema::new(vec![
        Field::new("timestamp_us", DataType::UInt64, false),
        Field::new("node_id", DataType::Utf8, false),
        Field::new("output_id", DataType::Utf8, false),
        Field::new("data", DataType::Binary, true),
        Field::new("type_info", DataType::Utf8, false),
        Field::new("metadata", DataType::Utf8, false),
    ]));

    // Create temporary file for atomic write
    let temp_path = output_path.with_extension("arrows.tmp");
    let file = std::fs::File::create(&temp_path)
        .with_context(|| format!("failed to create output file: {}", temp_path.display()))?;

    let mut writer =
        FileWriter::try_new(file, &schema).context("failed to create Arrow IPC writer")?;

    let mut message_count = 0;
    let mut batch_messages = Vec::new();
    const BATCH_SIZE: usize = 100;

    // Collect messages and write in batches
    while let Some(msg) = rx.recv().await {
        batch_messages.push(msg);

        if batch_messages.len() >= BATCH_SIZE {
            write_batch(&mut writer, &schema, &batch_messages)?;
            message_count += batch_messages.len();
            batch_messages.clear();
        }
    }

    // Write remaining messages
    if !batch_messages.is_empty() {
        write_batch(&mut writer, &schema, &batch_messages)?;
        message_count += batch_messages.len();
    }

    // Finish writing and close file
    writer.finish().context("failed to finish Arrow IPC file")?;
    drop(writer);

    // Move temp file to final location
    std::fs::rename(&temp_path, &output_path)
        .with_context(|| format!("failed to move temp file to {}", output_path.display()))?;

    Ok(message_count)
}

fn write_batch(
    writer: &mut FileWriter<std::fs::File>,
    schema: &Arc<Schema>,
    messages: &[RecordedMessage],
) -> eyre::Result<()> {
    let mut timestamp_builder = UInt64Builder::new();
    let mut node_id_builder = StringBuilder::new();
    let mut output_id_builder = StringBuilder::new();
    let mut data_builder = BinaryBuilder::new();
    let mut type_info_builder = StringBuilder::new();
    let mut metadata_builder = StringBuilder::new();

    for msg in messages {
        timestamp_builder.append_value(msg.timestamp_us);
        node_id_builder.append_value(&msg.node_id);
        output_id_builder.append_value(&msg.output_id);

        if let Some(ref data) = msg.data {
            data_builder.append_value(data);
        } else {
            data_builder.append_null();
        }

        type_info_builder.append_value(&msg.type_info);
        metadata_builder.append_value(&msg.metadata);
    }

    let batch = RecordBatch::try_new(
        schema.clone(),
        vec![
            Arc::new(timestamp_builder.finish()),
            Arc::new(node_id_builder.finish()),
            Arc::new(output_id_builder.finish()),
            Arc::new(data_builder.finish()),
            Arc::new(type_info_builder.finish()),
            Arc::new(metadata_builder.finish()),
        ],
    )
    .context("failed to create RecordBatch")?;

    writer
        .write(&batch)
        .context("failed to write batch to Arrow IPC file")?;

    Ok(())
}

async fn wait_for_ctrl_c() {
    tokio::signal::ctrl_c()
        .await
        .expect("failed to listen for Ctrl+C");
}
