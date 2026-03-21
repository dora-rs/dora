use std::path::PathBuf;

use aligned_vec::AVec;
use arrow::{
    array::{Array, AsArray, RecordBatch},
    ipc::reader::FileReader,
};
use clap::Args;
use dora_core::topics::{open_zenoh_session, zenoh_output_publish_topic};
use dora_message::{
    common::Timestamped,
    daemon_to_daemon::InterDaemonEvent,
    id::{DataId, NodeId},
    metadata::{ArrowTypeInfo, Metadata, MetadataParameters},
};
use eyre::{Context, bail, eyre};
use tokio::time::{Duration, sleep};
use uuid::Uuid;

use crate::{
    command::{
        Executable, default_tracing,
        topic::selector::DataflowSelector,
    },
    common::CoordinatorOptions,
};

/// Play back recorded dataflow messages.
///
/// This command reads an Arrow IPC recording file created with `dora record`
/// and publishes the messages back into a running dataflow. Messages are
/// replayed with their original timing, scaled by the rate parameter.
///
/// Examples:
///
/// Play back a recording at normal speed:
///   dora play recording.arrows
///
/// Play at 2x speed:
///   dora play recording.arrows --rate 2.0
///
/// Loop playback indefinitely:
///   dora play recording.arrows --loop
///
/// Note: The target dataflow must have the following configuration:
///
/// ```yaml
/// _unstable_debug:
///   publish_all_messages_to_zenoh: true
/// ```
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Play {
    /// Path to the recorded Arrow IPC file
    #[clap(value_name = "FILE")]
    pub file: PathBuf,

    /// Loop playback indefinitely
    #[clap(long, short = 'l')]
    pub loop_playback: bool,

    /// Playback rate multiplier (1.0 = real-time, 2.0 = 2x speed, 0.5 = half speed)
    #[clap(long, short = 'r', default_value = "1.0")]
    pub rate: f64,

    #[clap(flatten)]
    selector: DataflowSelector,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Play {
    async fn execute(self) -> eyre::Result<()> {
        default_tracing()?;

        if self.rate <= 0.0 {
            bail!("Rate must be positive");
        }

        play(
            self.coordinator,
            self.selector,
            self.file,
            self.loop_playback,
            self.rate,
        )
        .await
    }
}

async fn play(
    coordinator: CoordinatorOptions,
    selector: DataflowSelector,
    file: PathBuf,
    loop_playback: bool,
    rate: f64,
) -> eyre::Result<()> {
    // Open and validate recording file
    let file_handle = std::fs::File::open(&file)
        .with_context(|| format!("failed to open recording file: {}", file.display()))?;

    let reader = FileReader::try_new(file_handle, None)
        .context("failed to open Arrow IPC file")?;

    // Validate schema
    validate_schema(&reader)?;

    // Connect to coordinator and resolve dataflow
    let client = coordinator.connect_rpc().await?;
    let (dataflow_id, descriptor) = selector.resolve(&client).await?;

    if !descriptor.debug.publish_all_messages_to_zenoh {
        bail!(
            "Dataflow `{dataflow_id}` does not have `publish_all_messages_to_zenoh` enabled.\n\
            \n\
            Tip: Add the following snippet to your dataflow descriptor:\n\
            \n\
            ```\n\
            _unstable_debug:\n  publish_all_messages_to_zenoh: true\n\
            ```"
        );
    }

    println!("Playing back recording to dataflow {}", dataflow_id);

    let zenoh_session = open_zenoh_session(Some(coordinator.coordinator_addr))
        .await
        .context("failed to open zenoh session")?;

    // Wait for Zenoh peer connection before publishing
    print!("Connecting to daemon...");
    loop {
        let mut peers = zenoh_session.info().peers_zid().await;
        if peers.next().is_some() {
            break;
        }
        let mut routers = zenoh_session.info().routers_zid().await;
        if routers.next().is_some() {
            break;
        }
        sleep(Duration::from_millis(200)).await;
    }
    println!(" ok");

    // Read all batches into memory for potential looping
    let batches = reader.collect::<Result<Vec<_>, _>>()
        .context("failed to read batches from recording")?;

    if batches.is_empty() {
        println!("Recording is empty, nothing to play");
        return Ok(());
    }

    let mut playback_iteration = 0;
    loop {
        playback_iteration += 1;
        if loop_playback {
            println!("Playback iteration {}", playback_iteration);
        }

        let messages_played = play_batches(&batches, dataflow_id, &zenoh_session, rate).await?;

        println!("✓ Played {} messages", messages_played);

        if !loop_playback {
            break;
        }
    }

    Ok(())
}

fn validate_schema(reader: &FileReader<std::fs::File>) -> eyre::Result<()> {
    let schema = reader.schema();
    let expected_fields = ["timestamp_us", "node_id", "output_id", "data", "type_info", "metadata"];

    for field_name in expected_fields {
        if schema.field_with_name(field_name).is_err() {
            bail!("Invalid recording file: missing field '{field_name}'");
        }
    }

    Ok(())
}

async fn play_batches(
    batches: &[RecordBatch],
    dataflow_id: Uuid,
    zenoh_session: &zenoh::Session,
    rate: f64,
) -> eyre::Result<usize> {
    let mut message_count = 0;
    let mut last_timestamp_us: Option<u64> = None;

    for batch in batches {
        let timestamp_array = batch
            .column(0)
            .as_primitive::<arrow::datatypes::UInt64Type>();
        let node_id_array = batch.column(1).as_string::<i32>();
        let output_id_array = batch.column(2).as_string::<i32>();
        let data_array = batch.column(3).as_binary::<i32>();
        let type_info_array = batch.column(4).as_string::<i32>();
        let metadata_array = batch.column(5).as_string::<i32>();

        for i in 0..batch.num_rows() {
            let timestamp_us = timestamp_array.value(i);
            let node_id: NodeId = node_id_array.value(i).parse()
                .context("failed to parse node_id")?;
            let output_id = DataId::from(output_id_array.value(i));
            let data = if data_array.is_null(i) {
                None
            } else {
                Some(data_array.value(i).to_vec())
            };
            let type_info_json = type_info_array.value(i);
            let metadata_json = metadata_array.value(i);

            // Deserialize type_info and metadata
            let type_info: ArrowTypeInfo = serde_json::from_str(type_info_json)
                .context("failed to deserialize type_info")?;
            let parameters: MetadataParameters = serde_json::from_str(metadata_json)
                .context("failed to deserialize metadata")?;

            // Handle timing
            if let Some(last_ts) = last_timestamp_us {
                let delta_us = timestamp_us.saturating_sub(last_ts);
                let delay_us = (delta_us as f64 / rate) as u64;
                if delay_us > 0 {
                    sleep(Duration::from_micros(delay_us)).await;
                }
            }
            last_timestamp_us = Some(timestamp_us);

            // Create metadata with timestamp
            let timestamp = dora_core::uhlc::HLC::default().new_timestamp();
            let metadata = Metadata::from_parameters(timestamp, type_info, parameters);

            // Create topic and publish directly
            let topic = zenoh_output_publish_topic(dataflow_id, &node_id, &output_id);

            // Create InterDaemonEvent with aligned vec
            let data_message = data.map(|d| AVec::from_slice(128, &d));

            let event = InterDaemonEvent::Output {
                dataflow_id,
                node_id: node_id.clone(),
                output_id: output_id.clone(),
                metadata,
                data: data_message,
            };

            // Create timestamped event and serialize
            let timestamp = dora_core::uhlc::HLC::default().new_timestamp();
            let timestamped = Timestamped {
                inner: event,
                timestamp,
            };
            let payload = timestamped.serialize();

            // Publish directly using put on the session
            zenoh_session
                .put(&topic, payload)
                .await
                .map_err(|e| eyre!(e))
                .wrap_err_with(|| format!("failed to publish message for {node_id}/{output_id}"))?;

            message_count += 1;
        }
    }

    Ok(message_count)
}
