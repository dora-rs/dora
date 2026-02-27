use std::{
    collections::BTreeSet,
    fs::File,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    time::{Instant, SystemTime},
};

use adora_core::topics::{open_zenoh_session, zenoh_output_publish_topic};
use adora_recording::{RecordEntry, RecordingHeader, RecordingWriter};
use clap::Args;
use eyre::{Context, eyre};
use tokio::{runtime::Builder, task::JoinSet};

use crate::{
    command::{
        Executable, default_tracing,
        topic::selector::{TopicIdentifier, TopicSelector},
    },
    common::CoordinatorOptions,
};

/// Record dataflow messages to a file for offline replay.
///
/// Captures all (or filtered) topic data from a running dataflow via the
/// Zenoh debug tap and streams it to an `.adorec` recording file.
///
/// The dataflow descriptor must include:
///
/// ```yaml
/// _unstable_debug:
///   publish_all_messages_to_zenoh: true
/// ```
///
/// Examples:
///
///   Record all topics:
///     adora record -d my-dataflow
///
///   Record specific topics:
///     adora record -d my-dataflow sensor/image lidar/points
///
///   Specify output file:
///     adora record -d my-dataflow -o capture.adorec
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Record {
    #[clap(flatten)]
    selector: TopicSelector,

    /// Output file path (default: {name}_{timestamp}.adorec)
    #[clap(short, long, value_name = "PATH")]
    output: Option<String>,

    #[clap(flatten)]
    coordinator: CoordinatorOptions,
}

impl Executable for Record {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        run_record(self.coordinator, self.selector, self.output)
    }
}

fn run_record(
    coordinator: CoordinatorOptions,
    selector: TopicSelector,
    output_path: Option<String>,
) -> eyre::Result<()> {
    let session = coordinator.connect()?;
    let (dataflow_id, topics) = selector.resolve(&session)?;

    // Get descriptor YAML for embedding in recording
    let descriptor_yaml = {
        use adora_message::{
            cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply,
        };
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Info {
                    dataflow_uuid: dataflow_id,
                })
                .unwrap(),
            )
            .wrap_err("failed to send info request")?;
        let reply: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match reply {
            ControlRequestReply::DataflowInfo { descriptor, .. } => {
                serde_yaml::to_string(&descriptor)
                    .unwrap_or_default()
                    .into_bytes()
            }
            _ => vec![],
        }
    };

    let output_file = match output_path {
        Some(p) => p,
        None => {
            let ts = chrono::Local::now().format("%Y%m%d_%H%M%S");
            format!("recording_{ts}.adorec")
        }
    };

    let start_nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    let header = RecordingHeader {
        version: 1,
        start_nanos,
        dataflow_id,
        descriptor_yaml,
    };

    let file =
        File::create(&output_file).wrap_err_with(|| format!("failed to create {output_file}"))?;

    let writer =
        RecordingWriter::new(file, &header).wrap_err("failed to write recording header")?;
    let writer = Arc::new(std::sync::Mutex::new(writer));

    let running = Arc::new(AtomicBool::new(true));
    let msg_count = Arc::new(AtomicU64::new(0));

    // Set up Ctrl-C handler
    {
        let running = running.clone();
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
        })
        .wrap_err("failed to set Ctrl-C handler")?;
    }

    eprintln!("Recording to {output_file}");
    eprintln!("Topics: {}", format_topics(&topics));
    eprintln!("Press Ctrl-C to stop recording.\n");

    let start_time = Instant::now();

    let rt = Builder::new_multi_thread()
        .enable_all()
        .build()
        .context("tokio runtime failed")?;

    rt.block_on(async {
        let zenoh_session = open_zenoh_session(Some(coordinator.coordinator_addr))
            .await
            .context("failed to open zenoh session")?;

        let mut join_set = JoinSet::new();
        for TopicIdentifier { node_id, data_id } in &topics {
            let zenoh_session = zenoh_session.clone();
            let writer = writer.clone();
            let running = running.clone();
            let msg_count = msg_count.clone();
            let df_id = dataflow_id;
            let node_id = node_id.clone();
            let data_id = data_id.clone();
            let start = start_nanos;

            join_set.spawn(async move {
                let subscribe_topic = zenoh_output_publish_topic(df_id, &node_id, &data_id);
                let subscriber = zenoh_session
                    .declare_subscriber(&subscribe_topic)
                    .await
                    .map_err(|e| eyre!(e))
                    .wrap_err_with(|| format!("failed to subscribe to {node_id}/{data_id}"))?;

                while running.load(Ordering::SeqCst) {
                    let result = tokio::time::timeout(
                        std::time::Duration::from_millis(200),
                        subscriber.recv_async(),
                    )
                    .await;

                    let sample = match result {
                        Ok(Ok(sample)) => sample,
                        Ok(Err(_)) => break,
                        Err(_) => continue, // timeout, check running flag
                    };

                    let now_nanos = SystemTime::now()
                        .duration_since(SystemTime::UNIX_EPOCH)
                        .unwrap()
                        .as_nanos() as u64;

                    let entry = RecordEntry {
                        node_id: node_id.to_string(),
                        output_id: data_id.to_string(),
                        timestamp_offset_nanos: now_nanos.saturating_sub(start),
                        event_bytes: sample.payload().to_bytes().to_vec(),
                    };

                    let mut w = writer.lock().unwrap();
                    w.write_entry(&entry).wrap_err("failed to write record")?;
                    drop(w);

                    msg_count.fetch_add(1, Ordering::Relaxed);
                }

                Ok::<(), eyre::Error>(())
            });
        }

        // Wait for all tasks to finish
        while let Some(res) = join_set.join_next().await {
            match res {
                Ok(Ok(())) => {}
                Ok(Err(e)) => eprintln!("Error: {e:#}"),
                Err(e) => eprintln!("Task error: {e}"),
            }
        }

        Ok::<(), eyre::Error>(())
    })?;

    // Write footer and print summary
    let elapsed = start_time.elapsed();
    let total_msgs = msg_count.load(Ordering::Relaxed);

    let writer = Arc::try_unwrap(writer)
        .map_err(|_| eyre!("writer still shared"))?
        .into_inner()
        .unwrap();
    let footer = writer
        .finish()
        .wrap_err("failed to write recording footer")?;

    eprintln!("\nRecording complete:");
    eprintln!("  Messages: {total_msgs}");
    eprintln!("  Duration: {:.1}s", elapsed.as_secs_f64());
    eprintln!("  Bytes:    {}", footer.total_bytes);
    eprintln!("  File:     {output_file}");

    Ok(())
}

fn format_topics(topics: &BTreeSet<TopicIdentifier>) -> String {
    if topics.len() <= 5 {
        topics
            .iter()
            .map(|t: &TopicIdentifier| t.to_string())
            .collect::<Vec<_>>()
            .join(", ")
    } else {
        format!("{} topics", topics.len())
    }
}
