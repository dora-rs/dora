use std::{collections::BTreeMap, io::Write, path::PathBuf, time::SystemTime};

use clap::Args;
use dora_message::{common::Timestamped, daemon_to_daemon::InterDaemonEvent, id::NodeId};
use dora_recording::{RecordEntry, RecordingHeader, RecordingWriter};
use eyre::{Context, bail};

use crate::command::{Executable, Run, default_tracing};

/// Record dataflow messages to a file for offline replay.
///
/// Injects a record node into the dataflow that captures all (or filtered)
/// topic data and writes it to a `.drec` recording file.
///
/// Examples:
///
///   Record all topics:
///     dora record dataflow.yml
///
///   Record specific topics:
///     dora record dataflow.yml --topics sensor/image,lidar/points
///
///   Specify output file:
///     dora record dataflow.yml -o capture.drec
///
///   Just generate the modified YAML:
///     dora record dataflow.yml --output-yaml modified.yml
///
///   Stream data through coordinator WS (for diskless targets):
///     dora record dataflow.yml --proxy
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Record {
    /// Path to the dataflow descriptor YAML file
    #[clap(value_name = "DATAFLOW_YAML")]
    file: String,

    /// Output recording file (default: recording_{timestamp}.drec)
    #[clap(short, long, value_name = "PATH")]
    output: Option<String>,

    /// Topics to record (comma-separated node/output, default: all outputs)
    #[clap(long, value_name = "TOPICS", value_delimiter = ',')]
    topics: Vec<String>,

    /// Just generate modified YAML, don't run
    #[clap(long, value_name = "PATH")]
    output_yaml: Option<String>,

    /// Stream data through coordinator WebSocket instead of recording on target.
    /// Useful when the target machine has no local disk.
    #[clap(long)]
    proxy: bool,

    #[clap(flatten)]
    coordinator: crate::common::CoordinatorOptions,
}

impl Executable for Record {
    fn execute(self) -> eyre::Result<()> {
        if self.proxy {
            // Proxy mode handles recording directly without delegating to Run,
            // so it needs its own tracing subscriber.
            default_tracing()?;
            run_record_proxy(self)
        } else {
            // Run::execute() sets up its own tracing subscriber.
            run_record(self)
        }
    }
}

/// Discover all `(node_id, output_id)` pairs from a parsed descriptor YAML.
fn discover_descriptor_outputs(
    descriptor: &serde_yaml::Value,
) -> eyre::Result<Vec<(String, String)>> {
    let nodes = descriptor
        .get("nodes")
        .and_then(|v| v.as_sequence())
        .ok_or_else(|| eyre::eyre!("descriptor has no nodes array"))?;

    let mut outputs = Vec::new();
    for node in nodes {
        let node_id = node.get("id").and_then(|v| v.as_str()).unwrap_or_default();
        if let Some(node_outputs) = node.get("outputs").and_then(|v| v.as_sequence()) {
            for output in node_outputs {
                if let Some(output_id) = output.as_str() {
                    outputs.push((node_id.to_string(), output_id.to_string()));
                }
            }
        }
    }

    if outputs.is_empty() {
        bail!(
            "no outputs found in descriptor\n\n  \
             hint: ensure nodes in the dataflow declare `outputs` in their YAML definition"
        );
    }
    Ok(outputs)
}

fn run_record(args: Record) -> eyre::Result<()> {
    let yaml_bytes =
        std::fs::read(&args.file).wrap_err_with(|| format!("failed to read {}", args.file))?;
    let mut descriptor: serde_yaml::Value =
        serde_yaml::from_slice(&yaml_bytes).wrap_err("failed to parse descriptor YAML")?;

    let all_outputs = discover_descriptor_outputs(&descriptor)?;
    let mut all_topics: BTreeMap<String, String> = BTreeMap::new();
    for (node_id, output_id) in &all_outputs {
        let topic = format!("{node_id}/{output_id}");
        // input_id on record node: sanitized to avoid / in IDs
        let input_id = format!("{node_id}___{output_id}");
        all_topics.insert(topic, input_id);
    }

    // Filter topics if --topics specified
    let topics: BTreeMap<String, String> = if args.topics.is_empty() {
        all_topics
    } else {
        let mut filtered = BTreeMap::new();
        for requested in &args.topics {
            match all_topics.get(requested) {
                Some(input_id) => {
                    filtered.insert(requested.clone(), input_id.clone());
                }
                None => bail!(
                    "topic `{requested}` not found in descriptor. Available: {}",
                    all_topics.keys().cloned().collect::<Vec<_>>().join(", ")
                ),
            }
        }
        filtered
    };

    let output_file = match &args.output {
        Some(p) => p.clone(),
        None => {
            let ts = chrono::Local::now().format("%Y%m%d_%H%M%S");
            format!("recording_{ts}.drec")
        }
    };
    let cwd = std::env::current_dir().wrap_err("failed to get current directory")?;
    let output_path = dunce::canonicalize(&cwd).unwrap_or(cwd).join(&output_file);

    // Find record node binary
    let record_node_bin = find_record_node_binary()?;

    // Build topic map JSON: { "input_id": "node/output" }
    let topic_map: BTreeMap<&str, &str> = topics
        .iter()
        .map(|(topic, input_id)| (input_id.as_str(), topic.as_str()))
        .collect();
    let topics_json =
        serde_json::to_string(&topic_map).wrap_err("failed to serialize topic map")?;

    // Build inputs mapping for the record node YAML entry
    let mut inputs_mapping = serde_yaml::Mapping::new();
    for (topic, input_id) in &topics {
        inputs_mapping.insert(
            serde_yaml::Value::String(input_id.clone()),
            serde_yaml::Value::String(topic.clone()),
        );
    }

    // Build env vars
    let mut env_mapping = serde_yaml::Mapping::new();
    env_mapping.insert(
        serde_yaml::Value::String("DORA_RECORD_FILE".to_string()),
        serde_yaml::Value::String(output_path.to_string_lossy().to_string()),
    );
    env_mapping.insert(
        serde_yaml::Value::String("DORA_RECORD_TOPICS".to_string()),
        serde_yaml::Value::String(topics_json),
    );
    env_mapping.insert(
        serde_yaml::Value::String("DORA_RECORD_DESCRIPTOR".to_string()),
        serde_yaml::Value::String(String::from_utf8_lossy(&yaml_bytes).to_string()),
    );

    // Build the record node YAML entry
    let mut record_node = serde_yaml::Mapping::new();
    record_node.insert(
        serde_yaml::Value::String("id".to_string()),
        serde_yaml::Value::String("__dora_record__".to_string()),
    );
    record_node.insert(
        serde_yaml::Value::String("path".to_string()),
        serde_yaml::Value::String(record_node_bin.to_string_lossy().to_string()),
    );
    record_node.insert(
        serde_yaml::Value::String("inputs".to_string()),
        serde_yaml::Value::Mapping(inputs_mapping),
    );
    record_node.insert(
        serde_yaml::Value::String("env".to_string()),
        serde_yaml::Value::Mapping(env_mapping),
    );

    // Append record node to descriptor
    let nodes_mut = descriptor
        .get_mut("nodes")
        .and_then(|v| v.as_sequence_mut())
        .ok_or_else(|| eyre::eyre!("descriptor has no nodes array"))?;
    nodes_mut.push(serde_yaml::Value::Mapping(record_node));

    let modified_yaml =
        serde_yaml::to_string(&descriptor).wrap_err("failed to serialize modified descriptor")?;

    // If --output-yaml, just write and exit
    if let Some(yaml_output_path) = args.output_yaml {
        std::fs::write(&yaml_output_path, &modified_yaml)
            .wrap_err_with(|| format!("failed to write {yaml_output_path}"))?;
        eprintln!("Modified descriptor written to {yaml_output_path}");
        return Ok(());
    }

    // The tempfile lives in /tmp but descriptor-relative paths
    // (`build:` cargo, node binaries) must still resolve against the
    // original source dir, so pass it as an explicit `working_dir`
    // override to `Run`.
    let source_dir = PathBuf::from(&args.file)
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    let mut tmp =
        tempfile::NamedTempFile::with_suffix(".yml").wrap_err("failed to create temp file")?;
    tmp.write_all(modified_yaml.as_bytes())?;
    tmp.flush()?;
    let tmp_path = tmp.into_temp_path();

    eprintln!("Recording {} topics to {output_file}", topics.len());
    eprintln!(
        "Topics: {}",
        format_topics(topics.keys().cloned().collect())
    );
    eprintln!();

    Run::new(tmp_path.to_string_lossy().to_string())
        .with_working_dir(source_dir)
        .execute()
}

fn run_record_proxy(args: Record) -> eyre::Result<()> {
    let yaml_bytes =
        std::fs::read(&args.file).wrap_err_with(|| format!("failed to read {}", args.file))?;

    let descriptor: serde_yaml::Value =
        serde_yaml::from_slice(&yaml_bytes).wrap_err("failed to parse descriptor YAML")?;

    let all_topics = discover_descriptor_outputs(&descriptor)?;

    // Filter topics if --topics specified
    let topics: Vec<(String, String)> = if args.topics.is_empty() {
        all_topics
    } else {
        let mut filtered = Vec::new();
        for requested in &args.topics {
            let parts: Vec<&str> = requested.splitn(2, '/').collect();
            if parts.len() != 2 {
                bail!("invalid topic format `{requested}`, expected `node/output`");
            }
            let (node, output) = (parts[0].to_string(), parts[1].to_string());
            if !all_topics.iter().any(|(n, o)| n == &node && o == &output) {
                let available: Vec<String> =
                    all_topics.iter().map(|(n, o)| format!("{n}/{o}")).collect();
                bail!(
                    "topic `{requested}` not found in descriptor. Available: {}",
                    available.join(", ")
                );
            }
            filtered.push((node, output));
        }
        filtered
    };

    let output_file = match &args.output {
        Some(p) => {
            // Resolve to filename only if user provided a bare name, otherwise use as-is
            let path = PathBuf::from(p);
            path.to_string_lossy().to_string()
        }
        None => {
            let ts = chrono::Local::now().format("%Y%m%d_%H%M%S");
            format!("recording_{ts}.drec")
        }
    };

    eprintln!("Proxy recording {} topics to {output_file}", topics.len());
    eprintln!(
        "Topics: {}",
        format_topics(topics.iter().map(|(n, o)| format!("{n}/{o}")).collect())
    );
    eprintln!("Waiting for dataflow to start...");

    // Connect to coordinator and wait for the dataflow
    let session = args.coordinator.connect()?;

    // List running dataflows, find the one that matches our descriptor name
    // For proxy mode, the user should have already started the dataflow
    let list_raw = session
        .request(
            &serde_json::to_vec(&dora_message::cli_to_coordinator::ControlRequest::List)
                .wrap_err("failed to serialize List request")?,
        )
        .wrap_err("failed to list dataflows")?;
    let list_reply: dora_message::coordinator_to_cli::ControlRequestReply =
        serde_json::from_slice(&list_raw).wrap_err("failed to parse list reply")?;

    let active_ids = match list_reply {
        dora_message::coordinator_to_cli::ControlRequestReply::DataflowList(list) => {
            list.get_active()
        }
        _ => bail!("unexpected reply to List"),
    };

    if active_ids.is_empty() {
        bail!(
            "no running dataflows found. Start a dataflow first with `dora start`, \
             then use `dora record --proxy` to record it."
        );
    }

    // Use the first active dataflow (or let user pick if multiple)
    let dataflow_id = if active_ids.len() == 1 {
        active_ids[0].uuid
    } else {
        let choices: Vec<String> = active_ids.iter().map(|d| d.to_string()).collect();
        let selected = inquire::Select::new("Select dataflow to record:", choices)
            .prompt()
            .wrap_err("dataflow selection cancelled")?;
        active_ids
            .iter()
            .find(|d| d.to_string() == selected)
            .unwrap()
            .uuid
    };

    // Subscribe to topics via WS
    let ws_topics: Vec<_> = topics
        .iter()
        .map(|(n, o)| -> eyre::Result<_> {
            Ok((
                n.parse::<NodeId>()
                    .map_err(|e| eyre::eyre!("invalid node ID in topic: {e}"))?,
                o.clone().into(),
            ))
        })
        .collect::<eyre::Result<Vec<_>>>()?;
    let (_subscription_id, data_rx) = session.subscribe_topics(dataflow_id, ws_topics)?;

    // Set up recording writer
    let start_nanos = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    let header = RecordingHeader {
        version: 1,
        start_nanos,
        dataflow_id,
        descriptor_yaml: yaml_bytes,
    };

    let file = std::fs::File::create(&output_file)
        .wrap_err_with(|| format!("failed to create {output_file}"))?;
    let mut writer = RecordingWriter::new(file, &header)?;

    eprintln!("Recording... (press Ctrl-C to stop)");

    // Set up Ctrl-C handler
    let (stop_tx, stop_rx) = std::sync::mpsc::channel();
    ctrlc::set_handler(move || {
        let _ = stop_tx.send(());
    })
    .wrap_err("failed to set ctrl-c handler")?;

    let mut msg_count: u64 = 0;
    loop {
        // Check for Ctrl-C
        if stop_rx.try_recv().is_ok() {
            break;
        }

        match data_rx.recv_timeout(std::time::Duration::from_millis(100)) {
            Ok(Ok(payload)) => {
                // The payload is already `Timestamped<InterDaemonEvent>` bincode bytes.
                // Parse it to extract node_id and output_id for the recording entry.
                let event = match Timestamped::deserialize_inter_daemon_event(&payload) {
                    Ok(e) => e,
                    Err(_) => continue,
                };

                let (node_id, output_id) = match &event.inner {
                    InterDaemonEvent::Output {
                        node_id, output_id, ..
                    } => (node_id.to_string(), output_id.to_string()),
                    InterDaemonEvent::OutputClosed { .. } => continue,
                };

                let now_nanos = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64;

                let entry = RecordEntry {
                    node_id,
                    output_id,
                    timestamp_offset_nanos: now_nanos.saturating_sub(start_nanos),
                    event_bytes: payload,
                };
                writer.write_entry(&entry)?;
                msg_count += 1;

                if msg_count.is_multiple_of(100) {
                    writer.flush()?;
                }
            }
            Ok(Err(_)) => continue,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => continue,
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        }
    }

    let footer = writer.finish()?;
    eprintln!(
        "Recording complete: {} messages, {:.2} MB",
        footer.total_messages,
        footer.total_bytes as f64 / 1_048_576.0
    );

    Ok(())
}

fn find_record_node_binary() -> eyre::Result<PathBuf> {
    super::node_binary::find("dora-record-node", "dora-record-node")
}

fn format_topics(topics: Vec<String>) -> String {
    if topics.len() <= 5 {
        topics.join(", ")
    } else {
        format!("{} topics", topics.len())
    }
}
