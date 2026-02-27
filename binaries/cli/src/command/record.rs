use std::{collections::BTreeMap, io::Write, path::PathBuf};

use clap::Args;
use eyre::{Context, bail};

use crate::command::{Executable, Run, default_tracing};

/// Record dataflow messages to a file for offline replay.
///
/// Injects a record node into the dataflow that captures all (or filtered)
/// topic data and writes it to an `.adorec` recording file.
///
/// Examples:
///
///   Record all topics:
///     adora record dataflow.yml
///
///   Record specific topics:
///     adora record dataflow.yml --topics sensor/image,lidar/points
///
///   Specify output file:
///     adora record dataflow.yml -o capture.adorec
///
///   Just generate the modified YAML:
///     adora record dataflow.yml --output-yaml modified.yml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Record {
    /// Path to the dataflow descriptor YAML file
    #[clap(value_name = "DATAFLOW_YAML")]
    file: String,

    /// Output recording file (default: recording_{timestamp}.adorec)
    #[clap(short, long, value_name = "PATH")]
    output: Option<String>,

    /// Topics to record (comma-separated node/output, default: all outputs)
    #[clap(long, value_name = "TOPICS", value_delimiter = ',')]
    topics: Vec<String>,

    /// Just generate modified YAML, don't run
    #[clap(long, value_name = "PATH")]
    output_yaml: Option<String>,
}

impl Executable for Record {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        run_record(self)
    }
}

fn run_record(args: Record) -> eyre::Result<()> {
    let yaml_bytes =
        std::fs::read(&args.file).wrap_err_with(|| format!("failed to read {}", args.file))?;
    let mut descriptor: serde_yaml::Value =
        serde_yaml::from_slice(&yaml_bytes).wrap_err("failed to parse descriptor YAML")?;

    // Discover all node outputs from the YAML
    let nodes = descriptor
        .get("nodes")
        .and_then(|v| v.as_sequence())
        .ok_or_else(|| eyre::eyre!("descriptor has no nodes array"))?;

    let mut all_topics: BTreeMap<String, String> = BTreeMap::new();
    for node in nodes {
        let node_id = node.get("id").and_then(|v| v.as_str()).unwrap_or_default();
        if let Some(outputs) = node.get("outputs").and_then(|v| v.as_sequence()) {
            for output in outputs {
                if let Some(output_id) = output.as_str() {
                    let topic = format!("{node_id}/{output_id}");
                    // input_id on record node: sanitized to avoid / in IDs
                    let input_id = format!("{node_id}___{output_id}");
                    all_topics.insert(topic, input_id);
                }
            }
        }
    }

    if all_topics.is_empty() {
        bail!("no outputs found in descriptor");
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
            format!("recording_{ts}.adorec")
        }
    };
    let output_path = dunce::canonicalize(std::env::current_dir()?)
        .unwrap_or_else(|_| std::env::current_dir().unwrap())
        .join(&output_file);

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
        serde_yaml::Value::String("ADORA_RECORD_FILE".to_string()),
        serde_yaml::Value::String(output_path.to_string_lossy().to_string()),
    );
    env_mapping.insert(
        serde_yaml::Value::String("ADORA_RECORD_TOPICS".to_string()),
        serde_yaml::Value::String(topics_json),
    );
    env_mapping.insert(
        serde_yaml::Value::String("ADORA_RECORD_DESCRIPTOR".to_string()),
        serde_yaml::Value::String(String::from_utf8_lossy(&yaml_bytes).to_string()),
    );

    // Build the record node YAML entry
    let mut record_node = serde_yaml::Mapping::new();
    record_node.insert(
        serde_yaml::Value::String("id".to_string()),
        serde_yaml::Value::String("__adora_record__".to_string()),
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

    // Write to temp file and run
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

    let run = Run::new(tmp_path.to_string_lossy().to_string());
    run.execute()
}

fn find_record_node_binary() -> eyre::Result<PathBuf> {
    // Check next to current executable first
    if let Ok(exe) = std::env::current_exe() {
        let dir = exe.parent().unwrap_or(std::path::Path::new("."));
        let candidate = dir.join("adora-record-node");
        if candidate.exists() {
            return Ok(candidate);
        }
        #[cfg(target_os = "windows")]
        {
            let candidate = dir.join("adora-record-node.exe");
            if candidate.exists() {
                return Ok(candidate);
            }
        }
    }

    // Check PATH
    if let Ok(path) = which::which("adora-record-node") {
        return Ok(path);
    }

    // Check cargo target directory (development)
    let cargo_target = std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("target"));
    for profile in ["debug", "release"] {
        let candidate = cargo_target.join(profile).join("adora-record-node");
        if candidate.exists() {
            return Ok(dunce::canonicalize(candidate)?);
        }
    }

    bail!(
        "could not find `adora-record-node` binary.\n\
         Build it with: cargo build -p adora-record-node"
    )
}

fn format_topics(topics: Vec<String>) -> String {
    if topics.len() <= 5 {
        topics.join(", ")
    } else {
        format!("{} topics", topics.len())
    }
}
