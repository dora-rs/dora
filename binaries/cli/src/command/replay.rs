use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::Write,
    path::PathBuf,
};

use adora_recording::RecordingReader;
use clap::Args;
use eyre::{Context, bail};

use crate::command::{Executable, Run, default_tracing};

/// Replay a recorded dataflow from an `.adorec` file.
///
/// Reads a recording, identifies which nodes produced the recorded data,
/// replaces them with replay nodes, and runs the modified dataflow.
/// Downstream nodes receive replayed data identically to live data.
///
/// Examples:
///
///   Replay at original speed:
///     adora replay recording.adorec
///
///   Replay at 2x speed:
///     adora replay recording.adorec --speed 2.0
///
///   Replay as fast as possible:
///     adora replay recording.adorec --speed 0
///
///   Only replace specific nodes:
///     adora replay recording.adorec --replace sensor,camera
///
///   Just generate the modified YAML:
///     adora replay recording.adorec --output-yaml modified.yml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Replay {
    /// Path to the `.adorec` recording file
    #[clap(value_name = "FILE")]
    file: String,

    /// Nodes to replace with replay (comma-separated). Default: all recorded source nodes.
    #[clap(long, value_name = "NODE_IDS", value_delimiter = ',')]
    replace: Vec<String>,

    /// Playback speed multiplier (default: 1.0, 0 = fast as possible)
    #[clap(long, default_value = "1.0")]
    speed: f64,

    /// Loop the recording
    #[clap(long)]
    r#loop: bool,

    /// Just generate modified YAML, don't run
    #[clap(long, value_name = "PATH")]
    output_yaml: Option<String>,
}

impl Executable for Replay {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        run_replay(self)
    }
}

fn run_replay(args: Replay) -> eyre::Result<()> {
    let file = File::open(&args.file).wrap_err_with(|| format!("failed to open {}", args.file))?;
    let mut reader = RecordingReader::open(file).wrap_err("failed to read recording header")?;

    let header = reader.header().clone();
    let descriptor_yaml = std::str::from_utf8(&header.descriptor_yaml)
        .wrap_err("invalid descriptor YAML in recording")?;

    let mut descriptor: serde_yaml::Value =
        serde_yaml::from_str(descriptor_yaml).wrap_err("failed to parse descriptor YAML")?;

    // Discover which nodes produced recorded data
    let mut recorded_nodes: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    while let Some(entry) = reader.next_entry()? {
        recorded_nodes
            .entry(entry.node_id)
            .or_default()
            .insert(entry.output_id);
    }

    if recorded_nodes.is_empty() {
        bail!("recording contains no messages");
    }

    // Determine which nodes to replace
    let nodes_to_replace: BTreeSet<String> = if args.replace.is_empty() {
        recorded_nodes.keys().cloned().collect()
    } else {
        let requested: BTreeSet<String> = args.replace.into_iter().collect();
        for name in &requested {
            if !recorded_nodes.contains_key(name) {
                bail!(
                    "node `{name}` not found in recording. Recorded nodes: {}",
                    recorded_nodes
                        .keys()
                        .cloned()
                        .collect::<Vec<_>>()
                        .join(", ")
                );
            }
        }
        requested
    };

    // Find replay node binary
    let replay_node_bin = find_replay_node_binary()?;
    let recording_path =
        dunce::canonicalize(&args.file).wrap_err("failed to canonicalize recording path")?;

    // Modify the descriptor YAML
    let nodes = descriptor
        .get_mut("nodes")
        .and_then(|v| v.as_sequence_mut())
        .ok_or_else(|| eyre::eyre!("descriptor has no nodes array"))?;

    for node in nodes.iter_mut() {
        let node_id = node
            .get("id")
            .and_then(|v| v.as_str())
            .unwrap_or_default()
            .to_string();

        if !nodes_to_replace.contains(&node_id) {
            continue;
        }

        // Replace path with replay node binary
        node["path"] = serde_yaml::Value::String(replay_node_bin.to_string_lossy().to_string());

        // Remove build, git, operator, operators keys
        if let serde_yaml::Value::Mapping(map) = node {
            for key in [
                "build",
                "git",
                "branch",
                "tag",
                "rev",
                "operator",
                "operators",
                "custom",
                "args",
            ] {
                map.remove(serde_yaml::Value::String(key.to_string()));
            }
        }

        // Set env vars for replay node
        let env = node.get_mut("env").and_then(|v| v.as_mapping_mut());
        let env = if let Some(env) = env {
            env
        } else {
            node["env"] = serde_yaml::Value::Mapping(serde_yaml::Mapping::new());
            node.get_mut("env").unwrap().as_mapping_mut().unwrap()
        };

        env.insert(
            serde_yaml::Value::String("ADORA_REPLAY_FILE".to_string()),
            serde_yaml::Value::String(recording_path.to_string_lossy().to_string()),
        );
        env.insert(
            serde_yaml::Value::String("ADORA_REPLAY_NODE".to_string()),
            serde_yaml::Value::String(node_id),
        );
        env.insert(
            serde_yaml::Value::String("ADORA_REPLAY_SPEED".to_string()),
            serde_yaml::Value::String(args.speed.to_string()),
        );
        if args.r#loop {
            env.insert(
                serde_yaml::Value::String("ADORA_REPLAY_LOOP".to_string()),
                serde_yaml::Value::String("true".to_string()),
            );
        }

        // Clear inputs for replay nodes (they produce, not consume)
        if let serde_yaml::Value::Mapping(map) = node {
            map.remove(serde_yaml::Value::String("inputs".to_string()));
        }
    }

    let modified_yaml =
        serde_yaml::to_string(&descriptor).wrap_err("failed to serialize modified descriptor")?;

    // If --output-yaml, just write and exit
    if let Some(output_path) = args.output_yaml {
        std::fs::write(&output_path, &modified_yaml)
            .wrap_err_with(|| format!("failed to write {output_path}"))?;
        eprintln!("Modified descriptor written to {output_path}");
        return Ok(());
    }

    // Write to temp file and run
    let mut tmp =
        tempfile::NamedTempFile::with_suffix(".yml").wrap_err("failed to create temp file")?;
    tmp.write_all(modified_yaml.as_bytes())?;
    tmp.flush()?;
    let tmp_path = tmp.into_temp_path();

    eprintln!(
        "Replaying {} nodes from {}",
        nodes_to_replace.len(),
        args.file
    );
    eprintln!(
        "Replaced: {}",
        nodes_to_replace
            .iter()
            .cloned()
            .collect::<Vec<_>>()
            .join(", ")
    );
    eprintln!("Speed: {}x\n", args.speed);

    let run = Run::new(tmp_path.to_string_lossy().to_string());
    run.execute()
}

fn find_replay_node_binary() -> eyre::Result<PathBuf> {
    // Check next to current executable first
    if let Ok(exe) = std::env::current_exe() {
        let dir = exe.parent().unwrap_or(std::path::Path::new("."));
        let candidate = dir.join("adora-replay-node");
        if candidate.exists() {
            return Ok(candidate);
        }
        // Also check with .exe on Windows
        #[cfg(target_os = "windows")]
        {
            let candidate = dir.join("adora-replay-node.exe");
            if candidate.exists() {
                return Ok(candidate);
            }
        }
    }

    // Check PATH
    if let Ok(path) = which::which("adora-replay-node") {
        return Ok(path);
    }

    // Check cargo target directory (development)
    let cargo_target = std::env::var("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("target"));
    for profile in ["debug", "release"] {
        let candidate = cargo_target.join(profile).join("adora-replay-node");
        if candidate.exists() {
            return Ok(dunce::canonicalize(candidate)?);
        }
    }

    bail!(
        "could not find `adora-replay-node` binary.\n\
         Build it with: cargo build -p adora-replay-node"
    )
}
