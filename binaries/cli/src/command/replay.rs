use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::Write,
    path::PathBuf,
};

use clap::Args;
use dora_recording::RecordingReader;
use eyre::{Context, bail};

use crate::command::{Executable, Run};

/// Replay a recorded dataflow from a `.drec` file.
///
/// Reads a recording, identifies which nodes produced the recorded data,
/// replaces them with replay nodes, and runs the modified dataflow.
/// Downstream nodes receive replayed data identically to live data.
///
/// Examples:
///
///   Replay at original speed:
///     dora replay recording.drec
///
///   Replay at 2x speed:
///     dora replay recording.drec --speed 2.0
///
///   Replay as fast as possible:
///     dora replay recording.drec --speed 0
///
///   Only replace specific nodes:
///     dora replay recording.drec --replace sensor,camera
///
///   Just generate the modified YAML:
///     dora replay recording.drec --output-yaml modified.yml
#[derive(Debug, Args)]
#[clap(verbatim_doc_comment)]
pub struct Replay {
    /// Path to the `.drec` recording file
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
        // Run::execute() sets up its own tracing subscriber.
        run_replay(self)
    }
}

fn run_replay(args: Replay) -> eyre::Result<()> {
    let file = File::open(&args.file).wrap_err_with(|| {
        format!(
            "failed to open recording file `{}`\n\n  \
             hint: check the path is correct. Recording files have the `.drec` extension",
            args.file
        )
    })?;
    let mut reader = RecordingReader::open(file).wrap_err("failed to read recording header")?;

    let header = reader.header().clone();
    let descriptor_yaml = std::str::from_utf8(&header.descriptor_yaml)
        .wrap_err("invalid descriptor YAML in recording")?;

    let mut descriptor: serde_yaml::Value =
        serde_yaml::from_str(descriptor_yaml).wrap_err("failed to parse descriptor YAML")?;

    // Discover which nodes produced recorded data and how many messages each
    // output carries (used to size receiver queues below).
    let mut recorded_counts: BTreeMap<String, BTreeMap<String, u64>> = BTreeMap::new();
    while let Some(entry) = reader.next_entry()? {
        *recorded_counts
            .entry(entry.node_id)
            .or_default()
            .entry(entry.output_id)
            .or_default() += 1;
    }

    if recorded_counts.is_empty() {
        bail!(
            "recording `{}` contains no messages\n\n  \
             hint: the recording may be empty or corrupted. \
             Try re-recording with `dora record`",
            args.file
        );
    }

    // Determine which nodes to replace
    let nodes_to_replace: BTreeSet<String> = if args.replace.is_empty() {
        recorded_counts.keys().cloned().collect()
    } else {
        let requested: BTreeSet<String> = args.replace.into_iter().collect();
        for name in &requested {
            if !recorded_counts.contains_key(name) {
                bail!(
                    "node `{name}` not found in recording. Recorded nodes: {}",
                    recorded_counts
                        .keys()
                        .cloned()
                        .collect::<Vec<_>>()
                        .join(", ")
                );
            }
        }
        requested
    };

    if args.speed == 0.0 && nodes_to_replace.len() < recorded_counts.len() {
        eprintln!(
            "warning: partial --replace with --speed 0: live intermediate nodes may re-emit \
             faster than their downstream consumers' default input queues can absorb"
        );
    }

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
            serde_yaml::Value::String("DORA_REPLAY_FILE".to_string()),
            serde_yaml::Value::String(recording_path.to_string_lossy().to_string()),
        );
        env.insert(
            serde_yaml::Value::String("DORA_REPLAY_NODE".to_string()),
            serde_yaml::Value::String(node_id),
        );
        env.insert(
            serde_yaml::Value::String("DORA_REPLAY_SPEED".to_string()),
            serde_yaml::Value::String(args.speed.to_string()),
        );
        if args.r#loop {
            env.insert(
                serde_yaml::Value::String("DORA_REPLAY_LOOP".to_string()),
                serde_yaml::Value::String("true".to_string()),
            );
        }

        // Clear inputs for replay nodes (they produce, not consume)
        if let serde_yaml::Value::Mapping(map) = node {
            map.remove(serde_yaml::Value::String("inputs".to_string()));
        }
    }

    raise_replayed_input_queue_sizes(nodes, &nodes_to_replace, &recorded_counts);

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

    // The modified YAML lives in /tmp, but the original descriptor's
    // `build: cargo build -p <node>` directives need to run in a dir where
    // Cargo.toml is reachable and the descriptor's relative `path:` entries
    // resolve. Mirror `dora record`'s fix (#1674) with the .drec file's
    // parent as the working_dir.
    //
    // Convention: the .drec is expected to live next to (or under) the
    // original dataflow directory -- i.e. `dora record foo.yml -o foo.drec`
    // writes foo.drec into foo.yml's parent if invoked from that dir. If a
    // user moves the .drec to a dir without the workspace visible (e.g.
    // `/tmp`), they'll get the same confusing error they'd get from
    // `cargo build` in that dir, which is expected -- build commands need
    // to resolve against a workspace.
    let recording_dir = PathBuf::from(&args.file)
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("."));
    let run = Run::new(tmp_path.to_string_lossy().to_string()).with_working_dir(recording_dir);
    run.execute()
}

/// Sizes receiver queues so a full-speed replay cannot silently drop messages.
///
/// Replay can outpace receivers — especially with `--speed 0` — and dora's
/// real-time defaults drop under pressure: each input queue holds
/// `DEFAULT_QUEUE_SIZE` messages (drop-oldest), and the node event channel is
/// sized from the queue sizes. For every input fed by a replayed node, this
/// sets `queue_size` to the recorded message count for that output and
/// `queue_policy: backpressure`, so one pass of the recording fits entirely
/// and any residual overflow is logged loudly instead of dropped silently
/// (#2144).
///
/// Scope and limits:
/// - Inputs with an explicit `queue_size` are left untouched: an explicit
///   size encodes deliberate freshness semantics (e.g. `queue_size: 1`),
///   which replay should reproduce, not override.
/// - Only inputs *directly* sourced from replayed nodes are adjusted. With a
///   partial `--replace`, live intermediate nodes can re-emit at full speed
///   into their own downstream consumers' default queues (warned at the call
///   site).
/// - The sizing bounds one pass of the recording; `--loop` can still
///   overflow, at which point the backpressure policy logs errors at its
///   hard cap instead of dropping silently.
/// - Drops below this layer (zenoh congestion on multi-daemon replay) are
///   not addressed here.
fn raise_replayed_input_queue_sizes(
    nodes: &mut serde_yaml::Sequence,
    nodes_to_replace: &BTreeSet<String>,
    recorded_counts: &BTreeMap<String, BTreeMap<String, u64>>,
) {
    for node in nodes.iter_mut() {
        let node_id = node.get("id").and_then(|v| v.as_str()).unwrap_or_default();
        if nodes_to_replace.contains(node_id) {
            continue;
        }

        // Inputs can live at the node level, under the legacy `custom:` key,
        // under a single `operator:`, or per-entry in an `operators:` list.
        for holder_key in ["inputs", "custom", "operator"] {
            let Some(holder) = node.get_mut(holder_key) else {
                continue;
            };
            let inputs = match holder_key {
                "inputs" => holder.as_mapping_mut(),
                _ => holder.get_mut("inputs").and_then(|v| v.as_mapping_mut()),
            };
            if let Some(inputs) = inputs {
                raise_input_queue_sizes(inputs, nodes_to_replace, recorded_counts);
            }
        }
        if let Some(operators) = node.get_mut("operators").and_then(|v| v.as_sequence_mut()) {
            for operator in operators.iter_mut() {
                if let Some(inputs) = operator.get_mut("inputs").and_then(|v| v.as_mapping_mut()) {
                    raise_input_queue_sizes(inputs, nodes_to_replace, recorded_counts);
                }
            }
        }
    }
}

fn raise_input_queue_sizes(
    inputs: &mut serde_yaml::Mapping,
    nodes_to_replace: &BTreeSet<String>,
    recorded_counts: &BTreeMap<String, BTreeMap<String, u64>>,
) {
    let source_key = serde_yaml::Value::String("source".to_string());
    let queue_size_key = serde_yaml::Value::String("queue_size".to_string());
    let queue_policy_key = serde_yaml::Value::String("queue_policy".to_string());

    for (_input_id, value) in inputs.iter_mut() {
        // Inputs are either a plain `node/output` string or a mapping with
        // a `source` key (plus optional queue_size/queue_policy/timeout).
        let source = match &*value {
            serde_yaml::Value::String(s) => s.clone(),
            serde_yaml::Value::Mapping(m) => {
                if m.contains_key(&queue_size_key) {
                    // Explicit user sizing wins (see fn docs).
                    continue;
                }
                match m.get(&source_key).and_then(|v| v.as_str()) {
                    Some(s) => s.to_string(),
                    None => continue,
                }
            }
            _ => continue,
        };
        let Some((source_node, source_output)) = source.split_once('/') else {
            continue;
        };
        if !nodes_to_replace.contains(source_node) {
            continue;
        }
        let Some(&count) = recorded_counts
            .get(source_node)
            .and_then(|outputs| outputs.get(source_output))
        else {
            continue;
        };
        let new_size = count.max(dora_message::config::DEFAULT_QUEUE_SIZE as u64);

        let mapping = match value {
            serde_yaml::Value::Mapping(m) => m,
            _ => {
                let mut m = serde_yaml::Mapping::new();
                m.insert(source_key.clone(), serde_yaml::Value::String(source));
                *value = serde_yaml::Value::Mapping(m);
                match value.as_mapping_mut() {
                    Some(m) => m,
                    None => continue,
                }
            }
        };
        mapping.insert(
            queue_size_key.clone(),
            serde_yaml::Value::Number(new_size.into()),
        );
        if !mapping.contains_key(&queue_policy_key) {
            mapping.insert(
                queue_policy_key.clone(),
                serde_yaml::Value::String("backpressure".to_string()),
            );
        }
    }
}

fn find_replay_node_binary() -> eyre::Result<PathBuf> {
    super::node_binary::find("dora-replay-node", "dora-replay-node")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_rewrite(yaml: &str, replaced: &[&str], counts: &[(&str, &str, u64)]) -> String {
        let mut descriptor: serde_yaml::Value = serde_yaml::from_str(yaml).unwrap();
        let nodes = descriptor
            .get_mut("nodes")
            .and_then(|v| v.as_sequence_mut())
            .unwrap();
        let nodes_to_replace: BTreeSet<String> = replaced.iter().map(|s| s.to_string()).collect();
        let mut recorded_counts: BTreeMap<String, BTreeMap<String, u64>> = BTreeMap::new();
        for (n, o, c) in counts {
            recorded_counts
                .entry(n.to_string())
                .or_default()
                .insert(o.to_string(), *c);
        }
        raise_replayed_input_queue_sizes(nodes, &nodes_to_replace, &recorded_counts);
        serde_yaml::to_string(&descriptor).unwrap()
    }

    #[test]
    fn string_input_from_replayed_node_gets_count_size_and_backpressure() {
        let out = run_rewrite(
            "nodes:\n- id: sink\n  inputs:\n    message: source/status\n",
            &["source"],
            &[("source", "status", 100)],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();
        let input = &parsed["nodes"][0]["inputs"]["message"];
        assert_eq!(input["source"].as_str(), Some("source/status"));
        assert_eq!(input["queue_size"].as_u64(), Some(100));
        assert_eq!(input["queue_policy"].as_str(), Some("backpressure"));
    }

    #[test]
    fn explicit_queue_size_is_left_untouched() {
        let out = run_rewrite(
            "nodes:\n- id: sink\n  inputs:\n    message:\n      source: source/status\n      queue_size: 1\n",
            &["source"],
            &[("source", "status", 100)],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();
        let input = &parsed["nodes"][0]["inputs"]["message"];
        assert_eq!(input["queue_size"].as_u64(), Some(1));
        assert!(input.get("queue_policy").is_none());
    }

    #[test]
    fn explicit_queue_policy_is_preserved() {
        let out = run_rewrite(
            "nodes:\n- id: sink\n  inputs:\n    message:\n      source: source/status\n      queue_policy: drop_oldest\n",
            &["source"],
            &[("source", "status", 100)],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();
        let input = &parsed["nodes"][0]["inputs"]["message"];
        assert_eq!(input["queue_size"].as_u64(), Some(100));
        assert_eq!(input["queue_policy"].as_str(), Some("drop_oldest"));
    }

    #[test]
    fn operator_and_custom_inputs_are_rewritten() {
        let out = run_rewrite(
            concat!(
                "nodes:\n",
                "- id: runtime-node\n",
                "  operators:\n",
                "  - id: op\n",
                "    inputs:\n",
                "      image: source/status\n",
                "- id: single-op\n",
                "  operator:\n",
                "    inputs:\n",
                "      image: source/status\n",
                "- id: legacy\n",
                "  custom:\n",
                "    inputs:\n",
                "      image: source/status\n",
            ),
            &["source"],
            &[("source", "status", 100)],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();
        for input in [
            &parsed["nodes"][0]["operators"][0]["inputs"]["image"],
            &parsed["nodes"][1]["operator"]["inputs"]["image"],
            &parsed["nodes"][2]["custom"]["inputs"]["image"],
        ] {
            assert_eq!(input["queue_size"].as_u64(), Some(100));
            assert_eq!(input["queue_policy"].as_str(), Some("backpressure"));
        }
    }

    #[test]
    fn small_recordings_keep_at_least_the_default_queue_size() {
        let out = run_rewrite(
            "nodes:\n- id: sink\n  inputs:\n    message: source/status\n",
            &["source"],
            &[("source", "status", 3)],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();
        assert_eq!(
            parsed["nodes"][0]["inputs"]["message"]["queue_size"].as_u64(),
            Some(dora_message::config::DEFAULT_QUEUE_SIZE as u64)
        );
    }

    #[test]
    fn timer_and_live_inputs_are_untouched() {
        let out = run_rewrite(
            "nodes:\n- id: sink\n  inputs:\n    tick: dora/timer/millis/10\n    live: other/data\n",
            &["source"],
            &[("source", "status", 100)],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();
        assert_eq!(
            parsed["nodes"][0]["inputs"]["tick"].as_str(),
            Some("dora/timer/millis/10")
        );
        assert_eq!(
            parsed["nodes"][0]["inputs"]["live"].as_str(),
            Some("other/data")
        );
    }

    #[test]
    fn replayed_nodes_themselves_are_skipped() {
        let out = run_rewrite(
            "nodes:\n- id: source\n  inputs:\n    feedback: other/data\n",
            &["source", "other"],
            &[("other", "data", 100)],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();
        assert_eq!(
            parsed["nodes"][0]["inputs"]["feedback"].as_str(),
            Some("other/data")
        );
    }
}
