use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::Write,
    path::{Path, PathBuf},
    time::Duration,
};

mod verification;

use clap::Args;
use dora_recording::RecordingReader;
use eyre::{Context, bail};

use crate::command::{Executable, Run};

/// Replay a recorded dataflow from a `.drec` file.
///
/// Reads a recording, identifies which nodes produced the recorded data,
/// replaces them with replay nodes, and runs the modified dataflow.
/// Downstream nodes receive recorded payloads through the node input API.
/// Executed replay at speed 0 requires matching sender and receiver input-API receipts.
/// This verifies ordered delivery per direct recorded edge, not application processing.
/// Receivers must use a node API with replay receipt support.
/// Full-speed verification supports finite local graphs with static executable receivers.
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

    /// Maximum run duration for verified full-speed replay
    #[clap(long, default_value = "30s", value_parser = crate::common::parse_duration)]
    delivery_timeout: Duration,
}

impl Executable for Replay {
    fn execute(self) -> eyre::Result<()> {
        // Run::execute() sets up its own tracing subscriber.
        run_replay(self)
    }
}

fn run_replay(args: Replay) -> eyre::Result<()> {
    if !args.speed.is_finite() || args.speed < 0.0 {
        bail!("replay speed must be a finite nonnegative number");
    }
    let verify_delivery = args.speed == 0.0 && args.output_yaml.is_none();
    if verify_delivery && crate::common::write_events_to().is_some() {
        bail!(
            "verified full-speed replay does not support DORA_WRITE_EVENTS_TO because event tracing retains an unbounded history"
        );
    }
    if verify_delivery && args.r#loop {
        bail!("verified full-speed replay requires a finite recording; --loop is not supported");
    }
    if verify_delivery && args.delivery_timeout.is_zero() {
        bail!("replay delivery timeout must be greater than zero");
    }
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

    if verify_delivery
        && descriptor
            .get("deploy")
            .is_some_and(|value| !value.is_null())
    {
        bail!("verified full-speed replay does not support dataflow deployment settings");
    }

    // Discover which nodes produced recorded data and how many messages each
    // output carries (used to size receiver queues below).
    let mut recorded_counts: BTreeMap<String, BTreeMap<String, u64>> = BTreeMap::new();
    // Only the ids are needed here; `next_entry_header` skips copying each
    // entry's event payload out of the record buffer.
    while let Some(entry) = reader.next_entry_header()? {
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

    if verify_delivery && nodes_to_replace.len() < recorded_counts.len() {
        bail!(
            "verified full-speed replay requires all recorded nodes; partial --replace is not supported"
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

    replace_recorded_nodes_with_replay(
        nodes,
        &nodes_to_replace,
        &replay_node_bin,
        &recording_path,
        args.speed,
        args.r#loop,
    );
    if !verify_delivery {
        raise_replayed_input_queue_sizes(nodes, &nodes_to_replace, &recorded_counts);
    }

    let verification = if verify_delivery {
        Some(verification::Verification::prepare(
            nodes,
            &recorded_counts,
        )?)
    } else {
        None
    };

    let modified_yaml =
        serde_yaml::to_string(&descriptor).wrap_err("failed to serialize modified descriptor")?;

    // If --output-yaml, just write and exit
    if let Some(output_path) = args.output_yaml {
        std::fs::write(&output_path, &modified_yaml)
            .wrap_err_with(|| format!("failed to write {output_path}"))?;
        eprintln!("Modified descriptor written to {output_path}");
        eprintln!("Descriptor generation does not verify replay delivery");
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
    let mut run = Run::new(tmp_path.to_string_lossy().to_string()).with_working_dir(recording_dir);
    if verification.is_some() {
        run.stop_after = Some(args.delivery_timeout);
        run = run.with_fail_on_stop();
    }
    run.execute()?;
    if let Some(verification) = verification {
        verification.verify(&recorded_counts)?;
    }
    Ok(())
}

/// Replace recorded producers with the replay executable and recording settings.
fn replace_recorded_nodes_with_replay(
    nodes: &mut serde_yaml::Sequence,
    nodes_to_replace: &BTreeSet<String>,
    replay_node_bin: &Path,
    recording_path: &Path,
    speed: f64,
    r#loop: bool,
) {
    for node in nodes.iter_mut() {
        let node_id = node
            .get("id")
            .and_then(|v| v.as_str())
            .unwrap_or_default()
            .to_string();

        if !nodes_to_replace.contains(&node_id) {
            continue;
        }

        let outputs = replay_node_outputs(node);
        node["path"] = serde_yaml::Value::String(replay_node_bin.to_string_lossy().to_string());

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
                "inputs",
            ] {
                map.remove(serde_yaml::Value::String(key.to_string()));
            }
            map.insert(serde_yaml::Value::String("outputs".to_string()), outputs);
        }

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
            serde_yaml::Value::String(speed.to_string()),
        );
        if r#loop {
            env.insert(
                serde_yaml::Value::String("DORA_REPLAY_LOOP".to_string()),
                serde_yaml::Value::String("true".to_string()),
            );
        }
    }
}

fn replay_node_outputs(node: &serde_yaml::Value) -> serde_yaml::Value {
    let mut outputs = serde_yaml::Sequence::new();
    append_outputs(&mut outputs, node.get("outputs"));
    append_outputs(
        &mut outputs,
        node.get("operator").and_then(|v| v.get("outputs")),
    );
    if let Some(operators) = node.get("operators").and_then(|v| v.as_sequence()) {
        for operator in operators {
            let Some(operator_id) = operator.get("id").and_then(|v| v.as_str()) else {
                continue;
            };
            append_prefixed_outputs(&mut outputs, operator_id, operator.get("outputs"));
        }
    }
    serde_yaml::Value::Sequence(outputs)
}

fn append_outputs(outputs: &mut serde_yaml::Sequence, value: Option<&serde_yaml::Value>) {
    if let Some(node_outputs) = value.and_then(|v| v.as_sequence()) {
        outputs.extend(node_outputs.iter().cloned());
    }
}

fn append_prefixed_outputs(
    outputs: &mut serde_yaml::Sequence,
    prefix: &str,
    value: Option<&serde_yaml::Value>,
) {
    if let Some(node_outputs) = value.and_then(|v| v.as_sequence()) {
        outputs.extend(node_outputs.iter().filter_map(|output| {
            output
                .as_str()
                .map(|output_id| serde_yaml::Value::String(format!("{prefix}/{output_id}")))
        }));
    }
}

/// Size unspecified queues for one recording pass in paced replay or generated YAML.
/// Queue sizing does not verify delivery. Explicit queue sizes remain unchanged.
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

        // Inputs can live at the node level, under a single `operator:`, or
        // per-entry in an `operators:` list.
        for holder_key in ["inputs", "operator"] {
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

    #[test]
    fn rejects_speeds_that_bypass_full_speed_verification() {
        for speed in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            let error = run_replay(Replay {
                file: "unused.drec".into(),
                replace: Vec::new(),
                speed,
                r#loop: false,
                output_yaml: None,
                delivery_timeout: Duration::from_secs(30),
            })
            .expect_err("invalid speed must fail before opening the recording");
            assert!(error.to_string().contains("replay speed"));
        }
    }

    #[test]
    fn rejects_dataflow_deployment_before_starting_replay() {
        let file = tempfile::NamedTempFile::new().expect("recording file");
        let header = dora_recording::RecordingHeader {
            version: dora_recording::FORMAT_VERSION,
            start_nanos: 0,
            dataflow_id: uuid::Uuid::nil(),
            descriptor_yaml: b"deploy: {machine: remote}\nnodes: []\n".to_vec(),
        };
        dora_recording::RecordingWriter::new(file.reopen().expect("open recording"), &header)
            .expect("write header")
            .finish()
            .expect("finish recording");
        let error = run_replay(Replay {
            file: file.path().to_string_lossy().into_owned(),
            replace: Vec::new(),
            speed: 0.0,
            r#loop: false,
            output_yaml: None,
            delivery_timeout: Duration::from_secs(30),
        })
        .expect_err("deployment must be rejected before running");
        assert!(error.to_string().contains("dataflow deployment settings"));
    }

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

    fn replaced_with_replay(yaml: &str, replaced: &[&str]) -> String {
        let mut descriptor: serde_yaml::Value = serde_yaml::from_str(yaml).unwrap();
        let nodes = descriptor
            .get_mut("nodes")
            .and_then(|v| v.as_sequence_mut())
            .unwrap();
        let nodes_to_replace: BTreeSet<String> = replaced.iter().map(|s| s.to_string()).collect();
        replace_recorded_nodes_with_replay(
            nodes,
            &nodes_to_replace,
            &PathBuf::from("/tmp/dora-replay-node"),
            &PathBuf::from("/tmp/recording.drec"),
            1.0,
            false,
        );
        serde_yaml::to_string(&descriptor).unwrap()
    }

    #[test]
    fn replay_replacement_preserves_outputs_from_all_descriptor_node_kinds() {
        // `custom:` is deliberately absent: `Node` is `deny_unknown_fields` and
        // has no `custom` field, so such a descriptor cannot deserialize.
        let out = replaced_with_replay(
            concat!(
                "nodes:\n",
                "- id: single\n",
                "  operator:\n",
                "    python: single.py\n",
                "    outputs:\n",
                "      - image\n",
                "- id: runtime\n",
                "  operators:\n",
                "  - id: op\n",
                "    python: runtime.py\n",
                "    outputs:\n",
                "      - status\n",
                "- id: sink\n",
                "  inputs:\n",
                "    image: single/image\n",
                "    status: runtime/op/status\n",
            ),
            &["single", "runtime"],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();

        assert_eq!(parsed["nodes"][0]["outputs"][0].as_str(), Some("image"));
        assert!(parsed["nodes"][0].get("operator").is_none());
        assert_eq!(parsed["nodes"][1]["outputs"][0].as_str(), Some("op/status"));
        assert!(parsed["nodes"][1].get("operators").is_none());
        assert_eq!(
            parsed["nodes"][2]["inputs"]["image"].as_str(),
            Some("single/image")
        );
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
    fn operator_inputs_are_rewritten() {
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
            ),
            &["source"],
            &[("source", "status", 100)],
        );
        let parsed: serde_yaml::Value = serde_yaml::from_str(&out).unwrap();
        for input in [
            &parsed["nodes"][0]["operators"][0]["inputs"]["image"],
            &parsed["nodes"][1]["operator"]["inputs"]["image"],
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

    /// `record --proxy` and `replay` must agree on the output id.
    ///
    /// The daemon reports a single-`operator:` node's `image` as `op/image`.
    /// If the recording stores that wire id verbatim, replay declares
    /// `outputs: [image]` and then calls `send_output("op/image", ..)`;
    /// `validate_output` rejects it, `send_output` still returns `Ok(())`, and
    /// every message is silently dropped. Pin that the id the proxy path stores
    /// is one the replacement node actually declares (dora-rs/dora#2893).
    #[test]
    fn proxy_recorded_output_id_is_declared_by_the_replay_node() {
        const YAML: &str = "\
nodes:
  - id: single
    operator:
      python: single.py
      outputs:
        - image
  - id: runtime
    operators:
      - id: op
        python: runtime.py
        outputs:
          - status
";
        let typed: dora_core::descriptor::Descriptor =
            serde_yaml::from_str(YAML).expect("parse typed descriptor");
        let untyped: serde_yaml::Value = serde_yaml::from_str(YAML).expect("parse untyped");
        let untyped_nodes = untyped["nodes"].as_sequence().expect("nodes array");

        // (node id, id the daemon reports on the wire)
        for (node_id, wire) in [("single", "op/image"), ("runtime", "op/status")] {
            let typed_node = typed
                .nodes
                .iter()
                .find(|n| n.id.to_string() == node_id)
                .expect("node in fixture");
            let untyped_node = untyped_nodes
                .iter()
                .find(|n| n["id"].as_str() == Some(node_id))
                .expect("node in fixture");

            let stored = crate::command::topic::selector::public_topic_output_id(
                typed_node,
                &wire.to_string().into(),
            );
            let declared: Vec<String> = replay_node_outputs(untyped_node)
                .as_sequence()
                .expect("outputs sequence")
                .iter()
                .map(|v| v.as_str().expect("output id").to_string())
                .collect();

            assert!(
                declared
                    .iter()
                    .any(|d| d.as_str() == AsRef::<str>::as_ref(&stored)),
                "`{node_id}`: recorded `{stored}` (from wire `{wire}`) is not in the \
                 replay node's declared outputs {declared:?}",
            );
        }
    }
}
