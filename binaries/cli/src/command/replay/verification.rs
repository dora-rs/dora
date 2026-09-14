use std::collections::{BTreeMap, BTreeSet};

use dora_recording::replay_receipt::{ReplayReceiptConfig, read_receipt, verify_recording_counts};
use eyre::{Context, ContextCompat, ensure};
use serde_yaml::Value;
use tempfile::TempDir;
use uuid::Uuid;

const DEFAULT_REPLAY_QUEUE_SIZE: u64 = 64;
const MAX_REPLAY_QUEUE_SIZE: u64 = 1024;

struct Edge {
    producer: String,
    output: String,
    receiver: String,
    input: String,
}

pub(super) struct Verification {
    _directory: TempDir,
    producers: BTreeMap<String, ReplayReceiptConfig>,
    receivers: BTreeMap<String, ReplayReceiptConfig>,
    edges: Vec<Edge>,
}

impl Verification {
    pub(super) fn prepare(
        nodes: &mut [Value],
        recorded_counts: &BTreeMap<String, BTreeMap<String, u64>>,
    ) -> eyre::Result<Self> {
        let directory = tempfile::tempdir().context("failed to create replay receipt directory")?;
        let session_id = Uuid::new_v4();
        let mut producers = BTreeMap::new();
        let mut receivers = BTreeMap::new();
        let mut edges = Vec::new();
        for (index, node) in nodes.iter_mut().enumerate() {
            let node_id = node["id"]
                .as_str()
                .context("replay node has no ID")?
                .to_owned();
            for unsupported in [
                "deploy",
                "operator",
                "operators",
                "custom",
                "module",
                "restart_policy",
            ] {
                ensure!(
                    node.get(unsupported).is_none(),
                    "verified full-speed replay does not support `{unsupported}` on node `{node_id}`"
                );
            }
            ensure!(
                node["path"].as_str().is_some_and(|path| path != "dynamic"),
                "verified full-speed replay requires a static executable for node `{node_id}`"
            );
            if let Some(outputs) = recorded_counts.get(&node_id) {
                let config = ReplayReceiptConfig {
                    session_id,
                    node_id: node_id.clone(),
                    stream_ids: outputs.keys().cloned().collect(),
                    receipt_path: directory.path().join(format!("producer-{index}.json")),
                };
                set_config(node, "DORA_REPLAY_OUTPUT_RECEIPT", &config)?;
                producers.insert(node_id, config);
                continue;
            }
            let mut stream_ids = BTreeSet::new();
            if let Some(inputs) = node.get_mut("inputs").and_then(Value::as_mapping_mut) {
                for (input_id, input) in inputs {
                    let input_id = input_id.as_str().context("replay input has no ID")?;
                    let source = input
                        .as_str()
                        .or_else(|| input["source"].as_str())
                        .context("replay input has no source")?
                        .to_owned();
                    let Some((producer, output)) = source.split_once('/') else {
                        eyre::bail!("invalid replay input source `{source}`");
                    };
                    ensure!(
                        recorded_counts
                            .get(producer)
                            .is_some_and(|outputs| outputs.contains_key(output)),
                        "verified full-speed replay requires a recorded source for `{node_id}/{input_id}`"
                    );
                    ensure!(
                        input
                            .get("queue_policy")
                            .and_then(Value::as_str)
                            .is_none_or(|policy| policy == "backpressure"),
                        "verified full-speed replay does not support a lossy queue on `{node_id}/{input_id}`"
                    );
                    stream_ids.insert(input_id.to_owned());
                    edges.push(Edge {
                        producer: producer.to_owned(),
                        output: output.to_owned(),
                        receiver: node_id.clone(),
                        input: input_id.to_owned(),
                    });
                    let queue_size = input
                        .get("queue_size")
                        .map(|value| {
                            value
                                .as_u64()
                                .context("replay queue_size must be an integer")
                        })
                        .transpose()?
                        .unwrap_or(DEFAULT_REPLAY_QUEUE_SIZE);
                    ensure!(
                        (1..=MAX_REPLAY_QUEUE_SIZE).contains(&queue_size),
                        "verified replay queue_size for `{node_id}/{input_id}` must be between 1 and {MAX_REPLAY_QUEUE_SIZE}"
                    );
                    if input.as_str().is_some() {
                        *input = Value::Mapping(Default::default());
                        input["source"] = Value::String(source);
                    }
                    input["queue_size"] = Value::Number(queue_size.into());
                    input["queue_policy"] = Value::String("backpressure".into());
                }
            }
            ensure!(
                !stream_ids.is_empty(),
                "verified full-speed replay requires recorded inputs for receiver `{node_id}`"
            );
            let config = ReplayReceiptConfig {
                session_id,
                node_id: node_id.clone(),
                stream_ids,
                receipt_path: directory.path().join(format!("receiver-{index}.json")),
            };
            set_config(node, "DORA_REPLAY_INPUT_RECEIPT", &config)?;
            receivers.insert(node_id, config);
        }
        ensure!(
            producers.len() == recorded_counts.len(),
            "recording contains a source absent from the descriptor"
        );
        ensure!(
            !edges.is_empty(),
            "verified full-speed replay has no receiving edges"
        );
        Ok(Self {
            _directory: directory,
            producers,
            receivers,
            edges,
        })
    }

    pub(super) fn verify(
        &self,
        recorded_counts: &BTreeMap<String, BTreeMap<String, u64>>,
    ) -> eyre::Result<()> {
        let producers = self
            .producers
            .iter()
            .map(|(id, config)| {
                let receipt = read_receipt(config).wrap_err_with(|| {
                    format!("replay source `{id}` did not provide a valid output receipt")
                })?;
                verify_recording_counts(&recorded_counts[id], &receipt)?;
                Ok((id.clone(), receipt))
            })
            .collect::<eyre::Result<BTreeMap<_, _>>>()?;
        let receivers = self
            .receivers
            .iter()
            .map(|(id, config)| {
                let receipt = read_receipt(config).wrap_err_with(|| {
                    format!("replay receiver `{id}` did not provide a valid input receipt; receivers must use a node API with replay receipt support")
                })?;
                Ok((id.clone(), receipt))
            })
            .collect::<eyre::Result<BTreeMap<_, _>>>()?;
        for edge in &self.edges {
            let sent = &producers[&edge.producer].streams[&edge.output];
            let received = &receivers[&edge.receiver].streams[&edge.input];
            ensure!(
                sent == received,
                "replay delivery incomplete on `{}/{}` -> `{}/{}`: sent {}, received {}; ordered identity digests must match",
                edge.producer,
                edge.output,
                edge.receiver,
                edge.input,
                sent.count,
                received.count
            );
        }
        eprintln!(
            "Verified replay delivery on {} receiving edges",
            self.edges.len()
        );
        Ok(())
    }
}

fn set_config(node: &mut Value, key: &str, config: &ReplayReceiptConfig) -> eyre::Result<()> {
    if node.get("env").is_none() {
        node["env"] = Value::Mapping(Default::default());
    }
    let env = node["env"]
        .as_mapping_mut()
        .context("node env must be a mapping")?;
    env.insert(
        Value::String(key.to_owned()),
        Value::String(serde_json::to_string(config)?),
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use dora_recording::replay_receipt::ReplayReceiptRecorder;

    fn counts() -> BTreeMap<String, BTreeMap<String, u64>> {
        BTreeMap::from([("source".into(), BTreeMap::from([("output".into(), 2)]))])
    }

    fn nodes() -> Vec<Value> {
        serde_yaml::from_str(
            "- id: source\n  path: replay\n  outputs: [output]\n\
             - id: first\n  path: sink\n  inputs:\n    samples: source/output\n\
             - id: second\n  path: sink\n  inputs:\n    readings: source/output\n",
        )
        .expect("valid test descriptor")
    }

    fn publish(config: &ReplayReceiptConfig, identities: &[&[u8]]) {
        let mut recorder = ReplayReceiptRecorder::new(config.clone()).expect("valid config");
        for stream_id in &config.stream_ids {
            for identity in identities {
                recorder
                    .record(stream_id, identity)
                    .expect("record identity");
            }
        }
        recorder.finish().expect("publish receipt");
    }

    #[test]
    fn verifies_each_fan_out_edge_with_distinct_input_names() {
        let verification = Verification::prepare(&mut nodes(), &counts()).expect("prepare replay");
        for config in verification
            .producers
            .values()
            .chain(verification.receivers.values())
        {
            publish(config, &[b"first", b"last"]);
        }
        verification.verify(&counts()).expect("all edges delivered");
    }

    #[test]
    fn complete_receiver_does_not_hide_another_receivers_missing_suffix() {
        let verification = Verification::prepare(&mut nodes(), &counts()).expect("prepare replay");
        publish(&verification.producers["source"], &[b"first", b"last"]);
        publish(&verification.receivers["first"], &[b"first", b"last"]);
        publish(&verification.receivers["second"], &[b"first"]);
        let error = verification
            .verify(&counts())
            .expect_err("missing suffix must fail");
        assert!(error.to_string().contains("second/readings"));
    }

    #[test]
    fn missing_receiver_receipt_is_a_failure() {
        let verification = Verification::prepare(&mut nodes(), &counts()).expect("prepare replay");
        publish(&verification.producers["source"], &[b"first", b"last"]);
        assert!(verification.verify(&counts()).is_err());
    }

    #[test]
    fn rejects_explicit_lossy_inputs() {
        let mut nodes = nodes();
        nodes[1]["inputs"]["samples"] =
            serde_yaml::from_str("source: source/output\nqueue_policy: drop_oldest\n")
                .expect("valid input");
        assert!(Verification::prepare(&mut nodes, &counts()).is_err());
    }

    #[test]
    fn rejects_live_inputs_in_verified_graph() {
        let mut nodes = nodes();
        nodes[1]["inputs"]["timer"] = Value::String("dora/timer/millis/10".into());
        assert!(Verification::prepare(&mut nodes, &counts()).is_err());
    }

    #[test]
    fn equal_counts_do_not_hide_duplicates_or_reordering() {
        for received in [
            [b"first".as_slice(), b"first"],
            [b"last".as_slice(), b"first"],
        ] {
            let verification =
                Verification::prepare(&mut nodes(), &counts()).expect("prepare replay");
            publish(&verification.producers["source"], &[b"first", b"last"]);
            publish(&verification.receivers["first"], &received);
            publish(&verification.receivers["second"], &[b"first", b"last"]);
            let error = verification
                .verify(&counts())
                .expect_err("different identities must fail");
            assert!(error.to_string().contains("first/samples"));
        }
    }

    #[test]
    fn queue_sizes_do_not_grow_with_the_recording() {
        let mut nodes = nodes();
        nodes[2]["inputs"]["readings"] = serde_yaml::from_str(
            "source: source/output\nqueue_size: 1\nqueue_policy: backpressure\n",
        )
        .expect("valid input");
        let large_counts = BTreeMap::from([(
            "source".into(),
            BTreeMap::from([("output".into(), 1_000_000)]),
        )]);
        let _verification =
            Verification::prepare(&mut nodes, &large_counts).expect("prepare replay");
        assert_eq!(
            nodes[1]["inputs"]["samples"]["queue_size"].as_u64(),
            Some(64)
        );
        assert_eq!(
            nodes[2]["inputs"]["readings"]["queue_size"].as_u64(),
            Some(1)
        );
    }

    #[test]
    fn rejects_unsupported_receiver_forms() {
        for field in [
            "deploy",
            "operator",
            "operators",
            "custom",
            "module",
            "restart_policy",
        ] {
            let mut nodes = nodes();
            nodes[1][field] = Value::Mapping(Default::default());
            let error = Verification::prepare(&mut nodes, &counts())
                .err()
                .expect("unsupported receiver");
            assert!(error.to_string().contains(&format!("`{field}`")));
        }
        let mut nodes = nodes();
        nodes[1]["path"] = Value::String("dynamic".into());
        let error = Verification::prepare(&mut nodes, &counts())
            .err()
            .expect("dynamic receiver");
        assert!(error.to_string().contains("static executable"));
    }

    #[test]
    fn rejects_queue_sizes_outside_the_admission_contract() {
        for size in ["0", "1025", "-1", "1.5", "unbounded"] {
            let mut nodes = nodes();
            nodes[1]["inputs"]["samples"] =
                serde_yaml::from_str(&format!("source: source/output\nqueue_size: {size}\n"))
                    .expect("valid YAML");
            let error = Verification::prepare(&mut nodes, &counts())
                .err()
                .expect("invalid queue size");
            assert!(error.to_string().contains("queue_size"));
        }
    }
}
