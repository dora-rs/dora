use std::{
    collections::{BTreeSet, HashMap},
    fmt,
};

use crate::common::resolve_dataflow_identifier_interactive;
use crate::ws_client::WsSession;
use dora_core::{
    config::{Input, InputMapping},
    descriptor::{Descriptor, Node, SINGLE_OPERATOR_DEFAULT_ID},
};
use dora_message::{
    DataflowId,
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::ControlRequestReply,
    id::{DataId, NodeId, OperatorId},
};
use eyre::{Context, ContextCompat, bail};
use uuid::Uuid;

#[derive(Debug, clap::Args)]
pub struct DataflowSelector {
    /// Identifier of the dataflow
    #[clap(long, short, value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
}

impl DataflowSelector {
    pub fn resolve(&self, session: &WsSession) -> eyre::Result<(Uuid, Descriptor)> {
        let dataflow_id =
            resolve_dataflow_identifier_interactive(session, self.dataflow.as_deref())?;
        let reply_raw = session
            .request(
                &serde_json::to_vec(&ControlRequest::Info {
                    dataflow_uuid: dataflow_id,
                })
                .unwrap(),
            )
            .wrap_err("failed to send message")?;
        let reply: ControlRequestReply =
            serde_json::from_slice(&reply_raw).wrap_err("failed to parse reply")?;
        match reply {
            ControlRequestReply::DataflowInfo { descriptor, .. } => Ok((dataflow_id, descriptor)),
            ControlRequestReply::Error(err) => bail!("{err}"),
            other => bail!("unexpected list dataflow reply: {other:?}"),
        }
    }
}

#[derive(Debug, clap::Args)]
pub struct TopicSelector {
    #[clap(flatten)]
    pub dataflow: DataflowSelector,
    /// Data to inspect, e.g. `node_id/output_id`
    #[clap(value_name = "DATA")]
    pub data: Vec<String>,
}

#[derive(Debug, Clone, PartialOrd, Ord, PartialEq, Eq)]
pub struct TopicIdentifier {
    pub node_id: NodeId,
    pub data_id: DataId,
}

impl fmt::Display for TopicIdentifier {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}", self.node_id, self.data_id)
    }
}

/// Map a debug frame's *wire* output id back to the public id that
/// [`node_topic_outputs`] hands out.
///
/// The daemon keys and reports outputs by their post-resolution ids: a single
/// `operator:` node's `image` is `op/image` on the wire, because
/// `resolve_aliases_and_set_defaults` injects the operator segment that
/// runtime nodes require. The coordinator translates a subscription *request*
/// into that form, so frames come back carrying the wire id while the caller
/// only ever knew the public one. Every consumer -- echo's label, hz's
/// per-topic index, `record --proxy`'s stored entry -- has to undo the
/// translation before matching, displaying, or persisting it, or it silently
/// works on a name that never matches (dora-rs/dora#2893).
///
/// Multi-operator `operators:` nodes are already qualified on both sides and
/// plain nodes are never qualified, so both pass through unchanged.
pub(crate) fn public_topic_output_id(node: &Node, wire_output: &DataId) -> DataId {
    let Some(operator) = &node.operator else {
        return wire_output.clone();
    };
    let default_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());
    let operator_id = operator.id.as_ref().unwrap_or(&default_id);
    match wire_output.strip_prefix(&format!("{operator_id}/")) {
        Some(bare) => DataId::from(bare.to_string()),
        None => wire_output.clone(),
    }
}

/// Every `(local_input_id, mapping)` a node consumes.
///
/// Mirror of [`node_topic_outputs`]: `node.inputs` is empty for `operator:` and
/// `operators:` nodes -- they wire their inputs through `operator.config.inputs`
/// / `operators[].config.inputs` -- so iterating `node.inputs` alone leaves
/// every operator-node consumer out of the subscriber listing
/// (dora-rs/dora#2893).
pub(crate) fn node_topic_inputs(node: &Node) -> Vec<(&DataId, &Input)> {
    let mut inputs: Vec<_> = node.inputs.iter().collect();

    if let Some(operator) = &node.operator {
        inputs.extend(operator.config.inputs.iter());
    }
    if let Some(runtime) = &node.operators {
        for operator in &runtime.operators {
            inputs.extend(operator.config.inputs.iter());
        }
    }

    inputs
}

/// Inverse of [`public_topic_output_id`]: the id the daemon reports a frame
/// under, given the public id [`node_topic_outputs`] hands out.
///
/// Use this to key a lookup table that is consulted per frame, so the
/// translation is paid once at setup instead of on every message.
pub(crate) fn wire_topic_output_id(node: &Node, public_output: &DataId) -> DataId {
    let Some(operator) = &node.operator else {
        return public_output.clone();
    };
    let default_id = OperatorId::from(SINGLE_OPERATOR_DEFAULT_ID.to_string());
    let operator_id = operator.id.as_ref().unwrap_or(&default_id);
    DataId::from(format!("{operator_id}/{public_output}"))
}

pub(crate) fn node_topic_outputs(node: &Node) -> BTreeSet<DataId> {
    let mut outputs = node.outputs.clone();

    if let Some(operator) = &node.operator {
        outputs.extend(operator.config.outputs.iter().cloned());
    }
    if let Some(runtime) = &node.operators {
        for operator in &runtime.operators {
            outputs.extend(
                operator
                    .config
                    .outputs
                    .iter()
                    .map(|output| DataId::from(format!("{}/{}", operator.id, output))),
            );
        }
    }

    outputs
}

impl TopicSelector {
    /// Resolve the selected topics, returning the dataflow descriptor that was
    /// fetched to do it.
    ///
    /// Every caller needs the descriptor: to enumerate subscribers, or to
    /// translate between the public output ids this returns and the wire ids
    /// the daemon reports on frames (see [`public_topic_output_id`]). Handing
    /// it back avoids a second `Info` round-trip — and, when the dataflow is
    /// ambiguous, a second interactive selection prompt.
    pub fn resolve_with_descriptor(
        &self,
        session: &WsSession,
    ) -> eyre::Result<(DataflowId, BTreeSet<TopicIdentifier>, Descriptor)> {
        let (dataflow_id, dataflow_descriptor) = self.dataflow.resolve(session)?;
        let data = resolve_topics(&self.data, &dataflow_descriptor)?;
        Ok((dataflow_id, data, dataflow_descriptor))
    }
}

/// Resolve the requested `data` selectors against a dataflow `descriptor` into a
/// concrete set of `node/output` topics.
///
/// Each selector is one of:
/// - empty list -> every output of every node in the dataflow;
/// - a bare node id (`camera`) -> every output declared by that node;
/// - a fully-qualified `node/output` (`camera/frame`) -> exactly that output.
///
/// A bare node id is handled explicitly here. It must not be turned into
/// `node/` and fed through [`InputMapping`] parsing: the segment after the
/// first `/` is parsed as a [`DataId`], which rejects the empty string, so
/// `"camera/".parse::<InputMapping>()` errors out — meaning the "all outputs of
/// a node" case would never be reached (dora-rs/dora, `dora topic <node>`).
fn resolve_topics(
    data: &[String],
    descriptor: &Descriptor,
) -> eyre::Result<BTreeSet<TopicIdentifier>> {
    let node_map = descriptor
        .nodes
        .iter()
        .map(|node| (&node.id, node))
        .collect::<HashMap<_, _>>();

    let unknown_node_err = |node: &NodeId| {
        format!(
            "unknown node `{}`\n\n  \
             hint: available nodes: {}",
            node,
            node_map
                .keys()
                .map(|k| k.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        )
    };

    let mut topics = BTreeSet::new();
    if data.is_empty() {
        topics.extend(descriptor.nodes.iter().flat_map(|node| {
            node_topic_outputs(node)
                .into_iter()
                .map(|output| TopicIdentifier {
                    node_id: node.id.clone(),
                    data_id: output,
                })
        }));
        return Ok(topics);
    }

    for s in data {
        // A bare node id (no `/`) selects every output of that node. Handle it
        // directly rather than synthesizing `node/` and relying on an empty
        // `DataId`, which the identifier validator rejects.
        if !s.contains('/') {
            let node_id: NodeId = s
                .parse()
                .wrap_err_with(|| format!("invalid node id `{s}`"))?;
            let node = *node_map
                .get(&node_id)
                .with_context(|| unknown_node_err(&node_id))?;
            topics.extend(
                node_topic_outputs(node)
                    .into_iter()
                    .map(|output| TopicIdentifier {
                        node_id: node_id.clone(),
                        data_id: output,
                    }),
            );
            continue;
        }

        match s.parse() {
            Ok(InputMapping::User(user)) => {
                let node = *node_map
                    .get(&user.source)
                    .with_context(|| unknown_node_err(&user.source))?;
                let outputs = node_topic_outputs(node);
                if outputs.contains(&user.output) {
                    topics.insert(TopicIdentifier {
                        node_id: user.source,
                        data_id: user.output,
                    });
                } else {
                    bail!(
                        "node `{}` does not have output `{}`\n\n  \
                         hint: available outputs: {}",
                        user.source,
                        user.output,
                        outputs
                            .iter()
                            .map(|o| o.to_string())
                            .collect::<Vec<_>>()
                            .join(", ")
                    );
                }
            }
            Ok(_) => {
                bail!("Reserved input mapping cannot be inspected")
            }
            Err(e) => bail!("Invalid output id `{s}`: {e}"),
        }
    }

    if topics.is_empty() {
        bail!(
            "no outputs found in this dataflow\n\n  \
             hint: ensure nodes in the dataflow declare `outputs` in their YAML definition"
        );
    }

    Ok(topics)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn descriptor() -> Descriptor {
        serde_yaml::from_str(
            "\
nodes:
  - id: camera
    path: camera
    outputs:
      - frame
      - status
  - id: sink
    path: sink
    inputs:
      image: camera/frame
",
        )
        .expect("parse descriptor")
    }

    fn topic(node: &str, output: &str) -> TopicIdentifier {
        TopicIdentifier {
            node_id: node.to_string().into(),
            data_id: output.to_string().into(),
        }
    }

    #[test]
    fn bare_node_id_expands_to_all_outputs() {
        // Regression: `dora topic <node>` (a bare node id) previously errored
        // with `Invalid output id `camera/`: identifier must not be empty`
        // because the code appended `/` and relied on an empty `DataId`.
        let topics = resolve_topics(&["camera".to_string()], &descriptor()).unwrap();
        assert_eq!(
            topics,
            BTreeSet::from([topic("camera", "frame"), topic("camera", "status")])
        );
    }

    #[test]
    fn qualified_output_selects_exactly_one() {
        let topics = resolve_topics(&["camera/frame".to_string()], &descriptor()).unwrap();
        assert_eq!(topics, BTreeSet::from([topic("camera", "frame")]));
    }

    #[test]
    fn empty_selector_returns_every_output() {
        let topics = resolve_topics(&[], &descriptor()).unwrap();
        assert_eq!(
            topics,
            BTreeSet::from([topic("camera", "frame"), topic("camera", "status")])
        );
    }

    #[test]
    fn unknown_bare_node_reports_hint() {
        let err = resolve_topics(&["ghost".to_string()], &descriptor()).unwrap_err();
        let msg = format!("{err:#}");
        assert!(msg.contains("unknown node `ghost`"), "unexpected: {msg}");
        assert!(msg.contains("available nodes"), "unexpected: {msg}");
    }

    #[test]
    fn unknown_output_reports_hint() {
        let err = resolve_topics(&["camera/nope".to_string()], &descriptor()).unwrap_err();
        let msg = format!("{err:#}");
        assert!(
            msg.contains("does not have output `nope`"),
            "unexpected: {msg}"
        );
    }

    #[test]
    fn node_topic_outputs_include_all_descriptor_node_kinds() {
        // The legacy `custom:` node key was removed from `Node`, so a plain
        // path node stands in for that case: what it pins is that a
        // non-operator node's outputs pass through unqualified.
        let descriptor: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: standard
    path: ./source
    outputs:
      - status
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
",
        )
        .expect("valid descriptor");

        let topics: Vec<String> = descriptor
            .nodes
            .iter()
            .flat_map(|node| {
                node_topic_outputs(node)
                    .into_iter()
                    .map(|output| format!("{}/{}", node.id, output))
            })
            .collect();

        assert_eq!(
            topics,
            vec!["standard/status", "single/image", "runtime/op/status"]
        );
    }

    /// The public id the selector hands out and the wire id the daemon reports
    /// a frame under must round-trip, in both directions and for all three node
    /// kinds. Without this the two never meet for a single-`operator:` node:
    /// `hz` indexed by the public id and matched frames by the wire id, so it
    /// reported 0 samples; `record --proxy` stored the wire id, which replay
    /// then could not send (dora-rs/dora#2893).
    #[test]
    fn public_and_wire_output_ids_round_trip() {
        let descriptor: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: standard
    path: standard
    outputs:
      - status
  - id: single
    operator:
      python: single.py
      outputs:
        - image
  - id: named
    operator:
      id: myop
      python: named.py
      outputs:
        - image
  - id: runtime
    operators:
      - id: op
        python: runtime.py
        outputs:
          - status
",
        )
        .expect("parse descriptor");
        let node = |id: &str| {
            descriptor
                .nodes
                .iter()
                .find(|n| n.id.to_string() == id)
                .expect("node in fixture")
                .clone()
        };
        let d = |s: &str| DataId::from(s.to_string());

        // A single `operator:` node is the only kind where the two differ: the
        // default operator id `op` is injected during resolution.
        assert_eq!(
            wire_topic_output_id(&node("single"), &d("image")),
            d("op/image")
        );
        assert_eq!(
            public_topic_output_id(&node("single"), &d("op/image")),
            d("image")
        );

        // An explicit operator id is used instead of the default.
        assert_eq!(
            wire_topic_output_id(&node("named"), &d("image")),
            d("myop/image")
        );
        assert_eq!(
            public_topic_output_id(&node("named"), &d("image")),
            d("image")
        );
        assert_eq!(
            public_topic_output_id(&node("named"), &d("myop/image")),
            d("image")
        );

        // Plain and multi-operator nodes pass through untouched -- `operators:`
        // outputs are already qualified on both sides.
        for (id, output) in [("standard", "status"), ("runtime", "op/status")] {
            assert_eq!(wire_topic_output_id(&node(id), &d(output)), d(output));
            assert_eq!(public_topic_output_id(&node(id), &d(output)), d(output));
        }

        // Every id the selector hands out must survive a full round-trip.
        for id in ["standard", "single", "named", "runtime"] {
            let n = node(id);
            for public in node_topic_outputs(&n) {
                let wire = wire_topic_output_id(&n, &public);
                assert_eq!(
                    public_topic_output_id(&n, &wire),
                    public,
                    "round-trip failed for {id}/{public}"
                );
            }
        }
    }

    /// `node.inputs` is empty for operator-backed nodes, so a subscriber
    /// listing built from it alone silently omits them.
    #[test]
    fn node_topic_inputs_include_operator_backed_consumers() {
        let descriptor: Descriptor = serde_yaml::from_str(
            "\
nodes:
  - id: camera
    path: camera
    outputs:
      - frame
  - id: detect
    operator:
      python: detect.py
      inputs:
        image: camera/frame
  - id: plot
    operators:
      - id: op
        python: plot.py
        inputs:
          image: camera/frame
  - id: log
    path: log
    inputs:
      image: camera/frame
",
        )
        .expect("parse descriptor");

        for id in ["detect", "plot", "log"] {
            let node = descriptor
                .nodes
                .iter()
                .find(|n| n.id.to_string() == id)
                .expect("node in fixture");
            let inputs = node_topic_inputs(node);
            assert_eq!(inputs.len(), 1, "{id} should have one input");
            assert_eq!(inputs[0].0.to_string(), "image", "{id}");
        }
    }
}
