use std::{
    collections::{BTreeSet, HashMap},
    fmt,
};

use crate::common::resolve_dataflow_identifier_interactive;
use crate::ws_client::WsSession;
use dora_core::{config::InputMapping, descriptor::Descriptor};
use dora_message::{
    DataflowId,
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::ControlRequestReply,
    id::{DataId, NodeId},
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

impl TopicSelector {
    pub fn resolve(
        &self,
        session: &WsSession,
    ) -> eyre::Result<(DataflowId, BTreeSet<TopicIdentifier>)> {
        let (dataflow_id, topics, _descriptor) = self.resolve_with_descriptor(session)?;
        Ok((dataflow_id, topics))
    }

    /// Like [`Self::resolve`], but also returns the dataflow descriptor that
    /// was fetched to resolve the topics. Callers that additionally need the
    /// descriptor (e.g. to enumerate subscribers) should use this to avoid a
    /// second `Info` round-trip — and, when the dataflow is ambiguous, a second
    /// interactive selection prompt.
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
            node.outputs.iter().map(|output| TopicIdentifier {
                node_id: node.id.clone(),
                data_id: output.clone(),
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
            topics.extend(node.outputs.iter().map(|output| TopicIdentifier {
                node_id: node_id.clone(),
                data_id: output.clone(),
            }));
            continue;
        }

        match s.parse() {
            Ok(InputMapping::User(user)) => {
                let node = *node_map
                    .get(&user.source)
                    .with_context(|| unknown_node_err(&user.source))?;
                if node.outputs.contains(&user.output) {
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
                        node.outputs
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
}
