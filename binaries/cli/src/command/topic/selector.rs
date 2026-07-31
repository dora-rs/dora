use std::{
    borrow::Cow,
    collections::{BTreeSet, HashMap},
    fmt,
};

use crate::common::resolve_dataflow_identifier_interactive;
use crate::ws_client::WsSession;
use dora_core::{
    config::InputMapping,
    descriptor::{Descriptor, Node},
};
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

#[derive(Clone, PartialOrd, Ord, PartialEq, Eq)]
pub struct TopicIdentifier {
    pub node_id: NodeId,
    pub data_id: DataId,
}

impl fmt::Display for TopicIdentifier {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}", self.node_id, self.data_id)
    }
}

pub(crate) fn node_topic_outputs(node: &Node) -> BTreeSet<DataId> {
    let mut outputs = node.outputs.clone();

    if let Some(custom) = &node.custom {
        outputs.extend(custom.run_config.outputs.iter().cloned());
    }
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

        let node_map = dataflow_descriptor
            .nodes
            .iter()
            .map(|node| (&node.id, node))
            .collect::<HashMap<_, _>>();

        let mut data = BTreeSet::new();
        if self.data.is_empty() {
            data.extend(dataflow_descriptor.nodes.iter().flat_map(|node| {
                node_topic_outputs(node)
                    .into_iter()
                    .map(|output| TopicIdentifier {
                        node_id: node.id.clone(),
                        data_id: output,
                    })
            }));
            return Ok((dataflow_id, data, dataflow_descriptor));
        }

        for s in &self.data {
            let mut s = Cow::Borrowed(s.as_str());
            if !s.contains('/') {
                s.to_mut().push('/');
            }
            match s.parse() {
                Ok(InputMapping::User(user)) => {
                    let node = *node_map.get(&user.source).with_context(|| {
                        format!(
                            "unknown node `{}`\n\n  \
                             hint: available nodes: {}",
                            user.source,
                            node_map
                                .keys()
                                .map(|k| k.to_string())
                                .collect::<Vec<_>>()
                                .join(", ")
                        )
                    })?;
                    let outputs = node_topic_outputs(node);
                    if user.output.is_empty() {
                        data.extend(outputs.into_iter().map(|output| TopicIdentifier {
                            node_id: user.source.clone(),
                            data_id: output,
                        }));
                    } else if outputs.contains(&user.output) {
                        data.insert(TopicIdentifier {
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

        if data.is_empty() {
            bail!(
                "no outputs found in this dataflow\n\n  \
                 hint: ensure nodes in the dataflow declare `outputs` in their YAML definition"
            );
        }

        Ok((dataflow_id, data, dataflow_descriptor))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn node_topic_outputs_include_all_descriptor_node_kinds() {
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
  - id: legacy
    custom:
      path: legacy.py
      source: Local
      outputs:
        - buffer
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
            vec![
                "standard/status",
                "single/image",
                "legacy/buffer",
                "runtime/op/status",
            ]
        );
    }
}
