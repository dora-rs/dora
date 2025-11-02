use std::{borrow::Cow, collections::HashMap, net::IpAddr};

use crate::{
    LOCALHOST,
    common::{connect_to_coordinator, resolve_dataflow_identifier},
};
use communication_layer_request_reply::TcpRequestReplyConnection;
use dora_core::{
    config::InputMapping, descriptor::Descriptor, topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT,
};
use dora_message::{
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
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
}

impl DataflowSelector {
    pub fn resolve(&self) -> eyre::Result<(Box<TcpRequestReplyConnection>, Uuid, Descriptor)> {
        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .wrap_err("failed to connect to dora coordinator")?;
        let dataflow_id = resolve_dataflow_identifier(&mut *session, self.dataflow.as_deref())?;
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
            ControlRequestReply::DataflowInfo { descriptor, .. } => {
                Ok((session, dataflow_id, descriptor))
            }
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

#[derive(PartialOrd, Ord, PartialEq, Eq)]
pub struct TopicIdentifier {
    pub dataflow_id: Uuid,
    pub node_id: NodeId,
    pub data_id: DataId,
}

impl TopicSelector {
    pub fn resolve(&self) -> eyre::Result<(Box<TcpRequestReplyConnection>, Vec<TopicIdentifier>)> {
        let (session, dataflow_id, dataflow_descriptor) = self.dataflow.resolve()?;
        if !dataflow_descriptor.debug.publish_all_messages_to_zenoh {
            bail!(
                "Dataflow `{dataflow_id}` does not have `publish_all_messages_to_zenoh` enabled. You should enable it in order to inspect data.\n\
                \n\
                Tip: Add the following snipppet to your dataflow descriptor:\n\
                \n\
                ```\n\
                _unstable_debug:\n  publish_all_messages_to_zenoh: true\n\
                ```
                "
            );
        }

        let node_map = dataflow_descriptor
            .nodes
            .iter()
            .map(|node| (&node.id, node))
            .collect::<HashMap<_, _>>();

        let mut data = Vec::new();
        if self.data.is_empty() {
            data.extend(dataflow_descriptor.nodes.iter().flat_map(|node| {
                node.outputs.iter().map(|output| TopicIdentifier {
                    dataflow_id,
                    node_id: node.id.clone(),
                    data_id: output.clone(),
                })
            }));
            data.sort();
            return Ok((session, data));
        }

        for s in &self.data {
            let mut s = Cow::Borrowed(s.as_str());
            if !s.contains('/') {
                s.to_mut().push('/');
            }
            match serde_json::from_value::<InputMapping>(serde_json::Value::String(
                s.clone().into_owned(),
            )) {
                Ok(InputMapping::User(user)) => {
                    let node = *node_map
                        .get(&user.source)
                        .with_context(|| format!("Unknown node: `{}`", user.source))?;
                    if user.output.is_empty() {
                        data.extend(node.outputs.iter().map(|output| TopicIdentifier {
                            dataflow_id,
                            node_id: user.source.clone(),
                            data_id: output.clone(),
                        }));
                    } else if node.outputs.contains(&user.output) {
                        data.push(TopicIdentifier {
                            dataflow_id,
                            node_id: user.source,
                            data_id: user.output,
                        });
                    } else {
                        bail!(
                            "Node `{}` does not have output `{}`",
                            user.source,
                            user.output
                        );
                    }
                }
                Ok(_) => {
                    bail!("Reserved input mapping cannot be inspected")
                }
                Err(e) => bail!("Invalid output id `{s}`: {e}"),
            }
        }

        data.sort();
        data.dedup();

        Ok((session, data))
    }
}
