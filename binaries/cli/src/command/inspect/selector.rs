use std::net::IpAddr;

use crate::{
    common::{connect_to_coordinator, resolve_dataflow_identifier},
    LOCALHOST,
};
use dora_core::{config::InputMapping, topics::DORA_COORDINATOR_PORT_CONTROL_DEFAULT};
use dora_message::{
    cli_to_coordinator::ControlRequest,
    coordinator_to_cli::ControlRequestReply,
    id::{DataId, NodeId},
};
use eyre::{bail, Context};
use uuid::Uuid;

#[derive(Debug, clap::Args)]
pub struct InspectSelector {
    /// Identifier of the dataflow
    #[clap(long, short, value_name = "UUID_OR_NAME")]
    pub dataflow: Option<String>,
    /// Data to inspect, e.g. `node_id/output_id`
    #[clap(value_name = "DATA")]
    pub data: Vec<String>,
    /// Address of the dora coordinator
    #[clap(long, value_name = "IP", default_value_t = LOCALHOST)]
    pub coordinator_addr: IpAddr,
    /// Port number of the coordinator control server
    #[clap(long, value_name = "PORT", default_value_t = DORA_COORDINATOR_PORT_CONTROL_DEFAULT)]
    pub coordinator_port: u16,
}

type DataflowId = Uuid;

impl InspectSelector {
    pub fn resolve(&self) -> eyre::Result<Vec<(DataflowId, NodeId, DataId)>> {
        let mut session =
            connect_to_coordinator((self.coordinator_addr, self.coordinator_port).into())
                .wrap_err("failed to connect to dora coordinator")?;
        let dataflow_id = resolve_dataflow_identifier(&mut *session, self.dataflow.as_deref())?;
        let dataflow_descriptor = {
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
                ControlRequestReply::DataflowInfo { descriptor, .. } => descriptor,
                ControlRequestReply::Error(err) => bail!("{err}"),
                other => bail!("unexpected list dataflow reply: {other:?}"),
            }
        };
        if !dataflow_descriptor.debug.publish_all_messages_to_zenoh {
            bail!(
                "Dataflow `{dataflow_id}` does not have `publish_all_messages_to_zenoh` enabled. You should enable it in order to inspect data.\n\
                \n\
                Tip; Add the following snipppet to your dataflow descriptor:\n\
                \n\
                ```\n\
                _unstable_debug:\n  publish_all_messages_to_zenoh: true\n\
                ```
                "
            );
        }
        let data = self
            .data
            .iter()
            .map(|s| {
                match serde_json::from_value::<InputMapping>(serde_json::Value::String(s.clone())) {
                    Ok(InputMapping::User(user)) => Ok((dataflow_id, user.source, user.output)),
                    Ok(_) => {
                        bail!("Reserved input mapping cannot be inspected")
                    }
                    Err(e) => bail!("Invalid output id `{s}`: {e}"),
                }
            })
            .collect::<eyre::Result<Vec<_>>>()?;
        Ok(data)
    }
}
