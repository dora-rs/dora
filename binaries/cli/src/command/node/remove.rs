use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use dora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::bail;

/// Remove a node from a running dataflow.
#[derive(Debug, clap::Args)]
pub struct Remove {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    /// Dataflow name or UUID to remove the node from
    #[clap(long, short = 'd', value_name = "NAME_OR_UUID")]
    dataflow: Option<String>,
    node: String,
    /// Grace period before force-killing the node (e.g. "30s" or seconds)
    #[clap(long, value_name = "DURATION", value_parser = crate::common::parse_grace_period)]
    grace: Option<std::time::Duration>,
}

impl Executable for Remove {
    fn execute(self) -> eyre::Result<()> {
        default_tracing()?;
        let session = self.coordinator.connect()?;
        let dataflow_id =
            resolve_dataflow_identifier_interactive(&session, self.dataflow.as_deref())?;
        let node_id: NodeId = self
            .node
            .parse()
            .map_err(|e| eyre::eyre!("invalid node ID: {e}"))?;

        let reply = send_control_request(
            &session,
            &ControlRequest::RemoveNode {
                dataflow_id,
                node_id: node_id.clone(),
                grace_duration: self.grace,
            },
        )?;

        match reply {
            ControlRequestReply::NodeRemoved { node_id, .. } => {
                println!("Node `{node_id}` removed from dataflow {dataflow_id}");
            }
            ControlRequestReply::Error(err) => bail!("failed to remove node: {err}"),
            other => bail!("unexpected reply: {other:?}"),
        }
        Ok(())
    }
}
