use crate::{
    command::{Executable, default_tracing},
    common::{CoordinatorOptions, resolve_dataflow_identifier_interactive, send_control_request},
};
use adora_message::{
    cli_to_coordinator::ControlRequest, coordinator_to_cli::ControlRequestReply, id::NodeId,
};
use eyre::bail;

/// Remove a node from a running dataflow.
#[derive(Debug, clap::Args)]
pub struct Remove {
    #[clap(flatten)]
    coordinator: CoordinatorOptions,
    #[clap(long)]
    dataflow: Option<String>,
    node: String,
    /// Grace period in seconds before force-killing the node.
    #[clap(long)]
    grace: Option<f64>,
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
                grace_duration: self.grace.map(std::time::Duration::from_secs_f64),
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
